#ifndef RCLCPP_RCLCPP_PUBLISHER_PUBLISHER_HPP_
#define RCLCPP_RCLCPP_PUBLISHER_PUBLISHER_HPP_
#include <exception>
#include <mutex>
#include <string>

#include <ccpp.h>

#include <genidlcpp/resolver.h>

#include <rclcpp/impl/check_status.hpp>
#include <rclcpp/subscription/subscription.hpp>

namespace rclcpp
{

// Forward declare Node class for friendship with Publisher Interface
namespace node {class Node;}

namespace publisher
{

class DuplicatePublisherException : public std::exception
{
    virtual const char* what() const throw()
    {
        return "Publisher for topic already exists";
    }
};

class PublisherInterface
{
public:
    typedef std::shared_ptr<PublisherInterface> Ptr;
    virtual std::string get_topic_name() = 0;

};

template <typename ROSMsgType>
class Publisher : public PublisherInterface
{
    typedef dds_impl::DDSTypeResolver<ROSMsgType> r;
    typedef std::shared_ptr<
        std::map<
            std::string, std::shared_ptr<
                std::list<subscription::SubscriptionInterface::Ptr>
            >
        >
    > shared_subscriptions;

    friend class node::Node;
    Publisher(std::string topic_name, size_t queue_size,
              DDS::Publisher_var dds_publisher,
              DDS::Topic_var dds_topic,
              DDS::DataWriter_var dds_topic_datawriter,
              shared_subscriptions subscriptions_queues)
    : topic_name_(topic_name), queue_size_(queue_size),
      dds_publisher_(dds_publisher), dds_topic_(dds_topic),
      dds_topic_datawriter_(dds_topic_datawriter), subscriptions_queues_(subscriptions_queues)
    {
        this->data_writer_ = r::DDSMsgDataWriterType::_narrow(this->dds_topic_datawriter_.in());
        checkHandle(this->data_writer_, "DDSMsgDataWriter_t::_narrow");
    }

    Publisher(const Publisher &) = delete;
public:
    typedef std::shared_ptr<Publisher<ROSMsgType> > Ptr;
    ~Publisher() {}

    void publish(const ROSMsgType &msg)
    {
        typename r::DDSMsgType dds_msg;
        dds_impl::DDSTypeResolver<ROSMsgType>::convert_ros_message_to_dds(msg, dds_msg);
        DDS::InstanceHandle_t instance_handle = this->data_writer_->register_instance(dds_msg);
        DDS::ReturnCode_t status = this->data_writer_->write(dds_msg, instance_handle);
        checkStatus(status, "DDSMsgDataWriter_t::write");
    }

    void publish(typename ROSMsgType::Ptr msg)
    {
        std::lock_guard<std::mutex> lock(queues_mutex_);

        auto subscriptions_queues = this->subscriptions_queues_->find(this->topic_name_);
        if(subscriptions_queues != this->subscriptions_queues_->end()) {
            auto subscriptions_list = subscriptions_queues->second;
            for(auto it = subscriptions_list->begin(); it != subscriptions_list->end(); ++it) {
                auto subscription = std::dynamic_pointer_cast< typename subscription::Subscription<ROSMsgType> >(*it);
                subscription->consume(msg);
            }
        }
    }

    void publish(typename ROSMsgType::ConstPtr msg)
    {
        this->publish(*msg);
    }

    std::string get_topic_name()
    {
        return this->topic_name_;
    }

private:
    std::string topic_name_;
    size_t queue_size_;

    DDS::Publisher_var dds_publisher_;
    DDS::Topic_var dds_topic_;
    DDS::DataWriter_var dds_topic_datawriter_;

    typename r::DDSMsgDataWriterType_var data_writer_;

    std::shared_ptr<
        std::map<
            std::string, std::shared_ptr<
                std::list<subscription::SubscriptionInterface::Ptr>
            >
        >
    > subscriptions_queues_;

    std::mutex queues_mutex_;
};

}

}

#endif /* RCLCPP_RCLCPP_PUBLISHER_PUBLISHER_HPP_ */
