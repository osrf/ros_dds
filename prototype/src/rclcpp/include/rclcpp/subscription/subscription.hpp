#ifndef RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_
#define RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>

#include <ccpp_dds_dcps.h>
#include <dds_dcps.h>

#include <genidlcpp/resolver.h>

namespace rclcpp
{

// Forward declare Node class for friendship with Subscription Interface
namespace node {class Node;}

namespace subscription
{

class SubscriptionInterface
{
public:
    typedef std::shared_ptr<SubscriptionInterface> Ptr;
    virtual bool spin_once() = 0;
    virtual bool spin_some() = 0;
    virtual DDS::StatusCondition * get_status_condition() = 0;
    virtual std::string get_topic_name() const = 0;
};

template <typename ROSMsgType>
class Subscription : public SubscriptionInterface
{
public:
    typedef std::function<void(typename ROSMsgType::ConstPtr)> CallbackType;
    typedef std::shared_ptr<Subscription<ROSMsgType> > Ptr;
private:
    typedef dds_impl::DDSTypeResolver<ROSMsgType> r;
    friend class node::Node;

    Subscription(const std::string &topic_name, typename r::DDSMsgDataReaderType_var data_reader, CallbackType cb)
    : topic_name_(topic_name), data_reader_(data_reader), cb_(cb)
    {
        this->condition_ = this->data_reader_->get_statuscondition();
        this->condition_->set_enabled_statuses(DDS::DATA_AVAILABLE_STATUS);
    }

    bool spin_(DDS::ULong read_length)
    {
        typename ROSMsgType::Ptr ros_msg;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);

            while(!(internal_queue_.empty())) {
                ros_msg = internal_queue_.front();
                internal_queue_.pop();
                this->cb_(ros_msg);
            }
        }

        typename r::DDSMsgSeqType_var dds_msg_seq = new typename r::DDSMsgSeqType();
        DDS::SampleInfoSeq_var sample_info_seq = new DDS::SampleInfoSeq();
        this->data_reader_->take(
            dds_msg_seq,
            sample_info_seq,
            read_length,
            DDS::ANY_SAMPLE_STATE,
            DDS::ANY_VIEW_STATE,
            DDS::ALIVE_INSTANCE_STATE
        );

        bool result = true;
        if (dds_msg_seq->length() == 0)
        {
            result = false;
        }

        for (DDS::ULong i = 0; i < dds_msg_seq->length(); i++)
        {
            typename ROSMsgType::Ptr ros_msg(new ROSMsgType());
            dds_impl::DDSTypeResolver<ROSMsgType>::convert_dds_message_to_ros(dds_msg_seq[i], (*ros_msg.get()));
            try {
                this->cb_(ros_msg);
            } catch (const std::exception& e) {
                std::cerr << "Error handling callback for subscription to topic '" << this->topic_name_ << "':"
                          << std::endl << e.what() << std::endl;
            }
        }
        this->data_reader_->return_loan(dds_msg_seq, sample_info_seq);
        return result;
   }
public:
    ~Subscription() {}

    DDS::StatusCondition * get_status_condition()
    {
        return this->condition_;
    }

    bool spin_once()
    {
        return this->spin_(1);
    }

    bool spin_some()
    {
        return this->spin_(DDS::LENGTH_UNLIMITED);
    }

    void consume(typename ROSMsgType::Ptr msg) {
        this->internal_queue_.push(msg);
    }

    std::string get_topic_name() const {
        return this->topic_name_;
    }

private:
    typename r::DDSMsgDataReaderType_var data_reader_;
    CallbackType cb_;
    std::string topic_name_;
    DDS::StatusCondition * condition_;
    std::mutex queue_mutex_;
    std::queue<typename ROSMsgType::Ptr> internal_queue_;
};

class SubscriptionManager {
private:
    std::list<SubscriptionInterface::Ptr> subscriptions_;

public:
    typedef std::shared_ptr<SubscriptionManager> Ptr;

    void add(SubscriptionInterface::Ptr subscription_interface) {
        this->subscriptions_.push_back(subscription_interface);
    }

    void remove(SubscriptionInterface::Ptr subscription_interface) {
        this->subscriptions_.remove(subscription_interface);
    }

    template <typename ROSMsgType>
    void consume(const std::string &topic_name, typename ROSMsgType::Ptr msg) {
        for(auto it = this->subscriptions_.begin(); it != this->subscriptions_.end(); ++it) {
            if((*it)->get_topic_name() == topic_name) {
                auto sub = std::dynamic_pointer_cast< Subscription<ROSMsgType> >(*it);
                sub->consume(msg);
            }
        }
    }
};

}

}

#endif /* RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_ */
