#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <string>

#include <ccpp.h>

#include <genidlcpp/resolver.h>

// #include <rclcpp/impl/check_status.h>

namespace rclcpp
{

// Forward declare Node class for friendship with Publisher Interface
namespace node {class Node;}

namespace publisher
{

class PublisherInterface
{
public:
    PublisherInterface(std::string topic_name, size_t queue_size,
                       DDS::Publisher_var dds_publisher,
                       DDS::Topic_var dds_topic,
                       DDS::DataWriter_var dds_topic_datawriter)
    : topic_name_(topic_name), queue_size_(queue_size),
      dds_publisher_(dds_publisher), dds_topic_(dds_topic),
      dds_topic_datawriter_(dds_topic_datawriter)
    {}
    ~PublisherInterface() {}

    std::string get_topic_name()
    {
        return this->topic_name_;
    }

protected:
    std::string topic_name_;
    size_t queue_size_;

    DDS::Publisher_var dds_publisher_;
    DDS::Topic_var dds_topic_;
    DDS::DataWriter_var dds_topic_datawriter_;

};

template <typename ROSMsgType>
class Publisher : public PublisherInterface
{
private:
    typedef typename dds_impl::DDSTypeResolver<ROSMsgType>::DDSMsgType DDSMsg_t;
    typedef typename dds_impl::DDSTypeResolver<ROSMsgType>::DDSMsgType DDSMsg_var;
    typedef typename dds_impl::DDSTypeResolver<ROSMsgType>::DDSMsgTypeSupportType DDSMsgTypeSupport_t;
    typedef typename dds_impl::DDSTypeResolver<ROSMsgType>::DDSMsgTypeSupportType_var DDSMsgTypeSupport_var;
    typedef typename dds_impl::DDSTypeResolver<ROSMsgType>::DDSMsgDataWriterType DDSMsgDataWriter_t;
    typedef typename dds_impl::DDSTypeResolver<ROSMsgType>::DDSMsgDataWriterType_var DDSMsgDataWriter_var;
    friend class node::Node;
    Publisher(std::string topic_name, size_t queue_size,
              DDS::Publisher_var dds_publisher,
              DDS::Topic_var dds_topic,
              DDS::DataWriter_var dds_topic_datawriter)
    : PublisherInterface(topic_name, queue_size, dds_publisher, dds_topic, dds_topic_datawriter)
    {
        this->data_writer_ = DDSMsgDataWriter_t::_narrow(this->dds_topic_datawriter_.in());
        // TODO: Check result of _narrow
    }
public:
    typedef boost::shared_ptr< Publisher<ROSMsgType> > shared_publisher;
    ~Publisher() {}

    void publish(const ROSMsgType &msg)
    {
        DDSMsg_t dds_msg;
        dds_impl::DDSTypeResolver<ROSMsgType>::convert_ros_message_to_dds(msg, dds_msg);
        DDS::InstanceHandle_t instance_handle = this->data_writer_->register_instance(dds_msg);
        DDS::ReturnCode_t status = this->data_writer_->write(dds_msg, instance_handle);
        // checkStatus(status, "DDSMsgDataWriter_t::write");
    }

private:
    DDSMsgDataWriter_var data_writer_;
};

}
}
#endif
