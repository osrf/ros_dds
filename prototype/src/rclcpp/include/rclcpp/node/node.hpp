#ifndef NODE_HPP
#define NODE_HPP
#include <map>

#include <ccpp_dds_dcps.h>

#include <rclcpp/publisher/publisher.hpp>
#include <rclcpp/subscription/subscription.hpp>

// TODO: use something less generic
#include "ccpp_ROSMsg.h"

namespace rclcpp
{
    using publisher::Publisher;
    using subscription::Subscription;

    namespace node
    {
        class Node
        {
        public:
            Node(std::string name);
            ~Node();

            Publisher create_publisher(std::string topic_name, size_t queue_size);

            void destroy_publisher(Publisher publisher);
            void destroy_publisher(std::string topic_name);

            template <typename T>
            Subscription<T> create_subscription(std::string topic_name,
                                                size_t queue_size,
                                                typename Subscription<T>::CallbackType cb)
            {
                // TODO check return status
                DDS::ReturnCode_t status;
   
                // TODO define partiion name in common location
                std::string partition_name = "ros_partition";
   
                DDS::TopicQos default_topic_qos;
                status = this->participant_->get_default_topic_qos(default_topic_qos);
                default_topic_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
   
                ROSMessageTypeSupport_var ros_message_ts = new ROSMessageTypeSupport();
                char * ros_message_type_name = ros_message_ts->get_type_name();
                status = ros_message_ts->register_type(
                    this->participant_.in(), ros_message_type_name);
   
                DDS::SubscriberQos subscriber_qos;
                status = this->participant_->get_default_subscriber_qos(subscriber_qos);
                subscriber_qos.partition.name.length(1);
                subscriber_qos.partition.name[0] = partition_name.c_str();
   
                DDS::Subscriber_var dds_subscriber = this->participant_->create_subscriber(
                    subscriber_qos, NULL, DDS::STATUS_MASK_NONE);
   
                DDS::Topic_var ros_message_topic = this->participant_->create_topic(
                    topic_name.c_str(), ros_message_type_name, default_topic_qos, NULL,
                    DDS::STATUS_MASK_NONE
                );
   
                DDS::DataReader_var topic_reader = dds_subscriber->create_datareader(
                    ros_message_topic.in(), DATAREADER_QOS_USE_TOPIC_QOS,
                    NULL, DDS::STATUS_MASK_NONE);
   
                ROSMessageDataReader_var data_reader = ROSMessageDataReader::_narrow(topic_reader.in());

                return Subscription<T>(data_reader, cb);
            };

            template <typename T>
            void destroy_subscription(Subscription<T> subscription);
        private:
            std::string name_;
            DDS::DomainParticipantFactory_var dpf_;
            DDS::DomainParticipant_var participant_;

            std::map<std::string, Publisher> publishers_;
        };
    }
}
#endif
