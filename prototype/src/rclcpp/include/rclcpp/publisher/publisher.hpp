#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP
#include <string>

#include "ccpp_ROSMsg.h"

namespace rclcpp
{
    namespace publisher
    {
        class Publisher
        {
        public:
            Publisher(std::string topic_name, size_t queue_size,
                DDS::Publisher_var publisher, DDS::Topic_var ros_message_topic,
                DDS::DataWriter_var topic_writer,
                ROSMessageDataWriter_var data_writer) : topic_name_(topic_name),
                queue_size_(queue_size), publisher_(publisher),
                ros_message_topic_(ros_message_topic), topic_writer_(topic_writer),
                data_writer_(data_writer) {};
            ~Publisher() {};

            template <typename T>
            void publish(T msg);

            std::string get_topic_name()
            {
                return this->topic_name_;
            }
        private:
            std::string topic_name_;
            size_t queue_size_;

            DDS::Publisher_var publisher_;
            DDS::Topic_var ros_message_topic_;
            DDS::DataWriter_var topic_writer_;
            ROSMessageDataWriter_var data_writer_;
        };
    }
}
#endif
