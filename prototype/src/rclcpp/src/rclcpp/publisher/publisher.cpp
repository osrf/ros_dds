#include <rclcpp/publisher/publisher.hpp>

namespace rclcpp
{
    namespace publisher
    {
        template <> void rclcpp::publisher::Publisher::publish<std::string>(std::string msg)
        {
            ROSMessage_var ros_message = new ROSMessage();
            ros_message->data = msg.c_str();

            DDS::InstanceHandle_t instance_handle = this->data_writer_->register_instance(*ros_message);

            DDS::ReturnCode_t status = this->data_writer_->write(*ros_message, instance_handle);
        }
    }
}
