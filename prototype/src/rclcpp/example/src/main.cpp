#include <rclcpp/rclcpp.hpp>

#include <std_msgs/Int32.h>
#include "std_msgs/dds_impl/Int32_convert.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("example");

    typedef dds_impl::DDSTypeResolver<std_msgs::Int32> r;
    rclcpp::Publisher<std_msgs::Int32, r::DDSMsgType, r::DDSMsgDataWriterType_var, r::DDSMsgDataWriterType> publisher = node.create_publisher<std_msgs::Int32, r::DDSMsgTypeSupportType, r::DDSMsgType, r::DDSMsgDataWriterType_var, r::DDSMsgDataWriterType>("temp", 0);

    std_msgs::Int32 msg;
    msg.data = 2;
    publisher.publish(msg);
}
