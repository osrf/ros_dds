#include <rclcpp/rclcpp.hpp>

#include <std_msgs/Int32.h>
#include <std_msgs/AddTwoIntsRequest.h>
#include <std_msgs/AddTwoIntsResponse.h>

#include "std_msgs/dds_impl/Int32_convert.h"
#include "std_msgs/dds_impl/AddTwoIntsRequest_convert.h"
#include "std_msgs/dds_impl/AddTwoIntsResponse_convert.h"

#include <iostream>

bool add(const std_msgs::AddTwoIntsRequest &req, std_msgs::AddTwoIntsResponse& res)
{
    std::cout << "Incoming request from client_id(" << req.client_id << ") req.a(" << req.a << "), req.b(" << req.b << "), req.req_id(" << req.req_id << ")" << std::endl;
    res.sum = req.a + req.b;
    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::Ptr node = rclcpp::create_node("add_two_ints_server");
    rclcpp::Service<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>::Ptr service = node->create_service<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>("add_two_ints", add);
    node->spin();
    return 0;
}
