#include <rclcpp/rclcpp.hpp>

#include <std_msgs/Int32.h>
#include <std_msgs/AddTwoIntsRequest.h>
#include <std_msgs/AddTwoIntsResponse.h>

#include "std_msgs/dds_impl/Int32_convert.h"
#include "std_msgs/dds_impl/AddTwoIntsRequest_convert.h"
#include "std_msgs/dds_impl/AddTwoIntsResponse_convert.h"

#include <iostream>

bool add(std_msgs::AddTwoIntsRequest &req, std_msgs::AddTwoIntsResponse &res)
{
    res.sum = req.a + req.b;
    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node node = rclcpp::create_node("add_two_ints_server");
    rclcpp::Service<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse> service = node.create_service<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>("add_two_ints", add);
    node.wait();
    return 0;
}
