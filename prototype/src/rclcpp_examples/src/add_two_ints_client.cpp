#include <rclcpp/rclcpp.hpp>

#include <std_msgs/String.h>
#include "std_msgs/dds_impl/String_convert.h"

#include <std_msgs/Int32.h>
#include <std_msgs/AddTwoIntsRequest.h>
#include <std_msgs/AddTwoIntsResponse.h>

#include "std_msgs/dds_impl/Int32_convert.h"
#include "std_msgs/dds_impl/AddTwoIntsRequest_convert.h"
#include "std_msgs/dds_impl/AddTwoIntsResponse_convert.h"

#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::Ptr node = rclcpp::create_node("add_two_ints_server");
    auto client = node->create_client<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>("add_two_ints");
    std_msgs::AddTwoIntsRequest req;
    req.a = 2;
    req.b = 3;

    auto response = client->call(req);
    std::cout << "Sum: " << response->sum << std::endl;   

    return 0;
}
