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
    rclcpp::Client<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>::Ptr client = node->create_client<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>("add_two_ints");
    std_msgs::AddTwoIntsRequest req;
    req.a = 2;
    req.b = 3;

    typename rclcpp::Client<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>::shared_future f;

    f = client->async_call(req);
    std_msgs::AddTwoIntsResponse response = f.get();
    std::cout << "Sum: " << response.sum << std::endl;

    node->spin();
    return 0;
}
