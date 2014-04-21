#include <rclcpp/rclcpp.hpp>

#include <rclcpp_examples/AddTwoInts.h>

#include <std_msgs/Int32.h>
#include <std_msgs/impl/Int32_pubsub.hpp>

#include "rclcpp_examples/AddTwoIntsRequest.h"
#include "rclcpp_examples/impl/AddTwoIntsRequest_pubsub.hpp"
#include "rclcpp_examples/AddTwoIntsResponse.h"
#include "rclcpp_examples/impl/AddTwoIntsResponse_pubsub.hpp"

#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::Ptr node = rclcpp::create_node("add_two_ints_client");

    auto client = node->create_client<rclcpp_examples::AddTwoInts>("add_two_ints");

    rclcpp_examples::AddTwoIntsRequest req;
    req.a = 2;
    req.b = 3;

    auto response = client->call(req);
    std::cout << "Sum: " << response->sum << std::endl;

    return 0;
}
