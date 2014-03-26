#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <rclcpp_examples/AddTwoInts.h>

#include <std_msgs/String.h>
#include "std_msgs/dds_impl/String_convert.h"

#include <std_msgs/Int32.h>
#include <rclcpp_examples/AddTwoIntsRequest.h>
#include <rclcpp_examples/AddTwoIntsResponse.h>

#include "std_msgs/dds_impl/Int32_convert.h"
#include "rclcpp_examples/dds_impl/AddTwoIntsRequest_convert.h"
#include "rclcpp_examples/dds_impl/AddTwoIntsResponse_convert.h"

#include <rclcpp/parameter/client.hpp>

#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::Ptr node = rclcpp::create_node("parameter_client");
    auto parameter_client = node->create_parameter_client();

    std::string request1 = "rocks";
    parameter_client->set_param("ros", request1);

    std::string response1;
    parameter_client->get_param("ros", response1);
    std::cout << "ros " << response1 << std::endl;

    int64_t request2 = 42;
    parameter_client->set_param("ros", request2);

    int64_t response2;
    parameter_client->get_param("ros", response2);
    std::cout << "ros " << response2 << std::endl;

    bool request3 = true;
    parameter_client->set_param("ros", request3);

    bool response3;
    parameter_client->get_param("ros", response3);
    std::cout << "ros " << response3 << std::endl;

    return 0;
}
