#include <rclcpp/rclcpp.hpp>

#include <rclcpp_examples/AddTwoInts.h>

#include <std_msgs/Int32.h>
#include <rclcpp_examples/AddTwoIntsRequest.h>
#include <rclcpp_examples/AddTwoIntsResponse.h>

#include "rclcpp_examples/dds_impl/AddTwoIntsRequest_convert.h"
#include "rclcpp_examples/dds_impl/AddTwoIntsResponse_convert.h"

#include <iostream>

bool add(rclcpp_examples::AddTwoIntsRequest::ConstPtr req, rclcpp_examples::AddTwoIntsResponse::Ptr res)
{
    std::cout << "Incoming request from client_id(" << req->client_id << ") req.a(" << req->a << "), req.b(" << req->b << "), req.req_id(" << req->req_id << ")" << std::endl;
    res->sum = req->a + req->b;
    return true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::Ptr node = rclcpp::create_node("add_two_ints_server");
    rclcpp::Service<rclcpp_examples::AddTwoInts>::Ptr service = node->create_service<rclcpp_examples::AddTwoInts>("add_two_ints", add);
    node->spin();
    return 0;
}
