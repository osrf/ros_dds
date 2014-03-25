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

void process_future(typename rclcpp::Client<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>::shared_future f)
{
    std_msgs::AddTwoIntsResponse::ConstPtr response = f.get();
    std::cout << "Sum: " << response->sum << std::endl;   
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::Ptr node = rclcpp::create_node("add_two_ints_server");
    auto client = node->create_client<std_msgs::AddTwoIntsRequest, std_msgs::AddTwoIntsResponse>("add_two_ints");
    std_msgs::AddTwoIntsRequest req;
    req.a = 2;
    req.b = 3;

    auto f = client->async_call(req);
    // Need to spawn a separate thread because std::future<T>::get blocks
    // This could be remedied by using coroutines and boost.asio, but
    // the version of boost shipped with Ubuntu 12.04 is too old
    std::thread t(process_future, f);

    node->spin();
    t.join();
    return 0;
}
