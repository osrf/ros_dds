#include <ros/ros.h>
#include <large_message_cpp/LargeMessage.h>


void cb(const large_message_cpp::LargeMessageConstPtr msg)
{
    std::cout << "[" << msg->seq << "]: " << msg->content.length() << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("big_chatter", 0, cb);

    ros::spin();

    return 0;
}
