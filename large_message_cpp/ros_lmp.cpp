#include <ros/ros.h>
#include <large_message_cpp/LargeMessage.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<large_message_cpp::LargeMessage>("big_chatter", 0);

    ROS_INFO("Sending data...");
    for (int scale = 16; scale < 29; scale += 2)
    {
        for (int i = 0; i < 10; ++i)
        {
            large_message_cpp::LargeMessage msg;

            msg.seq = i;
            msg.content = std::string(pow(2, scale), '.').c_str();  // ~8.39 million characters

            ROS_INFO_STREAM("Sending scale " << scale << " iteration " << i);

            chatter_pub.publish(msg);
            ros::spinOnce();

            ros::Duration(0.1).sleep();
        }
    }

    ros::Duration(1.0).sleep();

    return 0;
}
