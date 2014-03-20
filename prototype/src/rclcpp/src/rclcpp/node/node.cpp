#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node/node.hpp>
#include <ccpp_dds_dcps.h>
//#include <boost/thread.hpp>

using namespace rclcpp::node;
using namespace rclcpp::publisher;

Node::Node(std::string name)
{
    this->name_ = name;
    this->dpf_ = DDS::DomainParticipantFactory::get_instance();
    // checkHandle(this->dpf_.in(), "DDS::DomainParticipantFactory::get_instance");
    DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;

    this->participant_ = this->dpf_->create_participant(
        domain, PARTICIPANT_QOS_DEFAULT, NULL,
        DDS::STATUS_MASK_NONE);
    // checkHandle(this->participant_.in(), "DDS::DomainParticipantFactory::create_participant");

    // Create the default QoS for Topics
    DDS::ReturnCode_t status = this->participant_->get_default_topic_qos(this->default_topic_qos_);
    // checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
    this->default_topic_qos_.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

    // Create the default QoS for Publishers
    status = this->participant_->get_default_publisher_qos(this->default_publisher_qos_);
    // checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    this->default_publisher_qos_.partition.name.length(1);
    this->default_publisher_qos_.partition.name[0] = "ros_partition";

    // Create the default QoS for Subscribers
    status = this->participant_->get_default_subscriber_qos(this->default_subscriber_qos_);
    // checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
    this->default_subscriber_qos_.partition.name.length(1);
    this->default_subscriber_qos_.partition.name[0] = "ros_partition";

    this->subscription_watcher_th = new boost::thread(boost::bind(&Node::subscription_watcher, this));
}

void Node::subscription_watcher()
{
    while(true)
    {
        std::list<rclcpp::SubscriptionInterface *>::const_iterator iterator;
        for (iterator = this->subscriptions_.begin(); iterator != this->subscriptions_.end(); ++iterator) {
            (*iterator)->spin();
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    }
}

void Node::wait()
{
    this->subscription_watcher_th->join();
}

Node::~Node() {
    this->dpf_->delete_participant(this->participant_);
    delete this->subscription_watcher_th;
}

void Node::destroy_publisher(PublisherInterface * publisher)
{
    this->destroy_publisher(publisher->get_topic_name());
}

void Node::destroy_publisher(std::string topic_name)
{
    if (this->publishers_.find(topic_name) == this->publishers_.end())
    {
        // TODO Raise, topic not in list of publishers
    }
    this->publishers_.erase(topic_name);
}
