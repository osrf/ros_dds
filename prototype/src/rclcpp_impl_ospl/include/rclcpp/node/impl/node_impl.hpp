#include <list>
#include <map>
#include <string>

#include <ccpp_dds_dcps.h>

#include <rclcpp/node/node.hpp>
#include <rclcpp/publisher/publisher.hpp>
#include <rclcpp/subscription/subscription.hpp>

#include <rclcpp/impl/check_status.hpp>

namespace rclcpp
{
namespace node
{
namespace impl
{

class NodeImpl
{
    friend class rclcpp::node::Node;

    DDS::DomainParticipantFactory_var dpf_;
    DDS::DomainParticipant_var participant_;
    DDS::TopicQos default_topic_qos_;
    DDS::PublisherQos default_publisher_qos_;
    DDS::SubscriberQos default_subscriber_qos_;
    DDS::WaitSet * waitset_;

    std::map<rclcpp::subscription::Subscription::Ptr, DDS::Condition_ptr> status_conditions_;

public:
    NodeImpl(const std::string &node_name)
    {
        this->dpf_ = DDS::DomainParticipantFactory::get_instance();
        checkHandle(this->dpf_.in(), "DDS::DomainParticipantFactory::get_instance");
        DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;

        this->participant_ = this->dpf_->create_participant(
            domain, PARTICIPANT_QOS_DEFAULT, NULL,
            DDS::STATUS_MASK_NONE);
        checkHandle(this->participant_.in(), "DDS::DomainParticipantFactory::create_participant");

        // Create the default QoS for Topics
        DDS::ReturnCode_t status = this->participant_->get_default_topic_qos(this->default_topic_qos_);
        checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
        this->default_topic_qos_.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

        // Create the default QoS for Publishers
        status = this->participant_->get_default_publisher_qos(this->default_publisher_qos_);
        checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
        this->default_publisher_qos_.partition.name.length(1);
        this->default_publisher_qos_.partition.name[0] = "ros_partition";

        // Create the default QoS for Subscribers
        status = this->participant_->get_default_subscriber_qos(this->default_subscriber_qos_);
        checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
        this->default_subscriber_qos_.partition.name.length(1);
        this->default_subscriber_qos_.partition.name[0] = "ros_partition";

        // Create a waitset for spin
        this->waitset_ = new DDS::WaitSet();
    }

    ~NodeImpl()
    {
        delete this->waitset_;
        this->dpf_->delete_participant(this->participant_);
    }

    void spin(Node * parent)
    {
        while(parent->running_)
        {
            DDS::ConditionSeq active_condition_seq;
            DDS::Duration_t timeout = {1, 0};
            DDS::ReturnCode_t retcode = this->waitset_->wait(active_condition_seq, timeout);

            if (active_condition_seq.length() == 0)
            {
                continue;
            }

            // For each subscription
            for (auto it = parent->subscriptions_.begin(); it != parent->subscriptions_.end(); ++it)
            {
                // Check each active condition
                for (int i = 0; i < active_condition_seq.length(); ++i)
                {
                    // To see if the subscriptions status condition matches
                    if (this->status_conditions_.at((*it)) == active_condition_seq[i])
                    {
                        // If so, spin as many as it can right now
                        (*it)->spin_some();
                    }
                }
            }
        }
    }

    template <typename ROSMsgType>
    rclcpp::publisher::Publisher::Ptr create_publisher(
        const std::string &topic_name,
        size_t queue_size
    );

    template <typename ROSMsgType>
    rclcpp::subscription::Subscription::Ptr create_subscription(
        const std::string &topic_name,
        size_t queue_size,
        std::function<void(typename ROSMsgType::ConstPtr)> callback
    );

};

}
}
}
