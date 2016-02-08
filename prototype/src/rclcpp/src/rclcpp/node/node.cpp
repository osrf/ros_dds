#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node/node.hpp>
#include <ccpp_dds_dcps.h>

#include <iostream>

#include <signal.h>

using namespace rclcpp::node;
using namespace rclcpp::publisher;

Node::Node(std::string name, subscription::SubscriptionManager::Ptr subscription_manager)
: running_(true), subscription_manager_(subscription_manager)
{
    this->nodes_.push_back(this);
    this->subscription_iterator_ = this->subscriptions_.end();
    this->name_ = name;
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

    // Create a parameter server for this node
    this->create_parameter_server(name);
}

Node::~Node()
{
    this->dpf_->delete_participant(this->participant_);
    delete this->waitset_;
}

void Node::spin()
{
    while(this->running_)
    {
        DDS::ConditionSeq active_condition_seq;
        DDS::Duration_t timeout = {1, 0};
        DDS::ReturnCode_t retcode = this->waitset_->wait(active_condition_seq, timeout);

        if (active_condition_seq.length() == 0)
        {
            continue;
        }

        // For each subscription
        for (auto it = this->subscriptions_.begin(); it != this->subscriptions_.end(); ++it)
        {
            // Check each active condition
            for (int i = 0; i < active_condition_seq.length(); ++i)
            {
                // To see if the subscriptions status condition matches
                if ((*it)->get_status_condition() == active_condition_seq[i])
                {
                    // If so, spin as many as it can right now
                    (*it)->spin_some();
                }
            }
        }
    }
}

bool Node::spin_once()
{
    if (this->subscription_iterator_ == this->subscriptions_.end())
    {
        this->subscription_iterator_ = this->subscriptions_.begin();
    }
    bool did_spin;
    if (this->subscription_iterator_ != this->subscriptions_.end())
    {
        did_spin = (*this->subscription_iterator_)->spin_once();
        auto current_it = this->subscription_iterator_;
        // Increament for the next loop to start on someone else
        ++this->subscription_iterator_;
        // If it did spin, return true
        if (did_spin)
        {
            return true;
        }
        // If it did not spin
        // Loop from the start to the end looking for one to spin
        for (auto it = current_it; it != this->subscriptions_.end(); ++it)
        {
            did_spin = (*it)->spin_once();
            if (did_spin)
            {
                return true;
            }
        }
        // If it still did not spin, loop from beginning to start
        for (auto it = this->subscriptions_.begin(); it != current_it; ++it)
        {
            did_spin = (*it)->spin_once();
            if (did_spin)
            {
                return true;
            }
        }
    }
    return false;
}

void Node::static_signal_handler(int signo)
{
    std::cout << "Catching SIGINT (ctrl-c), shutting down nodes..." << std::endl;
    for (auto it = Node::nodes_.begin(); it != Node::nodes_.end(); ++it)
    {
        (*it)->shutdown("Caught SIGINT (ctrl-c)");
    }
}

void Node::shutdown(const std::string &reason)
{
    this->running_ = false;
    this->shutdown_reason_ = reason;
}

void Node::destroy_publisher(const rclcpp::publisher::PublisherInterface::Ptr &publisher_interface)
{
    this->destroy_publisher(publisher_interface->get_topic_name());
}

void Node::destroy_publisher(std::string topic_name)
{
    if (this->publishers_.find(topic_name) == this->publishers_.end())
    {
        // TODO Raise, topic not in list of publishers
    }
    this->publishers_.erase(topic_name);
}

rclcpp::parameter::ParameterClient::Ptr Node::create_parameter_client(const std::string &prefix)
{
    auto parameter_client_get_string = this->create_client<ParameterServerGetString>(
        prefix + "parameter_server_get_string");

    auto parameter_client_set_string = this->create_client<ParameterServerSetString>(
        prefix + "parameter_server_set_string");

    auto parameter_client_get_int64 = this->create_client<ParameterServerGetInt64>(
        prefix + "parameter_server_get_int64");

    auto parameter_client_set_int64 = this->create_client<ParameterServerSetInt64>(
        prefix + "parameter_server_set_int64");

    auto parameter_client_get_bool = this->create_client<ParameterServerGetBool>(
        prefix + "parameter_server_get_bool");

    auto parameter_client_set_bool = this->create_client<ParameterServerSetBool>(
        prefix + "parameter_server_set_bool");

    auto parameter_client_has = this->create_client<ParameterServerHas>(
        prefix + "parameter_server_has");

    auto parameter_client_delete = this->create_client<ParameterServerDelete>(
        prefix + "parameter_server_delete");

    auto parameter_client = rclcpp::parameter::ParameterClient::Ptr(
        new rclcpp::parameter::ParameterClient(
            parameter_client_get_string, parameter_client_set_string,
            parameter_client_get_int64, parameter_client_set_int64,
            parameter_client_get_bool, parameter_client_set_bool,
            parameter_client_has, parameter_client_delete
        )
    );
    return parameter_client;
}

rclcpp::parameter::ParameterServer::Ptr Node::create_parameter_server(const std::string &prefix)
{
    rclcpp::parameter::ParameterServerHandler::Ptr parameter_server_handler(
        new rclcpp::parameter::ParameterServerHandler()
    );

    auto parameter_server_get_string = this->create_service<rclcpp::ParameterServerGetString>(
        prefix + "parameter_server_get_string",
         std::bind(
            &rclcpp::parameter::ParameterServerHandler::get_param_string,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    auto parameter_server_set_string = this->create_service<rclcpp::ParameterServerSetString>(
        prefix + "parameter_server_set_string",
        std::bind(
            &rclcpp::parameter::ParameterServerHandler::set_param_string,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    auto parameter_server_get_int64 = this->create_service<rclcpp::ParameterServerGetInt64>(
        prefix + "parameter_server_get_int64",
        std::bind(
            &rclcpp::parameter::ParameterServerHandler::get_param_int64,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    auto parameter_server_set_int64 = this->create_service<rclcpp::ParameterServerSetInt64>(
        prefix + "parameter_server_set_int64",
        std::bind(
            &rclcpp::parameter::ParameterServerHandler::set_param_int64,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    auto parameter_server_get_bool = this->create_service<rclcpp::ParameterServerGetBool>(
        prefix + "parameter_server_get_bool",
        std::bind(
            &rclcpp::parameter::ParameterServerHandler::get_param_bool,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    auto parameter_server_set_bool = this->create_service<rclcpp::ParameterServerSetBool>(
        prefix + "parameter_server_set_bool",
        std::bind(
            &rclcpp::parameter::ParameterServerHandler::set_param_bool,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    auto parameter_server_has = this->create_service<rclcpp::ParameterServerHas>(
        prefix + "parameter_server_has",
        std::bind(
            &rclcpp::parameter::ParameterServerHandler::has_param,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    auto parameter_server_delete = this->create_service<rclcpp::ParameterServerDelete>(
        prefix + "parameter_server_delete",
        std::bind(
            &rclcpp::parameter::ParameterServerHandler::delete_param,
            parameter_server_handler, std::placeholders::_1, std::placeholders::_2
        )
    );

    rclcpp::parameter::ParameterServer::Ptr parameter_server(
        new rclcpp::parameter::ParameterServer(
            parameter_server_handler,
            parameter_server_get_string,
            parameter_server_set_string,
            parameter_server_get_int64,
            parameter_server_set_int64,
            parameter_server_get_bool,
            parameter_server_set_bool,
            parameter_server_has,
            parameter_server_delete
        )
    );

    return parameter_server;
}
