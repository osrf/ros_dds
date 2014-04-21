#include <algorithm>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node/node.hpp>

#include <rclcpp/node/impl/node_impl.hpp>

using namespace rclcpp::node;
using namespace rclcpp::publisher;
using namespace rclcpp::subscription;

Node::Node(const std::string &name)
: name_(name), running_(true), shutdown_reason_(""), subscription_iterator_(this->subscriptions_.end())
{
    this->impl_ = new impl::NodeImpl(name);
    this->nodes_.push_back(this);
}

Node::~Node()
{
    this->running_ = false;
    this->subscriptions_.clear();
    this->nodes_.remove(this);
    delete this->impl_;
}

void Node::spin()
{
    this->impl_->spin(this);
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

bool Node::is_running()
{
    return this->running_;
}

void Node::shutdown(const std::string &reason)
{
    this->running_ = false;
    this->shutdown_reason_ = reason;
}

std::string Node::get_shutdown_reason()
{
    if (this->running_)
    {
        return "";
    }
    else
    {
        return this->shutdown_reason_;
    }
}

void Node::destroy_publisher(const rclcpp::publisher::Publisher::Ptr publisher)
{
    this->destroy_publisher(publisher->get_topic_name());
}

void Node::destroy_publisher(const std::string &topic_name)
{
    if (this->publishers_.find(topic_name) == this->publishers_.end())
    {
        // TODO Raise, topic not in list of publishers
    }
    this->publishers_.erase(topic_name);
}

void Node::destroy_subscription(const rclcpp::subscription::Subscription::Ptr subscription)
{
    this->impl_->status_conditions_.erase(subscription);
    this->subscriptions_.remove(subscription);
}

void Node::static_signal_handler(int signo)
{
    std::cout << "Catching SIGINT (ctrl-c), shutting down nodes..." << std::endl;
    for (auto it = Node::nodes_.begin(); it != Node::nodes_.end(); ++it)
    {
        (*it)->shutdown("Caught SIGINT (ctrl-c)");
    }
}
