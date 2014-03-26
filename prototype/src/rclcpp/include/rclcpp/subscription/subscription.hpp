#ifndef RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_
#define RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_
#include <functional>
#include <iostream>
#include <memory>

namespace rclcpp
{

/* Forward declare Node class for friendship with Publisher Interface */
namespace node {

class Node;

namespace impl
{

class NodeImpl;

} // namespace impl

} // namespace node

namespace subscription
{

namespace impl
{

class GenericSubscription
{
    friend class rclcpp::node::impl::NodeImpl;
public:
    virtual ~GenericSubscription() {};
    virtual bool spin_some(size_t spin_limit) = 0;
};

template <typename ROSMsgType>
class SpecificSubscriptionImpl;

template <typename ROSMsgType>
class SpecificSubscription : public GenericSubscription
{
    friend class rclcpp::node::impl::NodeImpl;
    SpecificSubscriptionImpl<ROSMsgType> * impl_;
public:
    SpecificSubscription(SpecificSubscriptionImpl<ROSMsgType> * impl)
    : impl_(impl)
    {}
    ~SpecificSubscription()
    {
        delete this->impl_;
    }

    bool spin_some(size_t spin_limit);

};

} // namespace impl

class Subscription
{
public:
    typedef std::shared_ptr<Subscription> Ptr;
private:
    friend class node::Node;
    friend class rclcpp::node::impl::NodeImpl;
    Subscription(const std::string &topic_name, impl::GenericSubscription * impl_ptr)
    : topic_name_(topic_name), impl_(impl_ptr)
    {}

public:
    ~Subscription()
    {
        delete this->impl_;
    }

    bool spin_once()
    {
        return this->impl_->spin_some(1);
    }

    bool spin_some()
    {
        return this->impl_->spin_some(0);
    }

private:
    std::string topic_name_;
    impl::GenericSubscription * impl_;
};

}

}

#endif /* RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_ */
