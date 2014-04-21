#include <rclcpp/publisher/publisher.hpp>

#include <rclcpp/publisher/impl/publisher_impl.hpp>

using namespace rclcpp::publisher;

Publisher::Publisher(impl::GenericPublisher * impl)
: impl_(impl)
{

}

Publisher::~Publisher()
{

}

std::string Publisher::get_topic_name()
{
    return this->impl_->topic_name_;
}

template <typename ROSMsgType>
void rclcpp::publisher::impl::SpecificPublisher<ROSMsgType>::publish(const ROSMsgType &msg)
{
    return this->impl_->publish(msg);
}
