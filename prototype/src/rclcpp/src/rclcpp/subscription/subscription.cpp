#include <rclcpp/subscription/subscription.hpp>

#include <rclcpp/subscription/impl/subscription_impl.hpp>

template <typename ROSMsgType>
bool rclcpp::subscription::impl::SpecificSubscription<ROSMsgType>::spin_some(size_t spin_limit)
{
    return this->impl_->spin_some(spin_limit);
}
