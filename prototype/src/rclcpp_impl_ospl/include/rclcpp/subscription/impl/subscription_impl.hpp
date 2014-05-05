#include <functional>

/* DDS Types */
#include <ccpp_dds_dcps.h>

namespace rclcpp
{
namespace subscription
{
namespace impl
{

template <typename ROSMsgType>
class SpecificSubscriptionImpl
{
public:
    typedef std::function<void(typename ROSMsgType::ConstPtr)> CallbackType;
    SpecificSubscriptionImpl(DDS::DataReader_var topic_reader, CallbackType callback, DDS::WaitSet * waitset);

private:
    void * data_reader_;
    CallbackType callback_;

};

}
}
}