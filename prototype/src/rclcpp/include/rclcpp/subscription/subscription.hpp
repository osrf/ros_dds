#ifndef RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_
#define RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_
#include <functional>
#include <memory>

#include <ccpp_dds_dcps.h>

#include <genidlcpp/resolver.h>

namespace rclcpp
{

namespace subscription
{

class SubscriptionInterface
{
public:
    virtual void spin_once() = 0;
};

template <typename ROSMsgType>
class Subscription : public SubscriptionInterface
{
public:
    typedef dds_impl::DDSTypeResolver<ROSMsgType> r;
    typedef std::function<void(typename ROSMsgType::ConstPtr)> CallbackType;

    Subscription(typename r::DDSMsgDataReaderType_var data_reader, CallbackType cb)
    : data_reader_(data_reader), cb_(cb)
    {}
    ~Subscription() {}


    void spin_once()
    {
        typename r::DDSMsgSeqType_var dds_msg_seq = new typename r::DDSMsgSeqType();
        DDS::SampleInfoSeq_var sample_info_seq = new DDS::SampleInfoSeq();
        this->data_reader_->take(
            dds_msg_seq,
            sample_info_seq,
            DDS::LENGTH_UNLIMITED,
            DDS::ANY_SAMPLE_STATE,
            DDS::ANY_VIEW_STATE,
            DDS::ALIVE_INSTANCE_STATE
        );

        for (DDS::ULong i = 0; i < dds_msg_seq->length(); i++)
        {
            typename ROSMsgType::Ptr ros_msg;
            dds_impl::DDSTypeResolver<ROSMsgType>::convert_dds_message_to_ros(dds_msg_seq[i], (*ros_msg.get()));
            this->cb_(ros_msg);
        }

        this->data_reader_->return_loan(dds_msg_seq, sample_info_seq);
   }

private:
    typename r::DDSMsgDataReaderType_var data_reader_;
    CallbackType cb_;
};

}

}

#endif /* RCLCPP_RCLCPP_SUBSCRIPTION_SUBSCRIPTION_HPP_ */
