#ifndef SUBSCRIPTION_HPP
#define SUBSCRIPTION_HPP

#include <boost/shared_ptr.hpp>

// TODO: use something less generic
#include "ccpp_ROSMsg.h"

namespace rclcpp
{
    namespace subscription
    {
        class SubscriptionInterface
        {
            public:
                virtual void spin() = 0;
        };

        template <typename T>
        class Subscription : public SubscriptionInterface
        {
        public:
            typedef boost::shared_ptr<const T> msg_shared_ptr;
            typedef void (*CallbackType)(const T &msg);
            typedef void (*SharedPtrCallbackType)(const msg_shared_ptr msg);

            Subscription(ROSMessageDataReader_var data_reader, CallbackType cb) : data_reader_(data_reader), cb_(cb) {}
            ~Subscription() {}


            void spin()
            {
                ROSMessageSeq_var ros_message_seq = new ROSMessageSeq();
                DDS::SampleInfoSeq_var sample_info_seq = new DDS::SampleInfoSeq();
                this->data_reader_->take(
                    ros_message_seq,
                    sample_info_seq,
                    DDS::LENGTH_UNLIMITED,
                    DDS::ANY_SAMPLE_STATE,
                    DDS::ANY_VIEW_STATE,
                    DDS::ALIVE_INSTANCE_STATE
                );

                for (DDS::ULong i = 0; i < ros_message_seq->length(); i++)
                {
                    ROSMessage *msg = &(ros_message_seq[i]);
                    this->cb_(T(msg->data));
                }

                this->data_reader_->return_loan(ros_message_seq, sample_info_seq);
           }

           
        private:
            ROSMessageDataReader_var data_reader_; 
            CallbackType cb_;
        };
    }
}
#endif
