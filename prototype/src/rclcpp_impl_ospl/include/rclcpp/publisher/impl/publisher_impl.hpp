#include <string>

/* DDS Types */
#include <ccpp.h>

namespace rclcpp
{
namespace publisher
{

class Publisher;

namespace impl
{

template <typename ROSMsgType>
class SpecificPublisherImpl
{
    friend class rclcpp::publisher::Publisher;
public:
    SpecificPublisherImpl(DDS::Publisher_var dds_publisher,
                          DDS::Topic_var dds_topic,
                          DDS::DataWriter_var dds_topic_datawriter);
    ~SpecificPublisherImpl();

    void publish(const ROSMsgType &msg);

private:
    DDS::Publisher_var dds_publisher_;
    DDS::Topic_var dds_topic_;
    DDS::DataWriter_var dds_topic_datawriter_;

    void * data_writer_;

};

}
}
}
