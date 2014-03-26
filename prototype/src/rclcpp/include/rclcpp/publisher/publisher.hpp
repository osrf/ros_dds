#ifndef RCLCPP_RCLCPP_PUBLISHER_PUBLISHER_HPP_
#define RCLCPP_RCLCPP_PUBLISHER_PUBLISHER_HPP_
#include <exception>
#include <string>

namespace rclcpp
{

// Forward declare Node class for friendship with Publisher Interface
namespace node {class Node;}

namespace publisher
{

class DuplicatePublisherException : public std::exception
{
    virtual const char* what() const throw()
    {
        return "Publisher for topic already exists";
    }
};

class Publisher;

namespace impl
{

class GenericPublisher
{
    friend class rclcpp::publisher::Publisher;
    std::string topic_name_;
public:
    GenericPublisher(const std::string &topic_name)
    : topic_name_(topic_name)
    {}
};

template <typename ROSMsgType>
class SpecificPublisherImpl;

template <typename ROSMsgType>
class SpecificPublisher : public GenericPublisher
{
    SpecificPublisherImpl<ROSMsgType> * impl_;
public:
    SpecificPublisher(const std::string &topic_name, SpecificPublisherImpl<ROSMsgType> * impl)
    : GenericPublisher(topic_name), impl_(impl)
    {}
    void publish(const ROSMsgType &msg);
};

} // namespace impl

class Publisher
{
public:
    typedef std::shared_ptr<Publisher> Ptr;
    Publisher(impl::GenericPublisher * impl);
    ~Publisher();

    template <typename ROSMsgType>
    void publish(const ROSMsgType &msg)
    {
        return static_cast<impl::SpecificPublisher<ROSMsgType> * >(this->impl_)->publish(msg);
    }

    std::string get_topic_name();

private:
    impl::GenericPublisher * impl_;

};

}

}

#endif /* RCLCPP_RCLCPP_PUBLISHER_PUBLISHER_HPP_ */
