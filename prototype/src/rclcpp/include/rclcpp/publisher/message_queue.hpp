#include <queue>

#include <boost/shared_ptr.hpp>

namespace rclcpp
{
namespace publisher
{

template <typename T>
class MessageQueue
{
public:
    typedef boost::shared_ptr<T> MessagePtr;
    MessageQueue(size_t queue_size) : queue_size_(queue_size) {}
    ~MessageQueue() {}

    bool enqueue_message(const T &message)
    {
        MessagePtr message_ptr(message);
        return this->enqueue_message(message_ptr);
    }

    bool enqueue_message(const MessagePtr message_ptr)
    {
        bool no_messages_dropped = true;
        if (this->queue_.size() == this->queue_size_)
        {
            this->queue_.pop();
            no_messages_dropped = false;
        }
        this->queue_.push(message_ptr);
        return no_messages_dropped;
    }

    MessagePtr dequeue_message()
    {
        return this->queue_.pop();
    }

private:
    std::queue<MessagePtr> queue_;
    size_t queue_size_;
};

}
}
