#include <rclcpp/publisher/publisher.hpp>
#include <rclcpp/subscription/subscription.hpp>

namespace rclcpp
{
    using publisher::Publisher;
    using subscription::Subscription;

    namespace node
    {
        class Node
        {
        public:
            Node();
            ~Node();

            template <typename T>
            Publisher<T> create_publisher(std::string topic_name, size_t queue_size);

            template <typename T>
            void destroy_publisher(Publisher<T> publisher);

            template <typename T>
            Subscription<T> create_subscription(std::string topic_name,
                                                size_t queue_size,
                                                typename Subscription<T>::CallbackType cb);

            template <typename T>
            void destroy_subscription(Subscription<T> subscription);
        };
    }
}