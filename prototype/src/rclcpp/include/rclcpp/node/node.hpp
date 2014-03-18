#include <map>

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
            Node(std::string name);
            ~Node();

            Publisher create_publisher(std::string topic_name, size_t queue_size);

            void destroy_publisher(Publisher publisher);
            void destroy_publisher(std::string topic_name);

            template <typename T>
            Subscription<T> create_subscription(std::string topic_name,
                                                size_t queue_size,
                                                typename Subscription<T>::CallbackType cb);

            template <typename T>
            void destroy_subscription(Subscription<T> subscription);
        private:
            std::string name_;

            std::map<std::string, Publisher> publishers_;
        };
    }
}