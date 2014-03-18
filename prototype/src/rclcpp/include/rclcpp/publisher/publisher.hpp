#include <string>

namespace rclcpp
{
    namespace publisher
    {
        class Publisher
        {
        public:
            Publisher(std::string topic_name, size_t queue_size) : topic_name_(topic_name), queue_size_(queue_size) {};
            ~Publisher() {};

            template <typename T>
            void publish(T msg);

            std::string get_topic_name()
            {
                return this->topic_name_;
            }
        private:
            std::string topic_name_;
            size_t queue_size_;
        };
    }
}
