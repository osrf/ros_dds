#include <boost/shared_ptr.hpp>

namespace rclcpp
{
    namespace subscription
    {
        template <typename T>
        class Subscription
        {
            typedef boost::shared_ptr<const T> msg_shared_ptr;
            typedef void (*CallbackType)(const T &msg);
            typedef void (*SharedPtrCallbackType)(const msg_shared_ptr msg);

        public:
            Subscription();
            ~Subscription();
        };
    }
}