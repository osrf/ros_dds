#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <boost/shared_ptr.hpp>

#include <genidlcpp/resolver.h>

#include <ccpp_dds_dcps.h>

namespace rclcpp
{
    namespace client
    {
        template <typename ROSRequestType, typename ROSResponseType>
        class Client
        {
        public:
            typedef boost::shared_ptr<const ROSRequestType> req_shared_ptr;
            typedef boost::shared_ptr<const ROSResponseType> res_shared_ptr;
            typedef void (*CallbackType)(const ROSRequestType &req, const ROSResponseType &res);
            typedef void (*SharedPtrCallbackType)(const ROSRequestType req, const ROSResponseType res);

            Client() {}
            ~Client() {}

        private:
            CallbackType cb_;
        };
    }
}
#endif
