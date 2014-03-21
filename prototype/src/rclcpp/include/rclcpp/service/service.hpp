#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <boost/shared_ptr.hpp>

#include <genidlcpp/resolver.h>

#include <ccpp_dds_dcps.h>

namespace rclcpp
{
    namespace service
    {
        template <typename ROSRequestType, typename ROSResponseType>
        class Service
        {
        public:
            typedef boost::shared_ptr<const ROSRequestType> req_shared_ptr;
            typedef boost::shared_ptr<const ROSResponseType> res_shared_ptr;
            typedef bool (*CallbackType)(ROSRequestType &req, ROSResponseType &res);
            typedef bool (*SharedPtrCallbackType)(ROSRequestType req, ROSResponseType res);

            Service()
            {
                
            }

            ~Service() {}

           void handle_request(const ROSResponseType& request) {
           }

        private:
            CallbackType cb_;
        };
    }
}
#endif
