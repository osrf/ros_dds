#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
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

            Client(rclcpp::Node *node) : node_(node) {}
            ~Client() {}

            void handle_response(const ROSResponseType& res) {
            }

            ROSResponseType call(const ROSRequestType& req) {
                ROSResponseType res;
                return res;
            }

        private:
            rclcpp::Node *node_;
        };
    }
}
#endif
