#ifndef PARAMETER_CLIENT_HPP
#define PARAMETER_CLIENT_HPP

#include <rclcpp/client/client.hpp>

#include <rclcpp/ParameterServerGetString.h>
#include <rclcpp/ParameterServerSetString.h>

#include "rclcpp/dds_impl/ParameterServerGetStringRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerGetStringResponse_convert.h"

#include "rclcpp/dds_impl/ParameterServerSetStringRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerSetStringResponse_convert.h"

#include <rclcpp/ParameterServerGetInt64.h>
#include <rclcpp/ParameterServerSetInt64.h>

#include "rclcpp/dds_impl/ParameterServerGetInt64Request_convert.h"
#include "rclcpp/dds_impl/ParameterServerGetInt64Response_convert.h"

#include "rclcpp/dds_impl/ParameterServerSetInt64Request_convert.h"
#include "rclcpp/dds_impl/ParameterServerSetInt64Response_convert.h"

#include <rclcpp/ParameterServerGetBool.h>
#include <rclcpp/ParameterServerSetBool.h>

#include "rclcpp/dds_impl/ParameterServerGetBoolRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerGetBoolResponse_convert.h"

#include "rclcpp/dds_impl/ParameterServerSetBoolRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerSetBoolResponse_convert.h"

#include <rclcpp/ParameterServerHas.h>

#include "rclcpp/dds_impl/ParameterServerHasRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerHasResponse_convert.h"

#include <rclcpp/ParameterServerDelete.h>

#include "rclcpp/dds_impl/ParameterServerDeleteRequest_convert.h"
#include "rclcpp/dds_impl/ParameterServerDeleteResponse_convert.h"

namespace rclcpp
{
    namespace parameter
    {
        class ParameterClient
        {
        public:
            typedef std::shared_ptr< ParameterClient > Ptr;

            ParameterClient(
                typename rclcpp::client::Client<ParameterServerGetString>::Ptr parameter_client_get_string,
                typename rclcpp::client::Client<ParameterServerSetString>::Ptr parameter_client_set_string,
                typename rclcpp::client::Client<ParameterServerGetInt64>::Ptr parameter_client_get_int64,
                typename rclcpp::client::Client<ParameterServerSetInt64>::Ptr parameter_client_set_int64,
                typename rclcpp::client::Client<ParameterServerGetBool>::Ptr parameter_client_get_bool,
                typename rclcpp::client::Client<ParameterServerSetBool>::Ptr parameter_client_set_bool,
                typename rclcpp::client::Client<ParameterServerHas>::Ptr parameter_client_has,
                typename rclcpp::client::Client<ParameterServerDelete>::Ptr parameter_client_delete
            ) : parameter_client_get_string_(parameter_client_get_string),
                parameter_client_set_string_(parameter_client_set_string),
                parameter_client_get_int64_(parameter_client_get_int64),
                parameter_client_set_int64_(parameter_client_set_int64),
                parameter_client_get_bool_(parameter_client_get_bool),
                parameter_client_set_bool_(parameter_client_set_bool),
                parameter_client_has_(parameter_client_has),
                parameter_client_delete_(parameter_client_delete)
            {
            }
            ~ParameterClient() {}

            void set_param(const std::string &key, const std::string &s)
            {
                ParameterServerSetString::Request req;
                req.param = key;
                req.value = s;
                this->parameter_client_set_string_->call(req);
            }

            void get_param(const std::string &key, std::string &s)
            {
                ParameterServerGetString::Request req;
                req.param = key;
                auto res = this->parameter_client_get_string_->call(req);
                s = res->value;
            }

            void set_param(const std::string &key, const int64_t i)
            {
                ParameterServerSetInt64::Request req;
                req.param = key;
                req.value = i;
                this->parameter_client_set_int64_->call(req);
            }

            void get_param(const std::string &key, int64_t &i)
            {
                ParameterServerGetInt64::Request req;
                req.param = key;
                auto res = this->parameter_client_get_int64_->call(req);
                i = res->value;
            }

            void set_param(const std::string &key, const bool b)
            {
                ParameterServerSetBool::Request req;
                req.param = key;
                req.value = b;
                this->parameter_client_set_bool_->call(req);
            }

            void get_param(const std::string &key, bool &b)
            {
                ParameterServerGetBool::Request req;
                req.param = key;
                auto res = this->parameter_client_get_bool_->call(req);
                b = res->value;
            }

            void has_param(const std::string &key)
            {
                ParameterServerHas::Request req;
                req.param = key;
                this->parameter_client_has_->call(req);
            }

            void delete_param(const std::string &key)
            {
                ParameterServerDelete::Request req;
                req.param = key;
                this->parameter_client_delete_->call(req);
            }


        private:
            rclcpp::client::Client<ParameterServerGetString>::Ptr parameter_client_get_string_;
            rclcpp::client::Client<ParameterServerSetString>::Ptr parameter_client_set_string_;
  
            rclcpp::client::Client<ParameterServerGetInt64>::Ptr parameter_client_get_int64_;
            rclcpp::client::Client<ParameterServerSetInt64>::Ptr parameter_client_set_int64_;
  
            rclcpp::client::Client<ParameterServerGetBool>::Ptr parameter_client_get_bool_;
            rclcpp::client::Client<ParameterServerSetBool>::Ptr parameter_client_set_bool_;

            rclcpp::client::Client<ParameterServerHas>::Ptr parameter_client_has_;

            rclcpp::client::Client<ParameterServerDelete>::Ptr parameter_client_delete_;
        };
    }
}
#endif
