#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node/node.hpp>
#include <rclcpp/publisher/publisher.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <genidlcpp/resolver.h>

#include <ccpp_dds_dcps.h>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>

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
            typedef boost::shared_ptr< boost::promise<const ROSResponseType&> > shared_promise;

            Client(boost::uuids::uuid client_id, typename Publisher<ROSRequestType>::shared_publisher publisher) : client_id_(client_id), publisher_(publisher), seq_id_(0) {}
            ~Client() {}

            void handle_response(const ROSResponseType& res) {
                shared_promise call_promise = this->pending_calls_[res.req_id];
                this->pending_calls_.erase(res.req_id);
                call_promise->set_value(res);
            }

            ROSResponseType call(ROSRequestType &req) {
                boost::uuids::uuid req_id = boost::uuids::random_generator()();
                const std::string str_req_id = boost::lexical_cast<std::string>(req_id);
                this->seq_id_++;
                req.req_id = this->seq_id_;

                shared_promise call_promise(new boost::promise<const ROSResponseType&>);
                pending_calls_[req.req_id] = call_promise;

                return call_promise->get_future().get();
            }

        private:
            typename Publisher<ROSRequestType>::shared_publisher publisher_;
            std::map<int, shared_promise> pending_calls_;
            boost::uuids::uuid client_id_;
            int seq_id_;
        };
    }
}
#endif
