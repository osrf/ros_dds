#ifndef RCLCPP_RCLCPP_CLIENT_CLIENT_HPP_
#define RCLCPP_RCLCPP_CLIENT_CLIENT_HPP_

#include <chrono>
#include <future>
#include <map>
#include <memory>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <rclcpp/publisher/publisher.hpp>

namespace rclcpp
{
namespace client
{

template <typename ROSService>
class Client
{
public:
    typedef std::shared_ptr<std::promise<typename ROSService::Response::ConstPtr> > shared_promise;
    typedef std::shared_ptr<Client<ROSService> > Ptr;
    typedef std::shared_future<typename ROSService::Response::ConstPtr> shared_future;


    Client(
        const std::string& client_id,
        rclcpp::publisher::Publisher::Ptr publisher,
        rclcpp::node::Node * node
    )
    : client_id_(client_id), publisher_(publisher), node_(node), req_id_(0)
    {}
    ~Client() {}

    void handle_response(typename ROSService::Response::ConstPtr res)
    {
        if (res->client_id == client_id_)
        {
            std::cout << "Got response" << std::endl;
            shared_promise call_promise = this->pending_calls_[res->req_id];
            this->pending_calls_.erase(res->req_id);
            call_promise->set_value(res);
        }
    }

    typename ROSService::Response::ConstPtr call(typename ROSService::Request &req)
    {
        shared_future f = this->async_call(req);
        // NOTE The version (4.6) of GCC that ships with Ubuntu 12.04
        // is broken, wait_for should return a std::future_status,
        // according to the C++11 spec, not a bool as is GCC's case
        std::future_status status;
        do {
            this->node_->spin_once();
            status = f.wait_for(std::chrono::milliseconds(100));
            if (!this->node_->is_running())
            {
               throw std::exception();
            }
        } while (status != std::future_status::ready);
        return f.get();
    }

    shared_future async_call(typename ROSService::Request &req) {
        req.req_id = ++(this->req_id_);
        req.client_id = client_id_;

        shared_promise call_promise(new std::promise<typename ROSService::Response::ConstPtr>);
        pending_calls_[req.req_id] = call_promise;

        this->publisher_->publish(req);

        return shared_future(call_promise->get_future());
    }

private:
    rclcpp::publisher::Publisher::Ptr publisher_;
    std::map<int, shared_promise> pending_calls_;
    std::string client_id_;
    int req_id_;
    rclcpp::node::Node *node_;
};

} // namespace client
} // namespace rclcpp

#endif /* RCLCPP_RCLCPP_CLIENT_CLIENT_HPP_ */
