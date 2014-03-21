#ifndef RCLNODEJS_RCLNODEJS__H
#define RCLNODEJS_RCLNODEJS__H

#define BUILDING_NODE_EXTENSION
#include <node/node.h>

#include <rclcpp/rclcpp.hpp>

class RCLInterface : public node::ObjectWrap {
 public:
  static void Init();
  static v8::Handle<v8::Value> NewInstance(const v8::Arguments& args);

 private:
  explicit RCLInterface();
  ~RCLInterface();

  static v8::Handle<v8::Value> New(const v8::Arguments& args);
  static v8::Handle<v8::Value> PlusOne(const v8::Arguments& args);
  static v8::Persistent<v8::Function> constructor;
  double value_;
};

#endif /* RCLNODEJS_RCLNODEJS__H */
