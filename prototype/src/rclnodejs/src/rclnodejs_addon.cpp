#define BUILDING_NODE_EXTENSION
#include <node/node.h>
#include "rclnodejs/rclnodejs.h"

using namespace v8;

Handle<Value> CreateObject(const Arguments& args) {
  HandleScope scope;
  return scope.Close(MyObject::NewInstance(args));
}

void InitAll(Handle<Object> exports, Handle<Object> module) {
  MyObject::Init();

  module->Set(String::NewSymbol("exports"),
      FunctionTemplate::New(CreateObject)->GetFunction());
}

NODE_MODULE(rclnodejs, InitAll)
