
#ifndef __genidlcpp__Resolver__h__
#define __genidlcpp__Resolver__h__

namespace  dds_impl
{
template<typename T>
struct DDSTypeResolver
{
  typedef void* DDSMsgType;
  typedef void* DDSMsgTypeSupportType;
  typedef void* DDSMsgTypeSupportType_var;
  typedef void* DDSMsgDataWriter;
  typedef void* DDSMsgDataWriter_var;
};

template <typename T, typename U>
void convert_ros_message_to_dds(const T&, U&);

template <typename T, typename U>
void convert_dds_message_to_ros(const T&, U&);

} // namespace dds_impl

#endif
