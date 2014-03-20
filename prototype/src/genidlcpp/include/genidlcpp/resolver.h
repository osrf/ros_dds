namespace  dds_impl
{
template<typename T>
struct DDSTypeResolver
{
  typedef void* DDSMsgType;
  typedef void* DDSMsgTypeSupportType;
  typedef void* DDSMsgDataWriter;
};
} // namespace dds_impl
