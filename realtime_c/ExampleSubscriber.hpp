#include "dds_dcps.h"
#include "LargeMsg.h"
#include "check_status.h"
#include "unistd.h"

class ExampleSubscriber
{

  private:

    DDS_DomainId_t domain = DDS_DOMAIN_ID_DEFAULT;
    DDS_DomainParticipantFactory dpf;
    DDS_DomainParticipant dp;
    DDS_SubscriberQos *sub_qos;
    DDS_Subscriber chatSubscriber;
    LargeMsg_LargeMessageDataReader mbReader;
    //char *partitionName;
    DDS_ReturnCode_t status;
    DDS_sequence_LargeMsg_LargeMessage *msgSeq;
    DDS_SampleInfoSeq *infoSeq;
    LargeMsg_LargeMessage *msg;
    unsigned long received_msgs_count;
    LargeMsg_LargeMessageTypeSupport chatMessageTS;
    char *chatMessageTypeName;
    DDS_Topic chatMessageTopic;

  public:
    void callback();
    bool init();
    bool teardown();
};

bool ExampleSubscriber::init()
{
  /* Create a DomainParticipantFactory and a DomainParticipant */
  /* (using Default QoS settings). */
  this->dpf = DDS_DomainParticipantFactory_get_instance();
  if (!dpf) {
    printf("Creating ParticipantFactory failed!!\n");
    exit(-1);
  }
  printf("Created ParticipantFactory.\n");

  this->dp = DDS_DomainParticipantFactory_create_participant (
    dpf,
    domain,
    DDS_PARTICIPANT_QOS_DEFAULT,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!dp) {
    printf("Creating Participant failed!!\n");
    exit(-1);
  }
  printf("Created Participant.\n");

  /* Register the required data type for LargeMessage. */
  this->chatMessageTS = LargeMsg_LargeMessageTypeSupport__alloc();
  if (!chatMessageTS) {
    printf ("Allocating TypeSupport failed!!\n");
    exit(-1);
  };
  this->chatMessageTypeName = LargeMsg_LargeMessageTypeSupport_get_type_name(chatMessageTS);
  status = LargeMsg_LargeMessageTypeSupport_register_type(
    chatMessageTS, dp, chatMessageTypeName);
  if (status != DDS_RETCODE_OK) {
    printf("Registering data type failed. Status = %d\n", status);
    exit(-1);
  };
  printf("Registered data type.\n");

  /* Set QoS policty to BEST_EFFORT */
  DDS_TopicQos *topic_qos = DDS_TopicQos__alloc();
  status = DDS_DomainParticipant_get_default_topic_qos(dp, topic_qos);
  checkStatus(status, "DDS_DomainParticipant_get_default_topic_qos");
  topic_qos->reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;

  /* Make the tailored QoS the new default. */
  status = DDS_DomainParticipant_set_default_topic_qos(participant,
      topic_qos);
  checkStatus(status, "DDS_DomainParticipant_set_default_topic_qos");

  /*Create the LargeMessage topic */
  this->chatMessageTopic = DDS_DomainParticipant_create_topic(
    dp,
    "LargeMsg_LargeMessage",
    chatMessageTypeName,
    DDS_TOPIC_QOS_DEFAULT,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!chatMessageTopic) {
    printf("Creating LargeMessage topic failed!!\n");
    exit(-1);
  };

  printf("Created LargeMessage topic.\n");


  /* Adapt the default SubscriberQos to read from the
  "ChatRoom" Partition. */
  //this->partitionName = "ChatRoom";
  this->sub_qos = DDS_SubscriberQos__alloc();
  // checkHandle(sub_qos, "DDS_SubscriberQos__alloc");
  status = DDS_DomainParticipant_get_default_subscriber_qos(this->dp, sub_qos);
  // checkStatus(status, "DDS_DomainParticipant_get_default_subscriber_qos");
  /*sub_qos->partition.name._length = 1;
  sub_qos->partition.name._maximum = 1;
  sub_qos->partition.name._buffer = DDS_StringSeq_allocbuf (1);
  checkHandle(sub_qos->partition.name._buffer, "DDS_StringSeq_allocbuf");
  sub_qos->partition.name._buffer[0] =
  DDS_string_alloc (strlen(partitionName));
  checkHandle(sub_qos->partition.name._buffer[0], "DDS_string_alloc");
  strcpy(sub_qos->partition.name._buffer[0], partitionName);*/
  /* Create a Subscriber for the MessageBoard application. */
  chatSubscriber = DDS_DomainParticipant_create_subscriber(
      this->dp, sub_qos, NULL, DDS_STATUS_MASK_NONE);
  // checkHandle(chatSubscriber, "DDS_DomainParticipant_create_subscriber");
  /* Create a DataReader for the ChatMessage Topic
  (using the appropriate QoS). */
  this->mbReader = DDS_Subscriber_create_datareader(
      chatSubscriber,
      chatMessageTopic,
      DDS_DATAREADER_QOS_USE_TOPIC_QOS,
      NULL,
      DDS_STATUS_MASK_NONE);
  // checkHandle(mbReader, "DDS_Subscriber_create_datareader");

  msgSeq = DDS_sequence_LargeMsg_LargeMessage__alloc();
  // checkHandle(msgSeq, "DDS_sequence_Chat_NamedMessage__alloc");
  infoSeq = DDS_SampleInfoSeq__alloc();
  // checkHandle(infoSeq, "DDS_SampleInfoSeq__alloc");
  received_msgs_count = 0;

  return true;
}

void ExampleSubscriber::callback()
{
  status = LargeMsg_LargeMessageDataReader_take(
      mbReader,
      msgSeq,
      infoSeq,
      DDS_LENGTH_UNLIMITED,
      DDS_ANY_SAMPLE_STATE,
      DDS_ANY_VIEW_STATE,
      DDS_ALIVE_INSTANCE_STATE);
  // checkStatus(status, "Chat_NamedMessageDataReader_take");

  DDS_unsigned_long i;
  for (i = 0; i < msgSeq->_length; i++) {
    msg = &(msgSeq->_buffer[i]);
    ++this->received_msgs_count;
  }

  status = LargeMsg_LargeMessageDataReader_return_loan(mbReader, msgSeq, infoSeq);
  // checkStatus(status, "LargeMsg_LargeMessageDataReader_return_loan");
}

bool ExampleSubscriber::teardown()
{
  /* Remove the DataReader */
  DDS_Subscriber_delete_datareader(this->chatSubscriber, this->mbReader);
  // checkStatus(status, "DDS_Subscriber_delete_datareader");
  /* Remove the Subscriber. */
  status = DDS_DomainParticipant_delete_subscriber(this->dp, chatSubscriber);
  // checkStatus(status, "DDS_DomainParticipant_delete_subscriber");
  /* De-allocate the SubscriberQoS holder. */
  DDS_free(this->sub_qos); // Note that DDS_free recursively
                           // de-allocates all indirections!!
  printf("Received %lu total messages.\n", this->received_msgs_count);
  return false;
}
