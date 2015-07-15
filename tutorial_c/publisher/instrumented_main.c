/* CreateTopics.c */
#include "dds_dcps.h"
#include "Chat.h"
#include "unistd.h"

#include <rttest/rttest.h>

#define MAX_MSG_LEN 16777216

int i = 0;
Chat_ChatMessageDataWriter talker;
Chat_ChatMessage *msg;
DDS_ReturnCode_t status;
DDS_InstanceHandle_t userHandle;
char *msg_content;

void* callback(void* unused)
{
  msg->index = i;
  snprintf(msg->content, MAX_MSG_LEN, msg_content, msg->index);
  status = Chat_ChatMessageDataWriter_write(talker, msg, userHandle);
  // checkStatus(status, "Chat_ChatMessageDataWriter_write");
  ++i;
}

int main (
  int argc,
  char *argv[])
{
  // begin init

  DDS_DomainParticipantFactory dpf;
  DDS_DomainParticipant dp;
  DDS_DomainId_t domain = DDS_DOMAIN_ID_DEFAULT;
  Chat_ChatMessageTypeSupport chatMessageTS;
  DDS_Topic chatMessageTopic;
  char *chatMessageTypeName;

  DDS_PublisherQos *pub_qos;
  DDS_DataWriterQos *dw_qos;
  DDS_Publisher chatPublisher;
  Chat_NameServiceDataWriter nameServer;
  char *partitionName = NULL;

  DDS_Topic nameServiceTopic;

  int ownID;

  Chat_NameService ns;


  /* Create a DomainParticipantFactory and a DomainParticipant */
  /* (using Default QoS settings). */
  dpf = DDS_DomainParticipantFactory_get_instance();
  if (!dpf) {
    printf("Creating ParticipantFactory failed!!\n");
    exit(-1);
  }
  printf("Created ParticipantFactory.\n");

  dp = DDS_DomainParticipantFactory_create_participant (
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


  /* Register the required data type for ChatMessage. */
  chatMessageTS = Chat_ChatMessageTypeSupport__alloc();
  if (!chatMessageTS) {
    printf ("Allocating TypeSupport failed!!\n");
    exit(-1);
  };
  chatMessageTypeName = Chat_ChatMessageTypeSupport_get_type_name(chatMessageTS);
  status = Chat_ChatMessageTypeSupport_register_type(
    chatMessageTS, dp, chatMessageTypeName);
  if (status != DDS_RETCODE_OK) {
    printf("Registering data type failed. Status = %d\n", status);
    exit(-1);
  };
  printf("Registered data type.\n");

  /*Create the ChatMessage topic */
  chatMessageTopic = DDS_DomainParticipant_create_topic(
    dp,
    "Chat_ChatMessage",
    chatMessageTypeName,
    DDS_TOPIC_QOS_DEFAULT,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!chatMessageTopic) {
    printf("Creating ChatMessage topic failed!!\n");
    exit(-1);
  };
  printf("Created ChatMessage topic.\n");

  /* Adapt the default PublisherQos to write into the
     "ChatRoom" Partition. */
  partitionName = "ChatRoom";
  pub_qos = DDS_PublisherQos__alloc();
  if (!pub_qos) {
    printf("Allocating PublisherQos failed!!\n");
    exit(-1);
  }
  status = DDS_DomainParticipant_get_default_publisher_qos (
    dp, pub_qos);
  if (status != DDS_RETCODE_OK) {
    printf("Getting default publisher qos failed!!\n");
    exit(-1);
  }
  pub_qos->partition.name._length = 1;
  pub_qos->partition.name._maximum = 1;
  pub_qos->partition.name._buffer = DDS_StringSeq_allocbuf (1);
  if (!pub_qos->partition.name._buffer) {
    printf("Allocating partition name failed!!\n");
    exit(-1);
  }
  pub_qos->partition.name._buffer[0] = DDS_string_alloc (
    strlen(partitionName));
  if (!pub_qos->partition.name._buffer[0]) {
    printf("Allocating partition name failed!!\n");
    exit(-1);
  }
  strcpy (pub_qos->partition.name._buffer[0], partitionName);

  /* Create a Publisher for the chatter application. */
  chatPublisher = DDS_DomainParticipant_create_publisher(
    dp, pub_qos, NULL, DDS_STATUS_MASK_NONE);
  if (!chatPublisher) {
    printf("Creating publisher failed!!\n");
    exit(-1);
  }
  printf("Created publisher.\n");

  /* Create a DataWriter for the ChatMessage Topic
     (using the appropriate QoS). */
  talker = DDS_Publisher_create_datawriter(
    chatPublisher,
    chatMessageTopic,
    DDS_DATAWRITER_QOS_USE_TOPIC_QOS,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!talker) {
    printf("Creating datawriter failed!!\n");
    exit(-1);
  }
  printf("Created datawriter.\n");

  ownID = 0;

  msg = Chat_ChatMessage__alloc();
  //checkHandle(msg, "Chat_ChatMessage__alolc");
  msg->userID = ownID;
  msg->index = 0;
  msg->content = DDS_string_alloc(MAX_MSG_LEN);
  //checkHandle(msg->content, "DDS_string_alloc");
  snprintf(msg->content, MAX_MSG_LEN, "hello world");

  // register a chat message
  userHandle = Chat_ChatMessageDataWriter_register_instance(talker, msg);

  ns.userID = ownID;
  ns.name = DDS_string_alloc(Chat_MAX_NAME+1);
  //checkHandle(ns.name, "DDS_string_alloc");
  char *chatterName;
  snprintf(ns.name, Chat_MAX_NAME+1, "Chatter %d", ownID);

  int j;
  msg_content = (char*) malloc(MAX_MSG_LEN);
  for (j < 0; j < MAX_MSG_LEN; ++j)
  {
    msg_content[j] = '.';
  }

  // Write user information
  status = Chat_NameServiceDataWriter_write(nameServer, &ns, DDS_HANDLE_NIL);
  //checkStatus(status, "Chat_ChatMessageDataWriter_write");

  printf("Created user handle and preallocated message.\n");

  // end init

  rttest_read_args(argc, argv);
  rttest_set_sched_priority(90, SCHED_RR);

  rttest_spin(callback, NULL);
  rttest_write_results();
  rttest_finish();

  // begin teardown

  /* Remove the DataWriters */
  status = DDS_Publisher_delete_datawriter(chatPublisher,
    talker);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting datawriter failed!!\n");
    exit(-1);
  }
  printf("Deleted datawriter.\n");

  /* Remove the Publisher. */
  status = DDS_DomainParticipant_delete_publisher(
    dp, chatPublisher);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting publisher failed!!\n");
    exit(-1);
  }
  /* De-allocate the PublisherQoS holder. */
  DDS_free(pub_qos); // Note that DDS_free recursively
                     // de-allocates all indirections!!
  printf("Deleted publisher.\n");

  /* Deleting the Topic. */
  status = DDS_DomainParticipant_delete_topic(
    dp, chatMessageTopic);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting topic failed. Status = %d\n", status);
    exit(-1);
  };
  printf("Deleted ChatMessage topic.\n");

  /* Deleting the DomainParticipant */
  status = DDS_DomainParticipantFactory_delete_participant(
  dpf, dp);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting participant failed. Status = %d\n", status);
    exit(-1);
  };
  printf("Deleted Participant.\n");

  /* Everything is fine, return normally. */
  return 0;
};
