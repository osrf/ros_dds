/* CreateTopics.c */
#include "dds_dcps.h"
#include "Chat.h"
#include "unistd.h"

int main (
  int argc,
  char *argv[])
{
  DDS_DomainParticipantFactory dpf;
  DDS_DomainParticipant dp;
  DDS_DomainId_t domain = DDS_DOMAIN_ID_DEFAULT;
  DDS_ReturnCode_t status;
  Chat_ChatMessageTypeSupport chatMessageTS;
  DDS_Topic chatMessageTopic;
  char *chatMessageTypeName;

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

  sleep(10);

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
