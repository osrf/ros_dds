#include "dds_dcps.h"
#include "LargeMsg.h"
#include "check_status.h"
#include "unistd.h"

#define LargeMsg_MAX_NAME 128
#define MAX_MSG_LEN 16777216

class ExamplePublisher
{
	private:
    DDS_DomainParticipantFactory dpf;
    DDS_DomainParticipant dp;
    DDS_PublisherQos pub_qos;
    DDS_TopicQos topic_qos;
    DDS_Publisher chatPublisher;
    DDS_Topic chatMessageTopic;
    LargeMsg_LargeMessageDataWriter talker;
    LargeMsg_LargeMessage msg;
    DDS_ReturnCode_t status;
    DDS_InstanceHandle_t userHandle;

		int i;

	public:
    unsigned int message_size;
		void callback();
		bool init();
		bool teardown();
};

bool ExamplePublisher::init()
{
  if (message_size > MAX_MSG_LEN)
  {
    std::cout << "Clamping message size to maximum" << std::endl;
    message_size = MAX_MSG_LEN;
  }
  if (message_size == 0)
  {
    std::cout << "Cannot have 0 message size, clamping to 1" << std::endl;
    message_size = 1;
  }

  DDS_DomainId_t domain = DDS_DOMAIN_ID_DEFAULT;
	this->i = 0;
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
  LargeMsg_LargeMessageTypeSupport chatMessageTS = LargeMsg_LargeMessageTypeSupport__alloc();
  if (!chatMessageTS) {
    printf ("Allocating TypeSupport failed!!\n");
    exit(-1);
  }

  char *chatMessageTypeName = LargeMsg_LargeMessageTypeSupport_get_type_name(chatMessageTS);
  status = LargeMsg_LargeMessageTypeSupport_register_type(chatMessageTS, dp, chatMessageTypeName);
  if (status != DDS_RETCODE_OK) {
    printf("Registering data type failed. Status = %d\n", status);
    exit(-1);
  };
  printf("Registered data type.\n");
  DDS_free(chatMessageTS);

  /* Set QoS policty to BEST_EFFORT */
  //topic_qos = DDS_TopicQos__alloc();
  status = DDS_DomainParticipant_get_default_topic_qos(dp, &topic_qos);
  checkStatus(status, "DDS_DomainParticipant_get_default_topic_qos");
  topic_qos.reliability.kind = DDS_BEST_EFFORT_RELIABILITY_QOS;
  //topic_qos->reliability.kind = DDS_RELIABLE_RELIABILITY_QOS;

  /* Make the tailored QoS the new default. */
  status = DDS_DomainParticipant_set_default_topic_qos(dp,
      &topic_qos);
  checkStatus(status, "DDS_DomainParticipant_set_default_topic_qos");

  /*Create the LargeMessage topic */
  this->chatMessageTopic = DDS_DomainParticipant_create_topic(
    dp,
    "LargeMsg_LargeMessage",
    chatMessageTypeName,
    &topic_qos,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!chatMessageTopic) {
    printf("Creating LargeMessage topic failed!!\n");
    exit(-1);
  };
  printf("Created LargeMessage topic.\n");

  /* Adapt the default PublisherQos to write into the
     "LargeMessageRoom" Partition. */

  const char *partitionName = "chatter";

  /*
  pub_qos = DDS_PublisherQos__alloc();
  if (!pub_qos) {
    printf("Allocating PublisherQos failed!!\n");
    exit(-1);
  }*/

  status = DDS_DomainParticipant_get_default_publisher_qos (
    dp, &pub_qos);
  if (status != DDS_RETCODE_OK) {
    printf("Getting default publisher qos failed!!\n");
    exit(-1);
  }
  pub_qos.partition.name._length = 1;
  pub_qos.partition.name._maximum = 1;
  pub_qos.partition.name._buffer = DDS_StringSeq_allocbuf(1);
  if (!pub_qos.partition.name._buffer) {
    printf("Allocating partition name failed!!\n");
    exit(-1);
  }
  pub_qos.partition.name._buffer[0] = DDS_string_alloc(strlen(partitionName));
  if (!pub_qos.partition.name._buffer[0]) {
    printf("Allocating partition name failed!!\n");
    exit(-1);
  }
  strcpy(pub_qos.partition.name._buffer[0], partitionName);

  /* Create a Publisher for the chatter application. */
  this->chatPublisher = DDS_DomainParticipant_create_publisher(
    dp, &pub_qos, NULL, DDS_STATUS_MASK_NONE);
  if (!chatPublisher) {
    printf("Creating publisher failed!!\n");
    exit(-1);
  }
  printf("Created publisher.\n");

  /* Create a DataWriter for the LargeMessage Topic
     (using the appropriate QoS). */
  this->talker = DDS_Publisher_create_datawriter(
    chatPublisher,
    chatMessageTopic,
    DDS_DATAWRITER_QOS_USE_TOPIC_QOS,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!talker) {
    printf("Creating datawriter failed!!\n");
    exit(-1);
  }
  checkHandle(talker, "DDS_Publisher_create_datawriter");
  printf("Created datawriter.\n");

  int ownID = 0;

  //this->msg = LargeMsg_LargeMessage__alloc();
  //checkHandle(msg, "LargeMsg_LargeMessage__alloc");
  msg.seq = 0;
  //const unsigned int message_size_const = message_size;
  msg.content = DDS_string_alloc(message_size);
  checkHandle(msg.content, "DDS_string__alloc");
  int j;
  for (j < 0; j < this->message_size; ++j)
  {
    msg.content[j] = '.';
  }

  // int j;
  // snprintf(msg->content, message_size_const, msg_content, msg->seq);

  // register a chat message

  userHandle = LargeMsg_LargeMessageDataWriter_register_instance(talker, &msg);
  printf("Created user handle and preallocated message.\n");
	return true;
}

void ExamplePublisher::callback()
{
  this->msg.seq = this->i;

  int j;
  for (j < 0; j < this->message_size; ++j)
  {
    msg.content[j] = i;
  }

  status = LargeMsg_LargeMessageDataWriter_write(talker, &msg, userHandle);
  checkStatus(status, "LargeMsg_LargeMessageDataWriter_write");
  ++i;
}


bool ExamplePublisher::teardown()
{
  /* Remove the DataWriters */
  status = DDS_Publisher_delete_datawriter(this->chatPublisher,
    this->talker);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting datawriter failed!!\n");
    exit(-1);
  }
  printf("Deleted datawriter.\n");

  /* Remove the Publisher. */
  status = DDS_DomainParticipant_delete_publisher(
    this->dp, this->chatPublisher);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting publisher failed!!\n");
    exit(-1);
  }
  /* De-allocate the PublisherQoS holder. */
  //DDS_free(this->pub_qos); // Note that DDS_free recursively
                     // de-allocates all indirections!!
  printf("Deleted publisher.\n");

  /* Deleting the Topic. */
  status = DDS_DomainParticipant_delete_topic(
    dp, chatMessageTopic);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting topic failed. Status = %d\n", status);
    exit(-1);
  };
  //DDS_free(this->topic_qos);
  printf("Deleted LargeMessage topic.\n");

  /* Deleting the DomainParticipant */
  status = DDS_DomainParticipantFactory_delete_participant(
      this->dpf, this->dp);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting participant failed. Status = %d\n", status);
    exit(-1);
  }

  printf("Deleted Participant.\n");
}
