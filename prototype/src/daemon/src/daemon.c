#include "dds_dcps.h"
#include "unistd.h"

#include <stdio.h>
#include <time.h>

#if __APPLE__
#include <sys/time.h>
#endif

char *RetCodeName[13] = {
    "DDS_RETCODE_OK",
    "DDS_RETCODE_ERROR",
    "DDS_RETCODE_UNSUPPORTED",
    "DDS_RETCODE_BAD_PARAMETER",
    "DDS_RETCODE_PRECONDITION_NOT_MET",
    "DDS_RETCODE_OUT_OF_RESOURCES",
    "DDS_RETCODE_NOT_ENABLED",
    "DDS_RETCODE_IMMUTABLE_POLICY",
    "DDS_RETCODE_INCONSISTENT_POLICY",
    "DDS_RETCODE_ALREADY_DELETED",
    "DDS_RETCODE_TIMEOUT",
    "DDS_RETCODE_NO_DATA",
    "DDS_RETCODE_ILLEGAL_OPERATION" };

void _print_time()
{
  int sec;
  long ms;
#if __APPLE__
  struct timeval tv;
  gettimeofday(&tv, NULL);
  ms = tv.tv_usec / 1.0e3;
  sec = (int)tv.tv_sec;
#else
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  ms = spec.tv_nsec / 1.0e6;
  sec = (int)spec.tv_sec;
#endif

  printf("time: %d.%03ld\n", sec, ms);
}

DDS_DomainParticipantFactory dpf;
DDS_DomainParticipant dp;

int create_participant()
{
  DDS_DomainId_t domain = DDS_DOMAIN_ID_DEFAULT;

  /* Create a DomainParticipantFactory and a DomainParticipant */
  /* (using Default QoS settings). */
  dpf = DDS_DomainParticipantFactory_get_instance();
  if (!dpf) {
    printf("Creating ParticipantFactory failed!!\n");
    return 1;
  }
  printf("Created ParticipantFactory.\n");

  dp = DDS_DomainParticipantFactory_create_participant(
    dpf,
    domain,
    DDS_PARTICIPANT_QOS_DEFAULT,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!dp) {
    printf("Creating Participant failed!! (did you setup the OSPL environment?)\n");
    return 1;
  }
  printf("Created Participant.\n");

  DDS_DomainId_t domain_id = DDS_DomainParticipant_get_domain_id(dp);
  printf("Domain id: %d\n", domain_id);

  return 0;
}

DDS_ParticipantBuiltinTopicDataDataReader *participantsDR;
DDS_PublicationBuiltinTopicDataDataReader *publicationsDR;
DDS_SubscriptionBuiltinTopicDataDataReader *subscriptionsDR;
DDS_TopicBuiltinTopicDataDataReader *topicsDR;

void wait_for_historical_data()
{
  DDS_Subscriber builtinSubscriber = DDS_DomainParticipant_get_builtin_subscriber(dp);
  printf("get_builtin_subscriber()\n");
  participantsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSParticipant");
  printf("lookup_datareader DCPSParticipant\n");
  publicationsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSPublication");
  printf("lookup_datareader DCPSPublication\n");
  subscriptionsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSSubscription");
  printf("lookup_datareader DCPSSubscription\n");
  topicsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSTopic");
  printf("lookup_datareader DCPSTopic\n");

  printf("wait_for_historical_data\n");
  _print_time();

  DDS_Duration_t wait_duration;
  wait_duration.sec = 30;
  wait_duration.nanosec = 0;
  DDS_DataReader_wait_for_historical_data(participantsDR, &wait_duration);
  printf("wait_for_historical_data DCPSParticipant\n");
  DDS_DataReader_wait_for_historical_data(publicationsDR, &wait_duration);
  printf("wait_for_historical_data DCPSPublication\n");
  DDS_DataReader_wait_for_historical_data(subscriptionsDR, &wait_duration);
  printf("wait_for_historical_data DCPSSubscription\n");
  DDS_DataReader_wait_for_historical_data(topicsDR, &wait_duration);
  printf("wait_for_historical_data DCPSTopic\n");
  _print_time();
}

int get_topics(char* buffer, int max_size)
{
  char* p = buffer;
  int size = 0;

  DDS_sequence_DDS_TopicBuiltinTopicData *data_values = DDS_sequence_DDS_TopicBuiltinTopicData__alloc();
  DDS_SampleInfoSeq *info_seq = DDS_sequence_DDS_SampleInfo__alloc();
  while (1) {
    DDS_ReturnCode_t status =   DDS_TopicBuiltinTopicDataDataReader_read (
      topicsDR,
      data_values,
      info_seq,
      DDS_LENGTH_UNLIMITED,
      DDS_ANY_SAMPLE_STATE,
      DDS_ANY_VIEW_STATE,
      DDS_ANY_INSTANCE_STATE);

    if (status != DDS_RETCODE_OK) {
      printf("Reading failed. Status = %d: %s\n", status, RetCodeName[status]);
      return 0;
    };
    printf("Read %d items from TopicBuiltinTopic\n", data_values->_length);
    int i = 0;
    for (;i < data_values->_length; i++) {
      if (i > 0) {
        if (size == max_size) {
          printf("Too many topic names for return buffer\n");
          return 1;
        }
        memcpy(p, ",", 1);
        p += 1;
        size += 1;
      }
      DDS_TopicBuiltinTopicData data = data_values->_buffer[i];
      DDS_SampleInfo info = info_seq->_buffer[i];
      printf("%d: topic_name=%s, type_name=%s, src_ts=%d:%d dst_ts=%d:%d\n", i, data.name, data.type_name, info.source_timestamp.sec, info.source_timestamp.nanosec, info.reception_timestamp.sec, info.reception_timestamp.nanosec);
      if (size + strlen(data.name) > max_size) {
        printf("Too many topic names for return buffer\n");
        return 1;
      }
      memcpy(p, data.name, strlen(data.name));
      p += strlen(data.name);
      size += strlen(data.name);
    }
  }
  return 0;
}

int delete_participant()
{
  /* Deleting the DomainParticipant */
  DDS_ReturnCode_t status = DDS_DomainParticipantFactory_delete_participant(dpf, dp);
  if (status != DDS_RETCODE_OK) {
    printf("Deleting participant failed. Status = %d\n", status);
    return 1;
  };
  printf("Deleted Participant.\n");

  /* Everything is fine, return normally. */
  return 0;
}
