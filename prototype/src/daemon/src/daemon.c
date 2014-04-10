#ifdef USE_OPENSPLICE
#include "dds_dcps.h"
#elif USE_CONNEXT
#include <ndds/ndds_c.h>
#else
#error "Unsupported DDS vendor"
#endif

#include "unistd.h"

#include <stdio.h>
#include <time.h>

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
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);

  long ms = spec.tv_nsec / 1.0e6;
  printf("time: %d.%03ld\n", (int)spec.tv_sec, ms);
}

// defined in OpenSplice
// not defined in Connext
#ifndef DDS_DOMAIN_ID_DEFAULT
#define DDS_DOMAIN_ID_DEFAULT 0
#endif

DDS_DomainParticipantFactory* dpf;
DDS_DomainParticipant* dp;

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
#ifdef USE_OPENSPLICE
    // OpenSplice without &
    DDS_PARTICIPANT_QOS_DEFAULT,
#elif USE_CONNEXT
    // Connext with &
    &DDS_PARTICIPANT_QOS_DEFAULT,
#endif
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

DDS_DataReader *participantsDR;
DDS_DataReader *publicationsDR;
DDS_DataReader *subscriptionsDR;
DDS_DataReader *topicsDR;

// defined in Connext
// not defined in OpenSplice
#ifndef DDS_PARTICIPANT_TOPIC_NAME
#define DDS_PARTICIPANT_TOPIC_NAME "DCPSParticipant"
#endif
#ifndef DDS_PUBLICATION_TOPIC_NAME
#define DDS_PUBLICATION_TOPIC_NAME "DCPSPublication"
#endif
#ifndef DDS_SUBSCRIPTION_TOPIC_NAME
#define DDS_SUBSCRIPTION_TOPIC_NAME "DCPSSubscription"
#endif
#ifndef DDS_TOPIC_TOPIC_NAME
#define DDS_TOPIC_TOPIC_NAME "DCPSTopic"
#endif

int get_topics(char* buffer, int max_size)
{
  char* p = buffer;
  int size = 0;

#ifdef USE_OPENSPLICE
  // OpenSplice without struct
  DDS_InstanceHandleSeq seq;
#elif USE_CONNEXT
  // Connext with struct
  struct DDS_InstanceHandleSeq seq;
#endif
  DDS_ReturnCode_t status = DDS_DomainParticipant_get_discovered_topics(dp, &seq);
  if (status != DDS_RETCODE_OK) {
    printf("Reading failed. Status = %d: %s\n", status, RetCodeName[status]);
    return 0;
  };

  int i = 0;
  for (;i < seq._length; i++) {
    if (i > 0) {
      if (size == max_size) {
        printf("Too many topic names for return buffer\n");
        return 1;
      }
      memcpy(p, ",", 1);
      p += 1;
      size += 1;
    }

#ifdef USE_OPENSPLICE
    // OpenSplice _buffer
    DDS_InstanceHandle_t handle = seq._buffer[i];
#elif USE_CONNEXT
    // Connext _contiguous_buffer
    DDS_InstanceHandle_t handle = seq._contiguous_buffer[i];
#endif
#ifdef USE_OPENSPLICE
    // OpenSplice without struct
    DDS_TopicBuiltinTopicData data;
#elif USE_CONNEXT
    // Connext with struct
    struct DDS_TopicBuiltinTopicData data;
#endif
    DDS_DomainParticipant_get_discovered_topic_data(
      dp,
      &data,
#ifdef USE_OPENSPLICE
      // OpenSplice without &
      handle
#elif USE_CONNEXT
      // Connext with &
      &handle
#endif
    );
    printf("%d: topic_name=%s\n", i, data.name);

    if (size + strlen(data.name) > max_size) {
      printf("Too many topic names for return buffer\n");
      return 1;
    }
    memcpy(p, data.name, strlen(data.name));
    p += strlen(data.name);
    size += strlen(data.name);
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
