#ifdef USE_OPENSPLICE
#include "dds_dcps.h"
#elif USE_CONNEXT
#include <ndds/ndds_c.h>
#elif USE_QEO
#include "dds/dds_dcps.h"
#define DDS_STATUS_MASK_NONE 0
#else
#error "Unsupported DDS vendor"
#endif

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

// defined in OpenSplice
// not defined in Connext
#ifndef DDS_DOMAIN_ID_DEFAULT
#define DDS_DOMAIN_ID_DEFAULT 0
#endif

#if defined(USE_OPENSPLICE) || defined(USE_CONNEXT)
DDS_DomainParticipantFactory* dpf;
DDS_DomainParticipant* dp;
#else
DDS_DomainParticipant dp;
#endif

int create_participant()
{
  _print_time();

  DDS_DomainId_t domain = DDS_DOMAIN_ID_DEFAULT;

#if defined(USE_OPENSPLICE) || defined(USE_CONNEXT)
  /* Create a DomainParticipantFactory and a DomainParticipant */
  /* (using Default QoS settings). */
  dpf = DDS_DomainParticipantFactory_get_instance();
  if (!dpf) {
    printf("Creating ParticipantFactory failed!!\n");
    return 1;
  }
  printf("Created ParticipantFactory.\n");
#endif

#if defined(USE_OPENSPLICE) || defined(USE_QEO)
  // OpenSplice without struct
  DDS_DomainParticipantFactoryQos factory_qos;
#elif USE_CONNEXT
  // Connext with struct
  struct DDS_DomainParticipantFactoryQos factory_qos = DDS_DomainParticipantFactoryQos_INITIALIZER;
#endif

#ifdef USE_QEO
  DDS_ReturnCode_t status = DDS_DomainParticipantFactory_get_qos(&factory_qos);
#else
  DDS_ReturnCode_t status = DDS_DomainParticipantFactory_get_qos(dpf, &factory_qos);
#endif
  if (status != DDS_RETCODE_OK) {
    printf("Get qos failed. Status = %d: %s\n", status, RetCodeName[status]);
    return 1;
  };
  factory_qos.entity_factory.autoenable_created_entities = 0;
#ifdef USE_QEO
  status = DDS_DomainParticipantFactory_set_qos(&factory_qos);
#else
  status = DDS_DomainParticipantFactory_set_qos(dpf, &factory_qos);
#endif
  if (status != DDS_RETCODE_OK) {
    printf("Set qos failed. Status = %d: %s\n", status, RetCodeName[status]);
    return 1;
  };

  dp = DDS_DomainParticipantFactory_create_participant(
#ifndef USE_QEO
    dpf,
#endif
    domain,
#if defined(USE_OPENSPLICE) || defined(USE_QEO)
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

  status = DDS_Entity_enable((DDS_Entity*)dp);
  if (status != DDS_RETCODE_OK) {
    printf("Enable failed. Status = %d: %s\n", status, RetCodeName[status]);
    return 1;
  };

  return 0;
}

#ifdef USE_QEO
DDS_DataReader participantsDR;
DDS_DataReader publicationsDR;
DDS_DataReader subscriptionsDR;
DDS_DataReader topicsDR;
#else
DDS_DataReader *participantsDR;
DDS_DataReader *publicationsDR;
DDS_DataReader *subscriptionsDR;
DDS_DataReader *topicsDR;
#endif

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

void wait_for_historical_data()
{
  _print_time();

#ifdef USE_QEO
  DDS_Subscriber builtinSubscriber = DDS_DomainParticipant_get_builtin_subscriber(dp);
#else
  DDS_Subscriber* builtinSubscriber = DDS_DomainParticipant_get_builtin_subscriber(dp);
#endif
  printf("get_builtin_subscriber()\n");
  participantsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, DDS_PARTICIPANT_TOPIC_NAME);//"DCPSParticipant");
  printf("lookup_datareader DCPSParticipant\n");
  publicationsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, DDS_PUBLICATION_TOPIC_NAME);//"DCPSPublication");
  printf("lookup_datareader DCPSPublication\n");
  subscriptionsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, DDS_SUBSCRIPTION_TOPIC_NAME);//"DCPSSubscription");
  printf("lookup_datareader DCPSSubscription\n");
  topicsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, DDS_TOPIC_TOPIC_NAME);//"DCPSTopic");
  printf("lookup_datareader DCPSTopic\n");

  printf("wait_for_historical_data\n");

#if defined(USE_OPENSPLICE) || defined(USE_QEO)
  // OpenSplice and Qeo without struct
  DDS_Duration_t wait_duration;
#elif USE_CONNEXT
  // Connext with struct
  struct DDS_Duration_t wait_duration;
#endif
  wait_duration.sec = 30;
  wait_duration.nanosec = 0;
  DDS_DataReader_wait_for_historical_data(participantsDR, &wait_duration);
  printf("wait_for_historical_data DCPSParticipant\n");
  //DDS_DataReader_wait_for_historical_data(publicationsDR, &wait_duration);
  //printf("wait_for_historical_data DCPSPublication\n");
  //DDS_DataReader_wait_for_historical_data(subscriptionsDR, &wait_duration);
  //printf("wait_for_historical_data DCPSSubscription\n");
  //DDS_DataReader_wait_for_historical_data(topicsDR, &wait_duration);
  //printf("wait_for_historical_data DCPSTopic\n");
}

int get_topics(char* buffer, int max_size)
{
  _print_time();

  char* p = buffer;
  int size = 0;

#if defined(USE_OPENSPLICE) || defined(USE_QEO)
  // OpenSplice and Qeo without struct
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

#if defined(USE_OPENSPLICE) || defined(USE_QEO)
    // OpenSplice and Qeo _buffer
    DDS_InstanceHandle_t handle = seq._buffer[i];
#elif USE_CONNEXT
    // Connext _contiguous_buffer
    DDS_InstanceHandle_t handle = seq._contiguous_buffer[i];
#endif
#if defined(USE_OPENSPLICE) || defined(USE_QEO)
    // OpenSplice without struct
    DDS_TopicBuiltinTopicData data;
#elif USE_CONNEXT
    // Connext with struct
    struct DDS_TopicBuiltinTopicData data;
#endif
    DDS_DomainParticipant_get_discovered_topic_data(
      dp,
      &data,
#if defined(USE_OPENSPLICE) || defined(USE_QEO)
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
  _print_time();

  /* Deleting the DomainParticipant */
#ifdef USE_QEO
  DDS_ReturnCode_t status = DDS_DomainParticipantFactory_delete_participant(dp);
#else
  DDS_ReturnCode_t status = DDS_DomainParticipantFactory_delete_participant(dpf, dp);
#endif
  if (status != DDS_RETCODE_OK) {
    printf("Deleting participant failed. Status = %d\n", status);
    return 1;
  };
  printf("Deleted Participant.\n");

  _print_time();

  /* Everything is fine, return normally. */
  return 0;
}
