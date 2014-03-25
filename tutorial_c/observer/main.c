#include "dds_dcps.h"
#include "unistd.h"

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

int main (
  int argc,
  char *argv[])
{
  DDS_DomainParticipantFactory dpf;
  DDS_DomainParticipant dp;
  DDS_DomainId_t domain = DDS_DOMAIN_ID_DEFAULT;
  DDS_ReturnCode_t status;

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

  //DDS_DomainParticipant_get_publishers();

  DDS_DomainId_t domain_id = DDS_DomainParticipant_get_domain_id(dp);
  printf("Domain id: %d\n", domain_id);

  DDS_DomainParticipant dp2 = DDS_DomainParticipantFactory_create_participant (
    dpf,
    0x7ffffffe,
    DDS_PARTICIPANT_QOS_DEFAULT,
    NULL,
    DDS_STATUS_MASK_NONE);
  if (!dp) {
    printf("Creating Participant failed!!\n");
    exit(-1);
  }
  DDS_DomainId_t domain_id2 = DDS_DomainParticipant_get_domain_id(dp2);
  printf("Domain id 2: %d\n", domain_id2);

  //DDS_DomainParticipant_get_builtin_subscriber

  DDS_Subscriber builtinSubscriber = DDS_DomainParticipant_get_builtin_subscriber(dp);
  printf("a\n");
  DDS_ParticipantBuiltinTopicDataDataReader *participantsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSParticipant");
  printf("b\n");
  DDS_PublicationBuiltinTopicDataDataReader *publicationsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSPublication");
  printf("c\n");
  DDS_SubscriptionBuiltinTopicDataDataReader *subscriptionsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSSubscription");
  printf("d\n");
  DDS_TopicBuiltinTopicDataDataReader *topicsDR = DDS_Subscriber_lookup_datareader(builtinSubscriber, "DCPSTopic");
  printf("e\n");

  printf("wait_for_historical_data\n");

  DDS_Duration_t wait_duration;
  wait_duration.sec = 30;
  wait_duration.nanosec = 0;
  DDS_DataReader_wait_for_historical_data(participantsDR, &wait_duration);
  printf("f\n");
  DDS_DataReader_wait_for_historical_data(publicationsDR, &wait_duration);
  printf("g\n");
  DDS_DataReader_wait_for_historical_data(subscriptionsDR, &wait_duration);
  printf("h\n");
  DDS_DataReader_wait_for_historical_data(topicsDR, &wait_duration);
  printf("h\n");

  /*
  DDS_PublicationBuiltinTopicData *data_values = DDS_PublicationBuiltinTopicData__alloc();
  DDS_SampleInfo sample_info;
  while (1) {
    status = DDS_PublicationBuiltinTopicDataDataReader_read_next_sample(
      publicationsDR,
      data_values,
      &sample_info);
      if (status != DDS_RETCODE_OK) {
        printf("Reading next sample failed. Status = %d: %s\n", status, RetCodeName[status]);
        break;
    };
    printf("Read sample\n");
  };
  */

  {
    DDS_sequence_DDS_ParticipantBuiltinTopicData *data_values = DDS_sequence_DDS_ParticipantBuiltinTopicData__alloc();
    DDS_SampleInfoSeq *info_seq = DDS_sequence_DDS_SampleInfo__alloc();
    while (1) {
      status =   DDS_ParticipantBuiltinTopicDataDataReader_read (
        participantsDR,
        data_values,
        info_seq,
        DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE,
        DDS_ANY_VIEW_STATE,
        DDS_ANY_INSTANCE_STATE);

        if (status != DDS_RETCODE_OK) {
          printf("Reading failed. Status = %d: %s\n", status, RetCodeName[status]);
          break;
      };
      printf("Read %d items from ParticipantBuiltinTopic\n", data_values->_length);
      int i = 0;
      for (;i < data_values->_length; i++) {
        DDS_ParticipantBuiltinTopicData data = data_values->_buffer[i];
        DDS_SampleInfo info = info_seq->_buffer[i];
        printf("%d: src_ts=%d:%d dst_ts=%d:%d\n", i, info.source_timestamp.sec, info.source_timestamp.nanosec, info.reception_timestamp.sec, info.reception_timestamp.nanosec);
      }
    }
  }

  {
    DDS_sequence_DDS_PublicationBuiltinTopicData *data_values = DDS_sequence_DDS_PublicationBuiltinTopicData__alloc();
    DDS_SampleInfoSeq *info_seq = DDS_sequence_DDS_SampleInfo__alloc();
    while (1) {
      status =   DDS_PublicationBuiltinTopicDataDataReader_read (
        publicationsDR,
        data_values,
        info_seq,
        DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE,
        DDS_ANY_VIEW_STATE,
        DDS_ANY_INSTANCE_STATE);

        if (status != DDS_RETCODE_OK) {
          printf("Reading failed. Status = %d: %s\n", status, RetCodeName[status]);
          break;
      };
      printf("Read %d items from PublicationBuiltinTopic\n", data_values->_length);
      int i = 0;
      for (;i < data_values->_length; i++) {
        DDS_PublicationBuiltinTopicData data = data_values->_buffer[i];
        DDS_SampleInfo info = info_seq->_buffer[i];
        printf("%d: topic_name=%s, type_name=%s, src_ts=%d:%d dst_ts=%d:%d\n", i, data.topic_name, data.type_name, info.source_timestamp.sec, info.source_timestamp.nanosec, info.reception_timestamp.sec, info.reception_timestamp.nanosec);
      }
    }
  }

  {
    DDS_sequence_DDS_SubscriptionBuiltinTopicData *data_values = DDS_sequence_DDS_SubscriptionBuiltinTopicData__alloc();
    DDS_SampleInfoSeq *info_seq = DDS_sequence_DDS_SampleInfo__alloc();
    while (1) {
      status =   DDS_SubscriptionBuiltinTopicDataDataReader_read (
        subscriptionsDR,
        data_values,
        info_seq,
        DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE,
        DDS_ANY_VIEW_STATE,
        DDS_ANY_INSTANCE_STATE);

        if (status != DDS_RETCODE_OK) {
          printf("Reading failed. Status = %d: %s\n", status, RetCodeName[status]);
          break;
      };
      printf("Read %d items from SubscriptionBuiltinTopic\n", data_values->_length);
      int i = 0;
      for (;i < data_values->_length; i++) {
        DDS_SubscriptionBuiltinTopicData data = data_values->_buffer[i];
        DDS_SampleInfo info = info_seq->_buffer[i];
        printf("%d: topic_name=%s, type_name=%s, src_ts=%d:%d dst_ts=%d:%d\n", i, data.topic_name, data.type_name, info.source_timestamp.sec, info.source_timestamp.nanosec, info.reception_timestamp.sec, info.reception_timestamp.nanosec);
      }
    }
  }

  {
    DDS_sequence_DDS_TopicBuiltinTopicData *data_values = DDS_sequence_DDS_TopicBuiltinTopicData__alloc();
    DDS_SampleInfoSeq *info_seq = DDS_sequence_DDS_SampleInfo__alloc();
    while (1) {
      status =   DDS_TopicBuiltinTopicDataDataReader_read (
        topicsDR,
        data_values,
        info_seq,
        DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE,
        DDS_ANY_VIEW_STATE,
        DDS_ANY_INSTANCE_STATE);

        if (status != DDS_RETCODE_OK) {
          printf("Reading failed. Status = %d: %s\n", status, RetCodeName[status]);
          break;
      };
      printf("Read %d items from TopicBuiltinTopic\n", data_values->_length);
      int i = 0;
      for (;i < data_values->_length; i++) {
        DDS_TopicBuiltinTopicData data = data_values->_buffer[i];
        DDS_SampleInfo info = info_seq->_buffer[i];
        printf("%d: topic_name=%s, type_name=%s, src_ts=%d:%d dst_ts=%d:%d\n", i, data.name, data.type_name, info.source_timestamp.sec, info.source_timestamp.nanosec, info.reception_timestamp.sec, info.reception_timestamp.nanosec);
      }
    }
  }


  /*
  DDS_sequence_Chat_ChatMessage *msgSeq = DDS_sequence_Chat_ChatMessage__alloc();
  //checkHandle(msgSeq, "DDS_sequence_Chat_NamedMessage__alloc");
  DDS_SampleInfoSeq *infoSeq = DDS_SampleInfoSeq__alloc();
  //checkHandle(infoSeq, "DDS_SampleInfoSeq__alloc");
  DDS_unsigned_long i;
  DDS_boolean terminated = FALSE;
  while (!terminated) {
  status = Chat_ChatMessageDataReader_take(
  mbReader,
  msgSeq,
  infoSeq,
  DDS_LENGTH_UNLIMITED,
  DDS_ANY_SAMPLE_STATE,
  DDS_ANY_VIEW_STATE,
  DDS_ALIVE_INSTANCE_STATE);
  checkStatus(status, "Chat_NamedMessageDataReader_take");
  for (i = 0; i < msgSeq->_length; i++) {
  Chat_ChatMessage *msg = &(msgSeq->_buffer[i]);
  if (msg->userID == TERMINATION_MESSAGE) {
  printf("Termination message received: exiting...\n");
  terminated = TRUE;
  } else {
  printf ("%s: %s\n", msg->userName, msg->content);
  }
  }
  status = Chat_ChatMessageDataReader_return_loan(
  mbReader, msgSeq, infoSeq);
  checkStatus(
  status, "Chat_ChatMessageDataReader_return_loan");
  usleep(100000);
  }
  */

  /*
  int i = 0;
  for(; i < 1; i++) {
    DDS_InstanceHandleSeq *participant_handles = DDS_InstanceHandleSeq__alloc();
    status = DDS_DomainParticipant_get_discovered_participants(
      dp,
      participant_handles);
    if (status != DDS_RETCODE_OK) {
      printf("Getting domain participants failed. Status = %d\n", status);
      exit(-1);
    };
    printf("Got sequence of domain participant: %d.\n", participant_handles->_maximum);
    int j = 0;
    for (; j < participant_handles->_maximum; j++) {
      printf("%d\n", j);
      DDS_InstanceHandle_t participant_handle = participant_handles->_buffer[j];

      DDS_ParticipantBuiltinTopicData *participant_data = DDS_sequence_DDS_ParticipantBuiltinTopicData_allocbuf(10);
      status = DDS_DomainParticipant_get_discovered_participant_data(
        dp,
        participant_data,
        participant_handle);
      if (status != DDS_RETCODE_OK) {
        printf("Getting domain participant data failed. Status = %d\n", status);
      } else {
        DDS_BuiltinTopicKey_t key;
        key[0] = participant_data->key[0];  // seems to be participant specific, 5 times the same value
        key[1] = participant_data->key[1];
        key[2] = participant_data->key[2];  // always 1 until now
        DDS_UserDataQosPolicy user_data = participant_data->user_data;
        DDS_sequence_octet value = user_data.value;
        char* str = malloc(value._length + 1);
        int k = 0;
        for (;k < value._length; k++) {
          str[k] = value._buffer[k];
        }
        str[value._length] = '\0';
        printf("Getting domain participant data succeeded: %d %d %d = %d: '%s'\n", key[0], key[1], key[2], value._length, str);
      }

      // if (DDS_InstanceHandle_is_nil(participant_handle)) {
      //   printf("%d: is nil\n", j);
      // }
      // char *handleName;
      // status = DDS_InstanceHandle_ToString(participant_handle, handleName);
      // if (status != DDS_RETCODE_OK) {
      //   printf("Getting string of handle failed. Status = %d\n", status);
      //   exit(-1);
      // };
      // printf("%d: %s\n", j, handleName);
    }

    sleep(1);
  }

  DDS_Subscriber builtInTopicsSubscriber = DDS_DomainParticipant_get_builtin_subscriber(dp);
  DDS_DataReader builtInTopicsReader = DDS_Subscriber_lookup_datareader(builtInTopicsSubscriber, "DCPSParticipant");
  */


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
