#include "ccpp_dds_dcps.h"
#include "check_status.h"

#include "ccpp_LargeMsg.h"

class ExampleSubscriber
{
  private:
    DDS::DomainParticipantFactory_var dpf;
    DDS::DomainParticipant_var participant;
    DDS::Subscriber_var subscriber;
    LargeMsg::LargeMessageDataReader_var data_reader;
    DDS::Topic_var large_message_topic;
    char * large_message_type_name;
    LargeMsg::LargeMessageSeq_var large_msg_seq;
    DDS::SampleInfoSeq_var sample_info_seq;
    bool subscriber_running;
    DDS::ReturnCode_t status;
    DDS::DomainId_t domain;
    const char * partition_name;
    const char * topic_name;
    DDS::TopicQos default_topic_qos;
    LargeMsg::LargeMessageTypeSupport_var large_message_ts;
    DDS::SubscriberQos sub_qos;
    DDS::DataReader_var topic_reader;

  public:
    unsigned long msgs_count;
    void callback();
    bool init();
    bool teardown();
};


bool ExampleSubscriber::init()
{
  this->msgs_count = 0;
  domain = DDS::DOMAIN_ID_DEFAULT;
  partition_name = "Default";
  topic_name = "big_chatter";

  /* Create Domain Participant Factory */
  this->dpf = DDS::DomainParticipantFactory::get_instance();
  checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");

  /* Create Domain Participant */
  std::cout << "Creating domain participant in subscriber" << std::endl;
  this->participant = dpf->create_participant(
      domain,
      PARTICIPANT_QOS_DEFAULT,
      NULL,
      DDS::STATUS_MASK_NONE
  );
  checkHandle(participant.in(), "DDS::DomainParticipantFactory::create_participant");


  /* Create a default QoS for Topics */
  status = participant->get_default_topic_qos(default_topic_qos);
  checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
  // default_topic_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
  default_topic_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

  /* Register the LargeMessage Type */
  large_message_ts = new LargeMsg::LargeMessageTypeSupport();
  checkHandle(large_message_ts.in(), "new LargeMessageTypeSupport");
  this->large_message_type_name = large_message_ts->get_type_name();
  status = large_message_ts->register_type(participant.in(), large_message_type_name);
  checkStatus(status, "LargeMsg::LargeMessageTypeSupport::register_type");

  /* Setup the Subscribers's QoS */
  status = participant->get_default_subscriber_qos(sub_qos);
  checkStatus(status, "DDS::DomainParticipant::get_default_subscriber_qos");
  sub_qos.partition.name.length(1);
  sub_qos.partition.name[0] = partition_name;

  /* Create the subscriber */
  this->subscriber = participant->create_subscriber(
      sub_qos,
      NULL,
      DDS::STATUS_MASK_NONE
  );
  checkHandle(subscriber.in(), "DDS::DomainParticipant::create_subscriber");

  /* Create the Topic */
  this->large_message_topic = participant->create_topic(
      topic_name,
      large_message_type_name,
      default_topic_qos,
      NULL,
      DDS::STATUS_MASK_NONE
  );
  checkHandle(large_message_topic.in(), "DDS::DomainParticipant::create_topic(LargeMessage)");

  /* Create Topic specific DataReader */
  topic_reader = subscriber->create_datareader(
        large_message_topic.in(),
        DATAREADER_QOS_USE_TOPIC_QOS,
        NULL,
        DDS::STATUS_MASK_NONE
  );
  checkHandle(topic_reader.in(), "DDS::Subscriber::create_datareader");

  /* Narrow topic_reader down to LargeMessage specific DataReader */
  this->data_reader = LargeMsg::LargeMessageDataReader::_narrow(topic_reader.in());
  checkHandle(data_reader.in(), "LargeMsg::LargeMessageDataReader::_narrow");

  this->large_msg_seq = new LargeMsg::LargeMessageSeq();
  this->sample_info_seq = new DDS::SampleInfoSeq();

  std::cout << "Polling DataReader..." << std::endl;
  return true;
}

void ExampleSubscriber::callback()
{
  status = data_reader->take(
      large_msg_seq,
      sample_info_seq,
      DDS::LENGTH_UNLIMITED,
      DDS::ANY_SAMPLE_STATE,
      DDS::ANY_VIEW_STATE,
      DDS::ALIVE_INSTANCE_STATE
  );
  checkStatus(status, "LargeMsg::LargeMessageDataReader::take");

  for (DDS::ULong i = 0; i < large_msg_seq->length(); i++)
  {
    //msg = &(large_msg_seq[i]);
    ++this->msgs_count;
  }

  status = data_reader->return_loan(large_msg_seq, sample_info_seq);
  checkStatus(status, "LargeMsg::LargeMessageDataReader::return_loan");
}

bool ExampleSubscriber::teardown()
{
  /* Shutdown */
  if (participant != NULL)
  {
    if (subscriber.in() != NULL) {
      status = participant->delete_subscriber(subscriber.in());
      checkStatus(status, "DDS::DomainParticipant::delete_subscriber");
    }

    status = participant->delete_topic(large_message_topic.in());
    checkStatus(status, "DDS::DomainParticipant::delete_topic (large_message_topic)");

    status = dpf->delete_participant(participant.in());
    checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");
  }

  DDS::string_free(large_message_type_name);

  return true;
}

