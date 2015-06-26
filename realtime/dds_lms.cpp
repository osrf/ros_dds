#include <signal.h>

#include "ccpp_dds_dcps.h"
#include "check_status.h"

#include "ccpp_LargeMsg.h"

bool running = true;

static void catch_function(int signo) {
    running = false;
    std::cout << "Catching Ctrl-C, shutting down..." << std::endl;
}

int main(int argc, char *argv[])
{
    /* Register a signal handler so DDS doesn't just sit there... */
    if (signal(SIGINT, catch_function) == SIG_ERR)
    {
        fputs("An error occurred while setting a signal handler.\n", stderr);
        return EXIT_FAILURE;
    }
    DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;
    const char * partition_name = "Default";
    const char * topic_name = "big_chatter";

    /* Create Domain Participant Factory */
    DDS::DomainParticipantFactory_var dpf = DDS::DomainParticipantFactory::get_instance();
    checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");

    /* Create Domain Participant */
    DDS::DomainParticipant_var participant = dpf->create_participant(
        domain,
        PARTICIPANT_QOS_DEFAULT,
        NULL,
        DDS::STATUS_MASK_NONE
    );
    checkHandle(participant.in(), "DDS::DomainParticipantFactory::create_participant");

    DDS::ReturnCode_t status;

    /* Create a default QoS for Topics */
    DDS::TopicQos default_topic_qos;
    status = participant->get_default_topic_qos(default_topic_qos);
    checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
    // default_topic_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
    default_topic_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

    /* Register the LargeMessage Type */
    LargeMsg::LargeMessageTypeSupport_var large_message_ts = new LargeMsg::LargeMessageTypeSupport();
    checkHandle(large_message_ts.in(), "new LargeMessageTypeSupport");
    char * large_message_type_name = large_message_ts->get_type_name();
    status = large_message_ts->register_type(participant.in(), large_message_type_name);
    checkStatus(status, "LargeMsg::LargeMessageTypeSupport::register_type");

    /* Setup the Subscribers's QoS */
    DDS::SubscriberQos sub_qos;
    status = participant->get_default_subscriber_qos(sub_qos);
    checkStatus(status, "DDS::DomainParticipant::get_default_subscriber_qos");
    sub_qos.partition.name.length(1);
    sub_qos.partition.name[0] = partition_name;

    /* Create the subscriber */
    DDS::Subscriber_var subscriber = participant->create_subscriber(
        sub_qos,
        NULL,
        DDS::STATUS_MASK_NONE
    );
    checkHandle(subscriber.in(), "DDS::DomainParticipant::create_subscriber");

    /* Create the Topic */
    DDS::Topic_var large_message_topic = participant->create_topic(
        topic_name,
        large_message_type_name,
        default_topic_qos,
        NULL,
        DDS::STATUS_MASK_NONE
    );
    checkHandle(large_message_topic.in(), "DDS::DomainParticipant::create_topic(LargeMessage)");

    /* Create Topic specific DataReader */
    DDS::DataReader_var topic_reader = subscriber->create_datareader(
          large_message_topic.in(),
          DATAREADER_QOS_USE_TOPIC_QOS,
          NULL,
          DDS::STATUS_MASK_NONE
    );
    checkHandle(topic_reader.in(), "DDS::Subscriber::create_datareader");

    /* Narrow topic_reader down to LargeMessage specific DataReader */
    LargeMsg::LargeMessageDataReader_var data_reader = LargeMsg::LargeMessageDataReader::_narrow(topic_reader.in());
    checkHandle(data_reader.in(), "LargeMsg::LargeMessageDataReader::_narrow");

    LargeMsg::LargeMessageSeq_var large_msg_seq = new LargeMsg::LargeMessageSeq();
    DDS::SampleInfoSeq_var sample_info_seq = new DDS::SampleInfoSeq();

		struct timespec t;
		t.tv_sec = 0;
		t.tv_nsec = 10000000;

    std::cout << "Polling DataReader..." << std::endl;
    while (running)
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
            LargeMsg::LargeMessage *msg = &(large_msg_seq[i]);
            //std::cout << "[" << msg->seq << "]: " << strlen(msg->content.m_ptr) << std::endl;
        }

        status = data_reader->return_loan(large_msg_seq, sample_info_seq);
        checkStatus(status, "LargeMsg::LargeMessageDataReader::return_loan");

        /* Sleep for some amount of time, as not to consume too much CPU cycles. */
#if defined _WIN32
        Sleep(100);
#else
				clock_nanosleep(CLOCK_MONOTONIC, 0, &t, NULL);
#endif
      }

    /* Shutdown */
    {
        status = participant->delete_subscriber(subscriber.in());
        checkStatus(status, "DDS::DomainParticipant::delete_subscriber");

        status = participant->delete_topic(large_message_topic.in());
        checkStatus(status, "DDS::DomainParticipant::delete_topic (large_message_topic)");

        DDS::string_free(large_message_type_name);

        status = dpf->delete_participant(participant.in());
        checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");
    }
}
