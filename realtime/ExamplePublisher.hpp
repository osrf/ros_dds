#include "ccpp_dds_dcps.h"
#include "check_status.h"

#include "ccpp_LargeMsg.h"

class ExamplePublisher
{
	private:
		LargeMsg::LargeMessageDataWriter_var data_writer;
		DDS::Publisher_var publisher;
		DDS::DomainParticipantFactory_var dpf;
		DDS::DomainParticipant_var participant;
		DDS::Topic_var large_message_topic;
		char * large_message_type_name;
		LargeMsg::LargeMessage msg;
		DDS::ReturnCode_t status;
		int i;

	public:
		void callback();
		bool init();
		bool teardown();
};

bool ExamplePublisher::init()
{
	this->i = 0;
	DDS::DomainId_t domain = DDS::DOMAIN_ID_DEFAULT;
	const char * partition_name = "Default";
	const char * topic_name = "big_chatter";

	/* Create Domain Participant Factory */
	this->dpf = DDS::DomainParticipantFactory::get_instance();
	checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");

	/* Create Domain Participant */
	std::cout << "Creating domain participant in publisher" << std::endl;
	this->participant = dpf->create_participant(
			domain,
			PARTICIPANT_QOS_DEFAULT,
			NULL,
			DDS::STATUS_MASK_NONE
	);
	checkHandle(participant.in(), "DDS::DomainParticipantFactory::create_participant");


	/* Create a default QoS for Topics */
	DDS::TopicQos default_topic_qos;
	status = participant->get_default_topic_qos(default_topic_qos);
	checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
	// default_topic_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
	default_topic_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

	/* Register the LargeMessage Type */
	LargeMsg::LargeMessageTypeSupport_var large_message_ts = new LargeMsg::LargeMessageTypeSupport();
	checkHandle(large_message_ts.in(), "new LargeMessageTypeSupport");
	this->large_message_type_name = large_message_ts->get_type_name();
	status = large_message_ts->register_type(participant.in(), large_message_type_name);
	checkStatus(status, "LargeMsg::LargeMessageTypeSupport::register_type");

	/* Setup the Publisher's QoS */
	DDS::PublisherQos pub_qos;
	status = participant->get_default_publisher_qos(pub_qos);
	checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
	pub_qos.partition.name.length(1);
	pub_qos.partition.name[0] = partition_name;

	/* Create the publisher */
	this->publisher = participant->create_publisher(pub_qos, NULL, DDS::STATUS_MASK_NONE);
	checkHandle(publisher.in(), "DDS::DomainParticipant::create_publisher");

	/* Create the Topic */
	this->large_message_topic = participant->create_topic(
			topic_name,
			large_message_type_name,
			default_topic_qos,
			NULL,
			DDS::STATUS_MASK_NONE
	);
	checkHandle(large_message_topic.in(), "DDS::DomainParticipant::create_topic(LargeMessage)");

	/* Create Topic DataWriter */
	DDS::DataWriter_var topic_writer = publisher->create_datawriter(
				large_message_topic.in(),
				DATAWRITER_QOS_USE_TOPIC_QOS,
				NULL,
				DDS::STATUS_MASK_NONE
	);
	checkHandle(topic_writer.in(), "DDS::Publisher::create_datawriter(LargeMessage)");

	/* Narrow the Topic DataWriter to be only for LargeMessage */
	this->data_writer = LargeMsg::LargeMessageDataWriter::_narrow(topic_writer.in());
	checkHandle(data_writer.in(), "LargeMsg::LargeMessageDataWriter::_narrow");

	/* Send some large messages */
	struct timespec t;
	t.tv_sec = 0;
	t.tv_nsec = 10000000;


	this->msg.content = std::string(pow(2, 24), '.').c_str();

	std::cout << "Ready to send LargeMessage's" << std::endl;
	return true;
}

void ExamplePublisher::callback()
{
	checkHandle(&msg, "new LargeMsg::LargeMessage");

	msg.seq = i;
	i++;

	DDS::InstanceHandle_t instance_handle = data_writer->register_instance(msg);
	checkStatus(status, "LargeMsg::LargeMessageDataWriter::write");
}


bool ExamplePublisher::teardown()
{
	std::cout << "Finished" << std::endl;
	getchar();
	/* Shutdown */
	{
			DDS::string_free(msg.content);

			status = publisher->delete_datawriter(data_writer.in());
			checkStatus(status, "DDS::Publisher::delete_datawriter(data_writer)");

			status = participant->delete_publisher(publisher.in());
			checkStatus(status, "DDS::DomainParticipant::delete_publisher");

			status = participant->delete_topic(large_message_topic.in());
			checkStatus(status, "DDS::DomainParticipant::delete_topic (large_message_topic)");

			DDS::string_free(large_message_type_name);

			status = dpf->delete_participant(participant.in());
			checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");
	}
	return true;
}
