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
    // DDS::InstanceHandle_t instance_handle;
    DDS::DomainId_t domain;
    const char * partition_name;
    const char * topic_name;
    DDS::TopicQos default_topic_qos;
    LargeMsg::LargeMessageTypeSupport_var large_message_ts;
    DDS::PublisherQos pub_qos;
    DDS::DataWriter_var topic_writer;
    DDS::InstanceHandle_t instance_handle;

		int i;

	public:
    ExamplePublisher(unsigned int message_size = 16777216);
    unsigned int message_size;
		void callback();
		bool init();
		bool teardown();
};

ExamplePublisher::ExamplePublisher(unsigned int message_size) : message_size(message_size)
{
}

bool ExamplePublisher::init()
{
	this->i = 0;
	domain = DDS::DOMAIN_ID_DEFAULT;
	partition_name = "Default";
	topic_name = "big_chatter";

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
	status = participant->get_default_topic_qos(default_topic_qos);
	checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
	// default_topic_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
	default_topic_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;

	/* Register the LargeMessage Type */
	this->large_message_ts = new LargeMsg::LargeMessageTypeSupport();
	checkHandle(large_message_ts.in(), "new LargeMessageTypeSupport");
	this->large_message_type_name = large_message_ts->get_type_name();
	status = large_message_ts->register_type(participant.in(), large_message_type_name);
	checkStatus(status, "LargeMsg::LargeMessageTypeSupport::register_type");

	/* Setup the Publisher's QoS */
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
	topic_writer = publisher->create_datawriter(
				large_message_topic.in(),
				DATAWRITER_QOS_USE_TOPIC_QOS,
				NULL,
				DDS::STATUS_MASK_NONE
	);
	checkHandle(topic_writer.in(), "DDS::Publisher::create_datawriter(LargeMessage)");

	/* Narrow the Topic DataWriter to be only for LargeMessage */
	this->data_writer = LargeMsg::LargeMessageDataWriter::_narrow(topic_writer.in());
	checkHandle(data_writer.in(), "LargeMsg::LargeMessageDataWriter::_narrow");

	this->msg.content = std::string(this->message_size, '.').c_str();
	checkHandle(&msg, "new LargeMsg::LargeMessage");

	std::cout << "Ready to send LargeMessage's" << std::endl;
	return true;
}

void ExamplePublisher::callback()
{
	msg.seq = i;
	++i;
	instance_handle = data_writer->register_instance(msg);

	status = data_writer->write(msg, instance_handle);

	checkStatus(status, "LargeMsg::LargeMessageDataWriter::write");
}


bool ExamplePublisher::teardown()
{
	std::cout << "Finished" << std::endl;
	//getchar();
	/* Shutdown */
	{
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
