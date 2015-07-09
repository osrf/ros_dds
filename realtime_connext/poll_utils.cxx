#ifndef EXAMPLE_POLL_UTILS_H
#define EXAMPLE_POLL_UTILS_H

#include <string>
#include <stdio.h>
#include <stdlib.h>

#ifdef RTI_VX653
#include <vThreadsData.h>
#endif
#include "poll.h"
#include "pollSupport.h"
#include "ndds/ndds_cpp.h"

#include <rttest/rttest.h>

unsigned int samples_received = 0;

struct PublisherNode
{
    pollDataWriter * poll_writer;
    poll *instance;
    DDS_InstanceHandle_t instance_handle;
    int count;
    unsigned int message_length;
};

struct Arguments
{
  int argc;
  char **argv;
};

/* Delete all entities */
static int subscriber_shutdown(
    DDSDomainParticipant *participant)
{
    printf("Samples received by subscriber: %d\n", samples_received);
    DDS_ReturnCode_t retcode;
    int status = 0;

    if (participant != NULL) {
        retcode = participant->delete_contained_entities();
        if (retcode != DDS_RETCODE_OK) {
            printf("delete_contained_entities error %d\n", retcode);
            status = -1;
        }

        retcode = DDSTheParticipantFactory->delete_participant(participant);
        if (retcode != DDS_RETCODE_OK) {
            printf("delete_participant error %d\n", retcode);
            status = -1;
        }
    }

    /* RTI Connext provides the finalize_instance() method on
       domain participant factory for people who want to release memory used
       by the participant factory. Uncomment the following block of code for
       clean destruction of the singleton. */
    retcode = DDSDomainParticipantFactory::finalize_instance();
    if (retcode != DDS_RETCODE_OK) {
        printf("finalize_instance error %d\n", retcode);
        //status = -1;
    }
    return status;
}

void *subscriber_callback(void *args)
{
    pollDataReader *poll_reader = static_cast<pollDataReader*>(args);
    DDS_SampleInfoSeq info_seq;
    pollSeq data_seq;

    /* Check for new data calling the DataReader's take() method */
    DDS_ReturnCode_t retcode = poll_reader->take(
        data_seq, info_seq, DDS_LENGTH_UNLIMITED,
        DDS_ANY_SAMPLE_STATE, DDS_ANY_VIEW_STATE, DDS_ANY_INSTANCE_STATE);
    if (retcode == DDS_RETCODE_NO_DATA) {
        /// Not an error
        return NULL;
    } else if (retcode != DDS_RETCODE_OK) {
        // Is an error
        printf("take error: %d\n", retcode);
        exit(-1);
    }

    int len = 0;
    double sum = 0;

    /* Iterate through the samples read using the take() method, getting
     * the number of samples got and, adding the value of x on each of
     * them to calculate the average afterwards. */
    for (int i = 0; i < data_seq.length(); ++i) {
        if (!info_seq[i].valid_data)
            continue;
        len ++;
        sum += data_seq[i].seq;
        ++samples_received;
    }

    /*
    if (len > 0)
        printf("Got %d samples.  Avg = %.1f\n", len, sum/len);
    */

    retcode = poll_reader->return_loan(data_seq, info_seq);
    if (retcode != DDS_RETCODE_OK) {
        printf("return loan error %d\n", retcode);
    }

}

int subscriber_main(int argc, char *argv[])
{
    rttest_read_args(argc, argv);

    int domainId = 0;
    DDSDomainParticipant *participant = NULL;
    DDSSubscriber *subscriber = NULL;
    DDSTopic *topic = NULL;
    DDSDataReader *reader = NULL;
    DDS_ReturnCode_t retcode;
    const char *type_name = NULL;
    int count = 0;
    /* Poll for new samples every 5 seconds */
    DDS_Duration_t receive_period = {5,0};
    int status = 0;

    /* To customize the participant QoS, use 
       the configuration file USER_QOS_PROFILES.xml */
    participant = DDSTheParticipantFactory->create_participant(
        domainId, DDS_PARTICIPANT_QOS_DEFAULT,
        NULL /* listener */, DDS_STATUS_MASK_NONE);
    if (participant == NULL) {
        printf("create_participant error\n");
        subscriber_shutdown(participant);
        return -1;
    }

    /* To customize the subscriber QoS, use 
       the configuration file USER_QOS_PROFILES.xml */
    subscriber = participant->create_subscriber(
        DDS_SUBSCRIBER_QOS_DEFAULT, NULL /* listener */, DDS_STATUS_MASK_NONE);
    if (subscriber == NULL) {
        printf("create_subscriber error\n");
        subscriber_shutdown(participant);
        return -1;
    }

    /* Register the type before creating the topic */
    type_name = pollTypeSupport::get_type_name();
    retcode = pollTypeSupport::register_type(
        participant, type_name);
    if (retcode != DDS_RETCODE_OK) {
        printf("register_type error %d\n", retcode);
        subscriber_shutdown(participant);
        return -1;
    }

    /* To customize the topic QoS, use 
       the configuration file USER_QOS_PROFILES.xml */
    topic = participant->create_topic(
        "Example poll",
        type_name, DDS_TOPIC_QOS_DEFAULT, NULL /* listener */,
        DDS_STATUS_MASK_NONE);
    if (topic == NULL) {
        printf("create_topic error\n");
        subscriber_shutdown(participant);
        return -1;
    }

    /* Call create_datareader passing NULL in the listener parameter */
    reader = subscriber->create_datareader(
        topic, DDS_DATAREADER_QOS_DEFAULT, NULL,
        DDS_STATUS_MASK_ALL);
    if (reader == NULL) {
        printf("create_datareader error\n");
        subscriber_shutdown(participant);
        return -1;
    }

    /* If you want to change datareader_qos.history.kind programmatically rather
     * than using the XML file, you will need to add the following lines to your
     * code and comment out the create_datareader call above. */

    /*
    DDS_DataReaderQos datareader_qos;
    retcode = subscriber->get_default_datareader_qos(datareader_qos);
    if (retcode != DDS_RETCODE_OK) {
        printf("get_default_datareader_qos error\n");
        return -1;
    }

    datareader_qos.history.kind = DDS_KEEP_ALL_HISTORY_QOS;

    reader = subscriber->create_datareader(
        topic, datareader_qos, NULL,
        DDS_STATUS_MASK_ALL);
    if (reader == NULL) {
        printf("create_datareader error\n");
        subscriber_shutdown(participant);
        return -1;
    }
    */

    pollDataReader *poll_reader = pollDataReader::narrow(reader);
    if (poll_reader == NULL) {
        printf("DataReader narrow error\n");
        return -1;
    }

    if (rttest_set_sched_priority(97, SCHED_RR) != 0) {
      perror("Failed to set scheduling priority");
    }

    if (rttest_lock_memory() != 0) {
      perror("Couldn't lock memory");
    }
    rttest_prefault_stack();

    /* Main loop */
    rttest_spin(subscriber_callback, static_cast<void*>(poll_reader));

    /* Delete all entities */
    status = subscriber_shutdown(participant);

    rttest_write_results_file("rttest_subscriber_results");
    rttest_finish();

    return status;
}

int subscriber_main(Arguments *args)
{
    return subscriber_main(args->argc, args->argv);
}

void *subscriber_main(void *args)
{
    Arguments *new_args = static_cast<Arguments*>(args);
    int ret = subscriber_main(new_args);
    return static_cast<void*>(&ret);
}

/* Delete all entities */
static int publisher_shutdown(
    DDSDomainParticipant *participant)
{
    DDS_ReturnCode_t retcode;
    int status = 0;

    if (participant != NULL) {
        retcode = participant->delete_contained_entities();
        if (retcode != DDS_RETCODE_OK) {
            printf("delete_contained_entities error %d\n", retcode);
            status = -1;
        }

        retcode = DDSTheParticipantFactory->delete_participant(participant);
        if (retcode != DDS_RETCODE_OK) {
            printf("delete_participant error %d\n", retcode);
            status = -1;
        }
    }

    /* RTI Connext provides finalize_instance() method on
       domain participant factory for people who want to release memory used
       by the participant factory. Uncomment the following block of code for
       clean destruction of the singleton. */
    retcode = DDSDomainParticipantFactory::finalize_instance();
    if (retcode != DDS_RETCODE_OK) {
        printf("finalize_instance error %d\n", retcode);
        //status = -1;
    }

    return status;
}

void *publisher_callback(void *args)
{
    PublisherNode *pub_node = static_cast<PublisherNode*>(args);
    // message instance, instance_handle, poll_writer
    /* Set x to a random number between 0 and 9 */
    pub_node->instance->seq = (int)(rand()/(RAND_MAX/10.0));
    for (int j = 0; j < pub_node->message_length; j++) {
      pub_node->instance->content[j] = pub_node->count;
    }

    DDS_ReturnCode_t retcode = pub_node->poll_writer->write(*pub_node->instance, pub_node->instance_handle);
    if (retcode != DDS_RETCODE_OK) {
        printf("write error %d\n", retcode);
    }
    ++pub_node->count;
}

int publisher_main(int argc, char *argv[])
{
    unsigned int message_length = 1;
    {
      int c;
      // l stands for message length
      opterr = 0;
      optind = 1;
      int argc_copy = argc;
      char *argv_copy[argc];
      for (int i = 0; i < argc; ++i)
      {
        size_t len = strlen(argv[i]);
        argv_copy[i] = (char*) malloc(len);
        memcpy(argv_copy[i], argv[i], len);
      }

      while ((c = getopt(argc_copy, argv_copy, "l:")) != -1)
      {
        switch(c)
        {
          case 'l':
            message_length = std::stoul(std::string(optarg));
            break;
          default:
            break;
        }
      }
    }

    rttest_read_args(argc, argv);
    int domainId = 0;
    DDSDomainParticipant *participant = NULL;
    DDSPublisher *publisher = NULL;
    DDSTopic *topic = NULL;
    DDSDataWriter *writer = NULL;
    pollDataWriter * poll_writer = NULL;
    poll *instance = NULL;
    DDS_ReturnCode_t retcode;
    DDS_InstanceHandle_t instance_handle = DDS_HANDLE_NIL;
    const char *type_name = NULL;
    int count = 0;

    /* To customize participant QoS, use 
       the configuration file USER_QOS_PROFILES.xml */
    participant = DDSTheParticipantFactory->create_participant(
        domainId, DDS_PARTICIPANT_QOS_DEFAULT,
        NULL /* listener */, DDS_STATUS_MASK_NONE);
    if (participant == NULL) {
        printf("create_participant error\n");
        publisher_shutdown(participant);
        return -1;
    }

    /* To customize publisher QoS, use 
       the configuration file USER_QOS_PROFILES.xml */
    publisher = participant->create_publisher(
        DDS_PUBLISHER_QOS_DEFAULT, NULL /* listener */, DDS_STATUS_MASK_NONE);
    if (publisher == NULL) {
        printf("create_publisher error\n");
        publisher_shutdown(participant);
        return -1;
    }

    /* Register type before creating topic */
    type_name = pollTypeSupport::get_type_name();
    retcode = pollTypeSupport::register_type(
        participant, type_name);
    if (retcode != DDS_RETCODE_OK) {
        printf("register_type error %d\n", retcode);
        publisher_shutdown(participant);
        return -1;
    }

    /* To customize topic QoS, use 
       the configuration file USER_QOS_PROFILES.xml */
    topic = participant->create_topic(
        "Example poll",
        type_name, DDS_TOPIC_QOS_DEFAULT, NULL /* listener */,
        DDS_STATUS_MASK_NONE);
    if (topic == NULL) {
        printf("create_topic error\n");
        publisher_shutdown(participant);
        return -1;
    }

    /* To customize data writer QoS, use 
       the configuration file USER_QOS_PROFILES.xml */
    writer = publisher->create_datawriter(
        topic, DDS_DATAWRITER_QOS_DEFAULT, NULL /* listener */,
        DDS_STATUS_MASK_NONE);
    if (writer == NULL) {
        printf("create_datawriter error\n");
        publisher_shutdown(participant);
        return -1;
    }
    poll_writer = pollDataWriter::narrow(writer);
    if (poll_writer == NULL) {
        printf("DataWriter narrow error\n");
        publisher_shutdown(participant);
        return -1;
    }

    /* Create data sample for writing */

    instance = pollTypeSupport::create_data();

    if (instance == NULL) {
        printf("pollTypeSupport::create_data error\n");
        publisher_shutdown(participant);
        return -1;
    }
    strcpy(instance->content, std::string(message_length, '.').c_str());

    /* For a data type that has a key, if the same instance is going to be
       written multiple times, initialize the key here
       and register the keyed instance prior to writing */
    instance_handle = poll_writer->register_instance(*instance);

    PublisherNode pub_node;
    pub_node.instance = instance;
    pub_node.instance_handle = instance_handle;
    pub_node.poll_writer = poll_writer;
    pub_node.count = 0;
    pub_node.message_length = message_length;

    /* Initialize random seed before entering the loop */
    // srand(time(NULL));

    if (rttest_set_sched_priority(98, SCHED_RR) != 0) {
      perror("Failed to set scheduling priority");
    }

    if (rttest_lock_memory() != 0) {
      perror("Couldn't lock memory");
    }
    rttest_prefault_stack();

    rttest_spin(publisher_callback, static_cast<void*>(&pub_node));
    rttest_write_results_file("rttest_publisher_results");
    rttest_finish();

    retcode = poll_writer->unregister_instance(
        *instance, instance_handle);
    if (retcode != DDS_RETCODE_OK) {
        printf("unregister instance error %d\n", retcode);
    }

    /* Delete data sample */
    retcode = pollTypeSupport::delete_data(instance);
    if (retcode != DDS_RETCODE_OK) {
        printf("pollTypeSupport::delete_data error %d\n", retcode);
    }

    /* Delete all entities */
    return publisher_shutdown(participant);
}

int publisher_main(Arguments *args)
{
    return publisher_main(args->argc, args->argv);
}

void *publisher_main(void *args)
{
    Arguments *new_args = static_cast<Arguments*>(args);
    int ret = publisher_main(new_args);
    return static_cast<void*>(&ret);
}

#endif
