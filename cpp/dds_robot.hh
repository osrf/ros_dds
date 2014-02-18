/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef __NODE_HH_INCLUDED__
#define __NODE_HH_INCLUDED__

#include "ccpp_dds_dcps.h"
#include "ccpp_StringMsg.h"
#include "CheckStatus.h"
#include "os_stdlib.h"

using namespace DDS;
using namespace StringMsg;

#define TERMINATION_MESSAGE -1

//  ---------------------------------------------------------------------
class Node
{
  public:
    //  ---------------------------------------------------------------------
    /// \brief Constructor.
    /// \param[in] _master End point with the master's endpoint.
    /// \param[in] _verbose true for enabling verbose mode.
    Node(std::string _master, bool _verbose)
    {
      StringMessageTypeSupport_var stringMessageTS;
      ReturnCode_t status;

      this->master = _master;
      this->verbose = _verbose;

      if (this->verbose)
      {
        std::cout << "Master URI: [" << this->master << "]\n";
        std::cout << "Verbose mode: " << this->verbose << std::endl;
      }

      stringMessageTypeName = NULL;
      partitionName = "Default";
      this->domain = DOMAIN_ID_DEFAULT;
      this->id = 0;

      // Create a DomainParticipantFactory and a DomainParticipant
      // (using Default QoS settings.
      this->dpf = DomainParticipantFactory::get_instance ();
      checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
      this->participant = this->dpf->create_participant(this->domain,
          PARTICIPANT_QOS_DEFAULT, NULL, STATUS_MASK_NONE);
      checkHandle(this->participant.in(),
          "DDS::DomainParticipantFactory::create_participant");

      // Set the ReliabilityQosPolicy to RELIABLE.
      status = participant->get_default_topic_qos(this->reliable_topic_qos);
      checkStatus(status, "DDS::DomainParticipant::get_default_topic_qos");
      this->reliable_topic_qos.reliability.kind = RELIABLE_RELIABILITY_QOS;

      // Register the required datatype for StringMessage.
      stringMessageTS = new StringMessageTypeSupport();
      checkHandle(stringMessageTS.in(), "new StringMessageTypeSupport");
      this->stringMessageTypeName = stringMessageTS->get_type_name();
      status = stringMessageTS->register_type(this->participant.in(),
          this->stringMessageTypeName);
      checkStatus(status, "StringMsg::StringMessageTypeSupport::register_type");

      // Adapt the default PublisherQos to write into the "default" Partition.
      status = this->participant->get_default_publisher_qos(this->pub_qos);
      checkStatus(status, "DDS::DomainParticipant::get_default_publisher_qos");
      this->pub_qos.partition.name.length(1);
      this->pub_qos.partition.name[0] = this->partitionName;

      // Adapt the default SubscriberQos to read from the "default" Partition.
      status = this->participant->get_default_subscriber_qos (this->sub_qos);
      checkStatus(status, "DDS::DomainParticipant::get_default_subscriber_qos");
      this->sub_qos.partition.name.length(1);
      this->sub_qos.partition.name[0] = partitionName;

      // Create a Publisher.
      this->publisher = this->participant->create_publisher(
          this->pub_qos, NULL, STATUS_MASK_NONE);
      checkHandle(publisher.in(), "DDS::DomainParticipant::create_publisher");

      // Create a Subscriber for the MessageBoard application.
      this->subscriber = this->participant->create_subscriber(this->sub_qos,
          NULL, STATUS_MASK_NONE);
      checkHandle(this->subscriber.in(),
          "DDS::DomainParticipant::create_subscriber");
    }

    //  ---------------------------------------------------------------------
    /// \brief Destructor.
    virtual ~Node()
    {
      ReturnCode_t status;

      // Remove the DataWriters
      status = this->publisher->delete_datawriter(this->talker.in() );
      checkStatus(status, "DDS::Publisher::delete_datawriter (talker)");

      // Remove the Publisher.
      status = this->participant->delete_publisher(this->publisher.in() );
      checkStatus(status, "DDS::DomainParticipant::delete_publisher");

      // Remove the Subscriber.
      status = this->participant->delete_subscriber(this->subscriber.in() );
      checkStatus(status, "DDS::DomainParticipant::delete_subscriber");

      // Remove the Topics.
      status = this->participant->delete_topic(this->stringMessageTopic.in() );
      checkStatus(status,
          "DDS::DomainParticipant::delete_topic (stringMessageTopic)");

      // Remove the type-names.
      DDS::string_free(this->stringMessageTypeName);

      // Remove the DomainParticipant.
      status = this->dpf->delete_participant(this->participant.in() );
      checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");
    }

    //  ---------------------------------------------------------------------
    /// \brief Advertise a new service.
    /// \param[in] _topic Topic to be advertised.
    /// \return 0 when success.
    int Advertise(const std::string &_topic)
    {
      assert(_topic != "");

      DataWriter_var parentWriter;


      //ToDo: create a topic list

      // Create the StringMessage topic
      this->stringMessageTopic = this->participant->create_topic(
        _topic.c_str(),
        this->stringMessageTypeName,
        this->reliable_topic_qos,
        NULL,
        STATUS_MASK_NONE);
      checkHandle(this->stringMessageTopic.in(),
         "DDS::DomainParticipant::create_topic (stringMessage)");

      // Create a DataWriter for the stringMessage topic
      // (using the appropriate QoS).
      parentWriter = publisher->create_datawriter(
          stringMessageTopic.in(),
          DATAWRITER_QOS_USE_TOPIC_QOS,
          NULL,
          STATUS_MASK_NONE);
      checkHandle(parentWriter.in(),
         "DDS::Publisher::create_datawriter (stringMessage)");

      // Narrow the abstract parent into its typed representative.
      this->talker = StringMessageDataWriter::_narrow(parentWriter.in());
      checkHandle(this->talker.in(),
          "StringMsg::StringMessageDataWriter::_narrow");

      return 0;
    }

    //  ---------------------------------------------------------------------
    /// \brief Unadvertise a new service.
    /// \param[in] _topic Topic to be unadvertised.
    /// \return 0 when success.
    int UnAdvertise(const std::string &_topic)
    {
      assert(_topic != "");

      return 0;
    }

    //  ---------------------------------------------------------------------
    /// \brief Publish data.
    /// \param[in] _topic Topic to be published.
    /// \param[in] _data Data to publish.
    /// \return 0 when success.
    int Publish(const std::string &_topic, const std::string &_data)
    {
      assert(_topic != "");

      InstanceHandle_t userHandle;
      ReturnCode_t status;

      // Initialize the string message on Heap.
      StringMessage *msg = new StringMessage();
      checkHandle(msg, "new StringMessage");

      msg->msgID = this->id++;
      msg->content = _data.c_str();

      // Register a string message for this user
      // (pre-allocating resources for it!!)
      userHandle = this->talker->register_instance(*msg);

      // Write a message using the pre-generated instance handle.
      status = talker->write(*msg, userHandle);
      checkStatus(status, "String::StringMessageDataWriter::write");

      delete msg;

      return 0;
    }

    //  ---------------------------------------------------------------------
    /// \brief Subscribe to a topic registering a callback.
    /// \param[in] _topic Topic to be subscribed.
    /// \param[in] _cb Pointer to the callback function.
    /// \return 0 when success.
    int Subscribe(const std::string &_topic,
      void(*_cb)(const std::string &, const std::string &))
    {
      assert(_topic != "");

      StringMessageDataReader_var reader;
      DataReader_var parentReader;
      bool terminated = false;
      StringMessageSeq_var msgSeq = new StringMessageSeq();
      SampleInfoSeq_var infoSeq = new SampleInfoSeq();
      ReturnCode_t status;

      // Create the StringMessage topic
      this->stringMessageTopic = this->participant->create_topic(
        _topic.c_str(),
        this->stringMessageTypeName,
        this->reliable_topic_qos,
        NULL,
        STATUS_MASK_NONE);
      checkHandle(this->stringMessageTopic.in(),
         "DDS::DomainParticipant::create_topic (StringMessage)");


      // Create a DataReader for the StringMessage Topic
      // (using the appropriate QoS).
      parentReader = subscriber->create_datareader(
          this->stringMessageTopic.in(),
          DATAREADER_QOS_USE_TOPIC_QOS,
          NULL,
          STATUS_MASK_NONE);
      checkHandle(parentReader.in(), "DDS::Subscriber::create_datareader");

      // Narrow the abstract parent into its typed representative.
      reader = StringMessageDataReader::_narrow(parentReader.in());
      checkHandle(reader.in(), "MsgString::StringMessageDataReader::_narrow");

      while (!terminated)
      {
        status = reader->take(
            msgSeq,
            infoSeq,
            LENGTH_UNLIMITED,
            ANY_SAMPLE_STATE,
            ANY_VIEW_STATE,
            ALIVE_INSTANCE_STATE);
        checkStatus(status, "MsgString::StringMessageDataReader::take");

        //std::cout << msgSeq->length() << std::endl;

        for (DDS::ULong i = 0; i < msgSeq->length(); i++) {
            StringMessage *msg = &(msgSeq[i]);
            if (msg->msgID == TERMINATION_MESSAGE) {
                cout << "Termination message received: exiting..." << endl;
                terminated = true;
            } else {
                cout << _topic << ": " << msg->content << endl;
            }
            fflush(stdout);
        }

        status = reader->return_loan(msgSeq, infoSeq);
        checkStatus(status, "MsgString::StringMessageDataReader::return_loan");

        /* Sleep for some amount of time, as not to consume too much CPU cycles. */
#ifdef USE_NANOSLEEP
        sleeptime.tv_sec = 0;
        sleeptime.tv_nsec = 100000000;
        nanosleep(&sleeptime, &remtime);
#elif defined _WIN32
        Sleep(100);
#else
        usleep(100000);
#endif
      }

      return 0;
    }

    //  ---------------------------------------------------------------------
    /// \brief Subscribe to a topic registering a callback.
    /// \param[in] _topic Topic to be unsubscribed.
    /// \return 0 when success.
    int UnSubscribe(const std::string &_topic)
    {
      return 0;
    }

  private:
    // Master address
    std::string master;

    // Verbose mode enabled if true
    int verbose;

    // Generic DDS entities
    DomainParticipantFactory_var dpf;
    DomainParticipant_var participant;

    // QosPolicy holders
    TopicQos reliable_topic_qos;

    // DDS Identifiers
    DomainId_t domain;

    Topic_var stringMessageTopic;
    Publisher_var publisher;
    Subscriber_var subscriber;
    TopicQos setting_topic_qos;
    DataWriterQos dw_qos;
    StringMessageDataWriter_var talker;
    char *stringMessageTypeName;
    PublisherQos pub_qos;
    SubscriberQos sub_qos;
    const char *partitionName;
    int id;
};

#endif
