#include <iostream>
#include <string.h>

#include "CheckStatus.h"

#ifndef _WIN32
#include <unistd.h>
#else
#include <Windows.h>
#endif

#include "example_main.h"

using namespace DDS;

#define TERMINATION_MESSAGE -1

int OSPL_MAIN (int argc, char *argv[]) {
    DomainParticipantFactory_var dpf;
    DomainParticipant_var participant;

    DomainId_t domain = DOMAIN_ID_DEFAULT;
    ReturnCode_t status;

    bool terminated = false;

    dpf = DomainParticipantFactory::get_instance();
    checkHandle(dpf.in(), "DDS::DomainParticipantFactory::get_instance");
    participant = dpf->create_participant(domain, PARTICIPANT_QOS_DEFAULT, NULL,
        STATUS_MASK_NONE);
    checkHandle(participant.in(), "DDS::DomainParticipantFactory::create_participant");

    Subscriber_var builtinSubscriber = participant->get_builtin_subscriber();
    checkHandle(builtinSubscriber, "DDS::DomainParticipant::get_builtin_subscriber");
 
    ParticipantBuiltinTopicDataDataReader_var participantsDR = ParticipantBuiltinTopicDataDataReader::_narrow(
        builtinSubscriber->lookup_datareader("DCPSParticipant"));
    checkHandle(participantsDR.in(), "DCPSParticipant");

    while(!terminated) {
        ParticipantBuiltinTopicDataSeq_var participantDataSeq = new ParticipantBuiltinTopicDataSeq();
        SampleInfoSeq_var infoSeq = new SampleInfoSeq();
        participantsDR->take(participantDataSeq, infoSeq, LENGTH_UNLIMITED,
            ANY_SAMPLE_STATE, ANY_VIEW_STATE, ANY_INSTANCE_STATE); //ALIVE_INSTANCE_STATE);

        for(int i = 0; i < infoSeq->length(); ++i) {
            ParticipantBuiltinTopicData_var participantData = participantDataSeq[i];
            SampleInfo_var info = infoSeq[i];

            String hostname;

            switch(info->instance_state) {
                case ALIVE_INSTANCE_STATE:
                    std::cout << "New participant joined" << std::endl;
                    std::cout << "NodeID: " << participantData->key[0] << std::endl;
                    // I still don't know what these keys are for
                    std::cout << "Key[1]: " << participantData->key[1] << std::endl;
                    std::cout << "Key[2]: " << participantData->key[2] << std::endl;
                    // Hostname is stored in the user_data field, maybe we can use this
                    // to distinguish between participants that represent the processes
                    // and the built-in participants. The second key can also be used to
                    // identify participant processes (value 0x30, don't know what it
                    // means but it's always 0x30 when there's data in user_data)
                    if(participantData->user_data.value.length() > 0) {
                        hostname = string_alloc(participantData->user_data.value.length());
                        memcpy(hostname, participantData->user_data.value.get_buffer(),
                            participantData->user_data.value.length());
                        hostname[participantData->user_data.value.length()] = '\0';
                        std::cout << "Host name: " << hostname << std::endl;
                    }
                    break;
                case NOT_ALIVE_DISPOSED_INSTANCE_STATE:
                    std::cout << "Participant deleted" << std::endl;
                    std::cout << "NodeID: " << participantData->key[0] << std::endl;
                    break;
                case NOT_ALIVE_NO_WRITERS_INSTANCE_STATE:
                    std::cout << "Participant unreachable" << std::endl;
                    std::cout << "NodeID: " << participantData->key[0] << std::endl;
                    break;
                default:
                    std::cout << "Unknown instance state: " << info->instance_state << std::endl;
            }
        }
        usleep(100000);
    }

    status = dpf->delete_participant(participant.in());
    checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");

    return 0;
}
