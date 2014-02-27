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

    while (!terminated) {
        DDS::InstanceHandleSeq participant_handles;

        status = participant->get_discovered_participants(participant_handles);

        std::cout << "Number of participants in the domain: " << participant_handles.length() << std::endl;

        usleep(100000);
    }

    status = dpf->delete_participant(participant.in());
    checkStatus(status, "DDS::DomainParticipantFactory::delete_participant");

    return 0;
}
