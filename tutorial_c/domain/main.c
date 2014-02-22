/* CreateTopics.c */
#include "dds_dcps.h"
#include "unistd.h"

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

  sleep(10);

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
