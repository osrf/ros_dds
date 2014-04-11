#include "daemon.c"

int main (
  int argc,
  char *argv[])
{
  int rc;
  rc = create_participant();
  if (rc) exit(rc);
#if USE_CONNEXT
  // only necessary for Connext
  wait_for_historical_data();
#endif
  const int max_size = 1024 * 1024;
  char buffer[max_size];
  rc = get_topics(buffer, max_size);
  if (rc) exit(rc);
  printf("topics: %s\n", buffer);
  rc = delete_participant();
  if (rc) exit(rc);
  return 0;
}
