#include <string>

#include <rttest/rttest.h>
#include "ExamplePublisher.hpp"

#define STACK_SIZE 1024*1024*1024

ExamplePublisher pub;

void* pub_callback(void * unused)
{
	pub.callback();
}

int main(int argc, char *argv[])
{
  unsigned int message_size = 1;
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
          message_size = std::stoul(std::string(optarg));
          break;
        default:
          break;
      }
    }
  }
  pub.message_size = message_size;
	pub.init();

	rttest_read_args(argc, argv);
	if (rttest_set_sched_priority(90, SCHED_RR) != 0)
  {
    perror("Failed to set scheduling priority and policy of thread");
  }

	if (rttest_lock_memory() != 0)
  {
    perror("Failed to lock memory");
  }
	//rttest_prefault_stack_size(STACK_SIZE);
	//rttest_lock_and_prefault_dynamic(STACK_SIZE);

	rttest_spin(pub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	pub.teardown();
}
