#include <string>

#include <rttest/rttest.h>
#include "ExamplePublisher.hpp"

ExamplePublisher pub;

void* pub_callback(void * unused)
{
	pub.callback();
}

int main(int argc, char *argv[])
{
  int c;

  unsigned int message_length;
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
	rttest_read_args(argc, argv);

  pub.message_size = message_length;
	pub.init();

	rttest_set_sched_priority(90, SCHED_RR);
	rttest_lock_memory();
	rttest_prefault_stack_size(message_length + 4096);
	//rttest_lock_and_prefault_dynamic(STACK_SIZE);

	rttest_spin(pub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	pub.teardown();
}
