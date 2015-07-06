#include <string>
#include <sys/mman.h>

#include <rttest/rttest.h>
#include "ExamplePublisher.hpp"

ExamplePublisher *pub;

void* pub_callback(void * unused)
{
	pub->callback();
}

int main(int argc, char *argv[])
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

  //getchar();

  pub = new ExamplePublisher;
  pub->message_size = message_length;

	pub->init();

	if (rttest_set_sched_priority(98, SCHED_RR) != 0)
  {
    std::cout << "Failed to set realtime priority of thread" << std::endl;
  }


	if (rttest_lock_and_prefault_dynamic(pool_size) != 0)
  {
    perror("Failed to lock dynamic memory");
  }

  //rttest_prefault_stack_size(stack_size);

  //clock_nanosleep(0, 0, &t, 0);

	rttest_spin(pub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	pub->teardown();
  delete pub;
}
