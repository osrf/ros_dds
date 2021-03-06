#include <rttest/rttest.h>

#include "ExampleSubscriber.hpp"
#include "ExamplePublisher.hpp"

ExamplePublisher *pub;
ExampleSubscriber *sub;
unsigned int message_length;

static void start_rt_thread(void *(*f)(void*))
{
  pthread_t thread;
  pthread_attr_t attr;

  /* init to default values */
  if (pthread_attr_init(&attr))
  {
    fprintf(stderr, "Couldn't initialize pthread to default value");
    exit(1);
  }

  /* Set the requested stacksize for this thread */
	// put this in rttest?
  if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + 2*message_length + sizeof(*sub)))
  {
    fprintf(stderr, "Couldn't set requested stack size for pthread");
    exit(1);
  }

  /* And finally start the actual thread */
  pthread_create(&thread, &attr, f, NULL);
}

void* pub_callback(void * unused)
{
  if (pub != NULL)
    pub->callback();
}

void* sub_callback(void * unused)
{
  if (sub != NULL)
    sub->callback();
}

void *publisher_thread(void *unused)
{
	rttest_spin(pub_callback, NULL);

	rttest_write_results_file("rttest_publisher_results");
  if (pub != NULL)
    pub->teardown();
	rttest_finish();
}

void *subscriber_thread(void *unused)
{
	rttest_init_new_thread();
	if (rttest_set_sched_priority(97, SCHED_RR) != 0)
  {
    perror("Failed to set scheduling priority and policy of thread");
  }


	rttest_spin(sub_callback, NULL);

	rttest_write_results_file("rttest_subscriber_results");
  if (sub != NULL)
    sub->teardown();
	rttest_finish();
}

int main(int argc, char *argv[])
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
	rttest_read_args(argc, argv);

  pub = new ExamplePublisher();
  pub->message_size = message_length;
	pub->init();
  sub = new ExampleSubscriber();
	sub->init();

	if (rttest_lock_memory() != 0)
  {
    perror("Failed to lock dynamic memory. Process might not be real-time safe");
  }
	rttest_prefault_stack();

  start_rt_thread(&subscriber_thread);

	if (rttest_set_sched_priority(98, SCHED_RR) != 0)
  {
    perror("Failed to set scheduling priority and policy of thread");
  }
	publisher_thread(NULL);
}
