#include <signal.h>
#include <sys/mman.h>

#include <rttest/rttest.h>
#include "ExampleSubscriber.hpp"

ExampleSubscriber *sub;

void* sub_callback(void * unused)
{
	sub->callback();
}

int main(int argc, char *argv[])
{
  sub = new ExampleSubscriber();
	sub->init();

	rttest_read_args(argc, argv);
	if (rttest_set_sched_priority(90, SCHED_RR) != 0)
  {
    perror("Failed to set scheduling priority and policy of thread");
  }

  size_t pool_size = 1024*1024*1024;
  size_t stack_size = sizeof(*sub) + 1024*1024;
	if (rttest_lock_memory() != 0)
  {
    perror("Failed to lock memory");
  }

	rttest_prefault_stack();

	rttest_spin(sub_callback, NULL);

	rttest_write_results();
	rttest_finish();

  std::cout << "Subscriber received " << sub->msgs_count << " messages." << std::endl;
	sub->teardown();
  delete sub;
}
