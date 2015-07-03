#include <signal.h>

#include <rttest/rttest.h>
#include "ExampleSubscriber.hpp"

ExampleSubscriber sub;

void* sub_callback(void * unused)
{
	sub.callback();
}

int main(int argc, char *argv[])
{
	sub.init();

	rttest_read_args(argc, argv);
	rttest_set_sched_priority(90, SCHED_RR);

  size_t pool_size = 1024*1024*1024;
  size_t stack_size = sizeof(sub) + 1024*1024;
	rttest_lock_and_prefault_dynamic(pool_size);
	rttest_prefault_stack_size(stack_size);

	rttest_spin(sub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	sub.teardown();
}
