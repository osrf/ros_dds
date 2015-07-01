#include <string>

#include <rttest/rttest.h>
#include "ExamplePublisher.hpp"

#define STACK_SIZE 1024*1024

ExamplePublisher pub;

void* pub_callback(void * unused)
{
	pub.callback();
}

int main(int argc, char *argv[])
{
	pub.init();

	rttest_read_args(argc, argv);
	rttest_set_sched_priority(90, SCHED_RR);
	rttest_lock_memory();
	rttest_prefault_stack_size(STACK_SIZE);

	rttest_spin(pub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	pub.teardown();
}
