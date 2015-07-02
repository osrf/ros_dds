#include <string>

#include <rttest/rttest.h>
#include "UnsafePublisher.hpp"

#define STACK_SIZE 1024*1024*1024

UnsafePublisher *pub;

void* pub_callback(void * unused)
{
	pub->callback();
}

int main(int argc, char *argv[])
{
  pub = new UnsafePublisher;
	pub->init();

	rttest_read_args(argc, argv);
	rttest_set_sched_priority(90, SCHED_RR);
	//rttest_lock_memory();
	//rttest_prefault_stack_size(STACK_SIZE);
	//rttest_lock_and_prefault_dynamic(STACK_SIZE);

	rttest_spin(pub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	pub->teardown();
  delete pub;
}
