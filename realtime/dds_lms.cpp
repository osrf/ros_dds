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

	rttest_spin(sub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	sub.teardown();
}
