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
	pub.init();

	rttest_read_args(argc, argv);

	rttest_spin(pub_callback, NULL);

	rttest_write_results();
	rttest_finish();

	pub.teardown();
}
