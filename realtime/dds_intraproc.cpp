#include <signal.h>
#include <pthread.h>
#include <memory>


#include <rttest/rttest.h>
#include "ExampleSubscriber.hpp"
#include "ExamplePublisher.hpp"

ExamplePublisher pub;
ExampleSubscriber sub;

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
	// put this in rttest
  /*if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + MAX_SAFE_STACK))
  {
    fprintf(stderr, "Couldn't set requested stack size for pthread");
    exit(1);
  }*/

  /* And finally start the actual thread */
  pthread_create(&thread, &attr, f, NULL);
}

void* pub_callback(void * unused)
{
	pub.callback();
}

void* sub_callback(void * unused)
{
	sub.callback();
}

void *publisher_thread(void *unused)
{
	pub.init();

	rttest_spin(pub_callback, NULL);

	rttest_write_results();
	pub.teardown();
}

void *subscriber_thread(void *unused)
{
	sub.init();

	rttest_init_new_thread();
	rttest_set_sched_priority(90, SCHED_RR);

	rttest_spin(sub_callback, NULL);

	rttest_write_results_file("rttest_subscriber_results");
	rttest_finish();

	sub.teardown();
}

int main(int argc, char *argv[])
{
	rttest_read_args(argc, argv);
  start_rt_thread(&subscriber_thread);
	publisher_thread(NULL);
	
	rttest_finish();
}
