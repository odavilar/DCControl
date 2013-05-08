#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <native/event.h>

#define EVENT_INIT 0x0
#define EVENT_MODE EV_PRIO
#define EVENT_WAIT_MASK (0x1|0x2)
#define EVENT_SIGNAL_MASK2 (0x2)
#define EVENT_SIGNAL_MASK1 (0x1)

RT_TASK demo_task;
RT_TASK demo_task2;
RT_EVENT ev_desc;

int fin = 1;

void demo(void *arg)
{
	RTIME now, previous;

	rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	previous = rt_timer_read();
	int cont = 0;

	while(fin){
		rt_task_wait_period(NULL);
		now = rt_timer_read();

		printf("Time since last turn: %ld.%06ld ms\n",
				(long)(now - previous) / 1000000,
				(long)(now - previous) % 1000000);
		previous = now;
		if(cont == 5)
		{
			printf("CINCO \n");
			//rt_event_signal(&ev_desc, EVENT_SIGNAL_MASK1);
		}
		if(cont == 10)
		{
			printf("DIEZ \n");
			rt_event_signal(&ev_desc, EVENT_SIGNAL_MASK2);

		}
		cont++;
	}
}

void demo2(void *arg)
{
	unsigned long mask_ret;
	rt_event_wait(&ev_desc, EVENT_WAIT_MASK, &mask_ret, EV_ALL, TM_INFINITE);
	printf("yajuuua");
}

void catch_signal(int sig)
{
	fin = 0;
}

int main(int argc, char* argv[])
{
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	mlockall(MCL_CURRENT|MCL_FUTURE);

	rt_event_create(&ev_desc, "myevent", EVENT_INIT, EVENT_MODE);

	rt_task_create(&demo_task, "trivial", 0, 99, 0);
	rt_task_create(&demo_task2, "trivial2", 0, 99, 0);

	rt_task_start(&demo_task, &demo, NULL);
	rt_task_start(&demo_task2, &demo2, NULL);

	pause();

	rt_event_delete(&ev_desc);
	rt_task_delete(&demo_task);

	return 0;
}
