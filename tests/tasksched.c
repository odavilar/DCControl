/*
 * La intención de este programa es poder hacer una pieza utilizando el tiempo real.
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <native/task.h>
#include <native/timer.h>
#include <sys/ioctl.h>
#include <signal.h>

#define TRUE 1
#define FALSE 0

void task1(void *arg)
{
}

void task2(void *arg)
{
}

void task3(void *arg)
{
}

void task4(void *arg)
{
}

void catch_signal(int sig)
{
	if( sig == SIGTERM)
	{
		printf("TERM\n");
	}else if( sig == SIGINT)
	{
		printf("INT\n");
	}
}

int main(int argc, char* argv[])
{
	RT_TASK Task1;
	RT_TASK Task2;
	RT_TASK Task3;
	RT_TASK Task4;

	/* Xenomai */
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Xenomai */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	rt_task_create(&Task1, "task1", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_start(&Task1, &task1, (void*)0 );
	rt_task_join(&Task1);
	rt_task_delete(&Task1);

	return 0;
}
