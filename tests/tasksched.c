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
	int cont = 0;
	rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	while(cont++ < 3)
	{
		rt_printf("Soy tarea 1 \n");
		rt_task_wait_period(NULL);
	}
}

void task2(void *arg)
{
	int cont = 0;
	rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	while(cont++ < 3)
	{
		rt_printf("Soy tarea 2 \n");
		rt_task_wait_period(NULL);
	}
}

void task3(void *arg)
{
	int cont = 0;
	rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	while(cont++ < 3)
	{
		rt_printf("Soy tarea 3 \n");
		rt_task_wait_period(NULL);
	}
}

void task4(void *arg)
{
	int cont = 0;
	rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	while(cont++ < 3)
	{
		rt_printf("Soy tarea 4 \n");
		rt_task_wait_period(NULL);
	}
}

void dispatcher(void *arg)
{
	RT_TASK Task1;
	RT_TASK Task2;
	RT_TASK Task3;
	RT_TASK Task4;
	rt_task_create(&Task1, "task1", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_create(&Task2, "task2", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_create(&Task3, "task3", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_create(&Task4, "task4", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_start(&Task1, &task1, (void*)0 );
	rt_task_start(&Task2, &task2, (void*)0 );
	rt_task_start(&Task3, &task3, (void*)0 );
	rt_task_start(&Task4, &task4, (void*)0 );
	rt_task_join(&Task1);
	rt_task_join(&Task2);
	rt_task_join(&Task3);
	rt_task_join(&Task4);
	rt_task_delete(&Task1);
	rt_task_delete(&Task2);
	rt_task_delete(&Task3);
	rt_task_delete(&Task4);

	rt_task_create(&Task1, "task1", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_create(&Task2, "task2", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_start(&Task1, &task1, (void*)0 );
	rt_task_start(&Task2, &task2, (void*)0 );
	rt_task_join(&Task1);
	rt_task_join(&Task2);

	rt_task_delete(&Task1);
	rt_task_delete(&Task2);
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
	rt_print_auto_init(1);

	RT_TASK Dispatcher;

	/* Xenomai */
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Xenomai */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	rt_task_create(&Dispatcher, "dispatcher", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_start(&Dispatcher, &dispatcher, (void*)0 );
	rt_task_join(&Dispatcher);
	rt_task_delete(&Dispatcher);

	return 0;
}
