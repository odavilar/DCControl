#include <stdio.h>
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
#include "ixpio.h"

#define MY_SIG 40

RT_TASK demo_task;

void sig_handler(int sig)
{
	static unsigned sig_counter;
	printf("\rGot single %d for %u times \n", sig, ++sig_counter);
}

void demo(void *arg)
{
	RTIME now, previous;

	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 1 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	previous = rt_timer_read();

	while (1) {
		rt_task_wait_period(NULL);
		now = rt_timer_read();

		/*
		 * NOTE: printf may have unexpected impact on the timing of
		 *       your program. It is used here in the critical loop
		 *       only for demonstration purposes.
		 */
		printf("Time since last turn: %ld.%06ld ms\n",
				(long)(now - previous) / 1000000,
				(long)(now - previous) % 1000000);
		previous = now;
	}
}

void catch_signal(int sig)
{
}

int main(int argc, char* argv[])
{
	/*
	 * Xenomai
	 */
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/*
	 * Ixpio
	 */
	int fd,index;
	char *dev_file;
	ixpio_reg_t reg,reg1;
	ixpio_signal_t sig;
	static struct sigaction act, act_old;

	dev_file = "/dev/ixpio1";

	/* open device file */
	fd = open(dev_file, O_RDWR);
	if (fd < 0) {
		printf("Failure of open device file \"%s.\"\n", dev_file);
		return FAILURE;
	}

	/* set action for signal */
	act.sa_handler = sig_handler;
	sigemptyset(&act.sa_mask);
	sigaddset(&act.sa_mask, MY_SIG);
	if (sigaction(MY_SIG, &act, &act_old)) {
		close(fd);
		puts("Failure of signal action.");
		return FAILURE;
	}

	/* port configuration */
	reg.id = IXPIO_PCA;
	reg.value = 2;                  /* Port 1 as DO, Port 2 as DI */
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MY_SIG, &act_old, NULL);
		puts("Failure of configuring port.");
		return FAILURE;
	}

	/* configure board interrupt */
	reg.id = IXPIO_IMCR;

	/* enable INT_CHAN_0 for interrupt */
	reg.value = 0x01;

	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MY_SIG, &act_old, NULL);
		puts("Failure of configuring interrupt.");
		return FAILURE;
	}

	/* signal condiction */
	sig.sid = MY_SIG;
	sig.pid = getpid();
	sig.is = 0x01;   /* signal for the P2C0 channels */
	sig.edge = 0x01;  /* high level trigger */
	if (ioctl(fd, IXPIO_SET_SIG, &sig)) {
		close(fd);
		sigaction(MY_SIG, &act_old, NULL);
		puts("Failure of signal condiction.");
		return FAILURE;
	}

	sigaction(MY_SIG, &act_old, NULL);

	/*
	 * Xenomai
	 */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	rt_task_create(&demo_task, "trivial", 0, 99, 0);
	rt_task_start(&demo_task, &demo, NULL);
	rt_task_join(&demo_task);
	rt_task_delete(&demo_task);

	/*
	 * Ixpio
	 */
	close(fd);

	return 0;
}
