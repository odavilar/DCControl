/*
 * System Identification
 * Captura en un archivo los datos adquiridos por un motor.
 * Utilizado para la identificación de la planta.
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
#include "ixpio.h"

#define TRUE 1
#define FALSE 0
#define MotorXSignal 40

unsigned counterX;
RT_TASK demo_task;
FILE *file; 
int salir;

void sig_handler(int sig)
{
	++counterX;
}

void demo(void *arg)
{
	RTIME now, previous;

	int z = 0;
	previous = rt_timer_read();
	do{
		rt_task_set_periodic(NULL, TM_NOW, 30000);
		fprintf(file,"%lu %d\n",rt_timer_read(), counterX); /*writes*/ 
/*		z = rt_task_wait_period(NULL);
		if ( z != 0 )
		{
			switch(z)
			{
				case -ETIMEDOUT:
					printf("VETIMEOUT\n");
					break;
				case -EINTR:
					printf("VEINTR\n");
					break;
				case -EPERM:
					printf("VEPERM\n");
					break;
				case -EWOULDBLOCK:
					printf("VEWOULDBLOCK\n");
					break;
				default:
					break;
				}
		}*/
	}while(salir == 0);
}

void catch_signal(int sig)
{
}

int main(int argc, char* argv[])
{
	salir = 0;

	/* Xenomai */
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Ixpio */
	int fd;
	char *dev_file;
	ixpio_reg_t reg;
	ixpio_signal_t sig;
	static struct sigaction act, act_old;

	dev_file = "/dev/ixpio1";

	/* Open board */
	fd = open(dev_file, O_RDWR);
	if (fd < 0) {
		printf("Failure of open device file \"%s.\"\n", dev_file);
		return FAILURE;
	}

	file = fopen("file.txt","a+"); 

	/* Enable signal Handler */
	act.sa_handler = sig_handler;
	sigemptyset(&act.sa_mask);
	sigaddset(&act.sa_mask, MotorXSignal);
	if (sigaction(MotorXSignal, &act, &act_old)) {
		close(fd);
		puts("Failure of signal action.");
		return FAILURE;
	}

	/* Port Configuration */
	reg.id = IXPIO_PCB;
	reg.value = 0x01; // Enable P3 as Output
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		puts("Failure of configuring port.");
		return FAILURE;
	}

	reg.id = IXPIO_PCC;
	reg.value = 0x01; // Enable P6 as Output
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		puts("Failure of configuring port.");
		return FAILURE;
	}

	/* Interrupts Configuration */
	reg.id = IXPIO_IMCR;
	reg.value = 0x06; // Enable P5C0 and P8C0 Interrupts
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		puts("Failure of configuring interrupt.");
		return FAILURE;
	}

	/* Signal Condiction */
	sig.sid = MotorXSignal;
	sig.pid = getpid();
	sig.is = 0x02;   /* Signal for the P5C0 channel */
	sig.edge = 0x02;  /* High level trigger */
	sig.bedge = 0x02;  /* High level trigger */
	if (ioctl(fd, IXPIO_SET_SIG, &sig)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		puts("Failure of signal condiction.");
		return FAILURE;
	}

	/*
	 * Xenomai
	 */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_create(&demo_task, "trivial", 0, 90, T_JOINABLE );
	rt_task_start(&demo_task, &demo, NULL);
	getchar();
	printf("corriendo\n");
	getchar();
	rt_task_join(&demo_task);
	salir = 1;
	rt_task_delete(&demo_task);



	fclose(file); /*done!*/ 
	/* Ixpio */
	close(fd);
	sigaction(MotorXSignal, &act_old, NULL);

	return 0;
}
