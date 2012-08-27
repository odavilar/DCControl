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

#define TRUE 1
#define FALSE 0
#define MY_SIG 40


RT_TASK demo_task;
RT_TASK MoveMotor;
static unsigned sig_counter;
int done = FALSE;
int fd;

void sig_handler(int sig)
{
	//	static unsigned sig_counter;
	//	printf("\rGot single %d for %u times \n", sig, ++sig_counter);
	++sig_counter;
}

void demo(void *arg)
{
	RTIME now, previous;
	unsigned sc_now = 0, sc_previous = 0;
	double distancia;
	int cont = 0;
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 1 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 80000);
	//rt_task_set_periodic(NULL, TM_NOW, 1000000000);
	previous = rt_timer_read();

	while (1){
		rt_task_wait_period(NULL);
		now = rt_timer_read();
		sc_now = sig_counter;

		/*
		 * NOTE: printf may have unexpected impact on the timing of
		 *       your program. It is used here in the critical loop
		 *       only for demonstration purposes.
		 */
		/*printf("Time since last turn: %ld.%06ld ms\n",
		  (long)(now - previous) / 1000000,
		  (long)(now - previous) % 1000000);*/
		distancia = ((float)sc_now - (float)sc_previous)/7.0/4096.0;
		cont++;
		printf("Vel mm/s: %f Contador: %d \n",distancia, sig_counter);
		/*if(cont >= 12499)
		{
			printf("Vel mm/s: %f \n",distancia);
			cont = 0;
		}*/
		previous = now;
		sc_previous = sc_now;
	}
}

void move(void *arg)
{
	RTIME now, previous;
	ixpio_reg_t reg;
	unsigned short int data = 0;
	int bit = 0;
	rt_task_set_periodic(NULL, TM_NOW, 142500);
	reg.id = IXPIO_P3;
	reg.value = data;
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		puts("Failure of configuring interrupt.");
	}

	while(getchar() != 10 );
	printf("listo el enter");
	while(!done)
	{
		rt_task_wait_period(NULL);
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
			close(fd);
			puts("Failure of configuring interrupt.");
		}
		//	printf("Envío pulsos.\n");
	}
	data = 0;
	reg.value = data;
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		puts("Failure of configuring interrupt.");
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
	int index;
	char *dev_file;
	ixpio_reg_t reg,reg1;
	ixpio_signal_t sig;
	static struct sigaction act, act_old;

	dev_file = "/dev/ixpio1";

	/* Abrir fd de la tarjeta */
	fd = open(dev_file, O_RDWR);
	if (fd < 0) {
		printf("Failure of open device file \"%s.\"\n", dev_file);
		return FAILURE;
	}

	/* Activar Handler de la señal */
	act.sa_handler = sig_handler;
	sigemptyset(&act.sa_mask);
	sigaddset(&act.sa_mask, MY_SIG);
	if (sigaction(MY_SIG, &act, &act_old)) {
		close(fd);
		puts("Failure of signal action.");
		return FAILURE;
	}

	/* Configuración del Puerto */
	reg.id = IXPIO_PCB;
	reg.value = 0x01; // Habilitar P3 como salida
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MY_SIG, &act_old, NULL);
		puts("Failure of configuring port.");
		return FAILURE;
	}

	/* Configuro interrupciones */
	reg.id = IXPIO_IMCR;
	reg.value = 0x02; //Habilito contador
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MY_SIG, &act_old, NULL);
		puts("Failure of configuring interrupt.");
		return FAILURE;
	}

	/* signal condiction */
	sig.sid = MY_SIG;
	sig.pid = getpid();
	sig.is = 0x02;   /* signal for the P2C0 channels */
	sig.edge = 0x02;  /* high level trigger */
	if (ioctl(fd, IXPIO_SET_SIG, &sig)) {
		close(fd);
		sigaction(MY_SIG, &act_old, NULL);
		puts("Failure of signal condiction.");
		return FAILURE;
	}


	/*
	 * Xenomai
	 */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_create(&demo_task, "trivial", 0, 99, T_JOINABLE);
	rt_task_create(&MoveMotor, "mmotor", 0, 99, T_JOINABLE);
	rt_task_start(&demo_task, &demo, NULL);
	rt_task_start(&MoveMotor, &move, NULL);
	rt_task_join(&demo_task);
	rt_task_join(&MoveMotor);
	//		pause();
	//printf("chale\n %d",shit);
	rt_task_delete(&demo_task);
	rt_task_delete(&MoveMotor);

	//	while(1);
	/*
	 * Ixpio
	 */
	sigaction(MY_SIG, &act_old, NULL);
	close(fd);
	return 0;
}
