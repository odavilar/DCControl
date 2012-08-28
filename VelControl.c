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

#define SIZE 30000

int periodo = 285000;
RT_TASK demo_task;
RT_TASK MoveMotor;
static unsigned sig_counter;
int done = FALSE;
int fd;
float val[SIZE];
RTIME dutyns = 142500;
static struct timespec told, tnew;
double vel;
double datos[SIZE][3];
int duty_to_ns(float duty);
float pid(float sp, float pv);

void sig_handler(int sig)
{	/*
	   double pid_val;
	   */
	++sig_counter;
	/*
	 * clock_gettime(CLOCK_REALTIME,&tnew);

	 if(tnew.tv_nsec > told.tv_nsec){
	 vel = 0.000139509/(((float)(tnew.tv_nsec - told.tv_nsec))/1000000000.0);
	//if(vel > 0 && vel < (3 * 1.1)){
	pid_val = pid(1,vel);
	dutyns = duty_to_ns(pid_val);
	//}
	printf("Tiempo: %4.10f Velocidad: %4.10f PID: %4.5f\n",((float)(tnew.tv_nsec - told.tv_nsec))/1000000000.0, vel, pid_val);
	}
	told = tnew;*/
}

int duty_to_ns(float duty)
{
	return (float)duty * (float)periodo / 100.0;
}

float pid(float sp, float pv)
{
	static float err_old;
	static float P_err, I_err, D_err;
	static float err;
	float Kp, Kd, Ki;
	float pid;

	Kp = 104.805378005201;
	/*Kd = 22.181268156919350;
	Ki = 30347549.92739094;*/

	err_old = err;
	err = sp - pv;

	P_err = err;
	I_err = I_err + err_old;
/*	if ( I_err > 9000000000000000000) //Improvements
	{
		I_err = 0;
	}*/
	D_err = err - err_old;
	//printf(" %f %f %f",P_err, I_err, D_err);

	pid = (Kp * P_err) /*+ (Kd * D_err) + (Ki * I_err)*/;
	if ( pid > 100 )
	{
		pid = 100;
	}
	if ( pid < 1)
	{
		pid = 0.001;
	}

	return pid;
}

void demo(void *arg)
{
	static float dis_old;
	static float dis_new;
	float pid_val;
	rt_task_set_periodic(NULL, TM_NOW, 15000000);
	dis_old = 0;
	dis_new = 0;
	while(!done){
		dis_old = dis_new;
		dis_new = sig_counter * 2 * 2 / 4096.0 / 7.0;
		vel = (dis_new - dis_old) * 1000.0 / 15.0;
		pid_val = pid(1,vel);
		dutyns = duty_to_ns(pid_val);
		printf("Velocidad: %f \n ", vel);

		if(dis_new >= 4 )
		{
			done = TRUE;
		}
		rt_task_wait_period(NULL);
	}
}

void move(void *arg)
{
	RTIME now, previous;
	ixpio_reg_t reg;
	unsigned short int data = 0;
	int bit = 0;
	rt_task_set_periodic(NULL, TM_NOW, periodo);
	reg.id = IXPIO_P3;
	reg.value = data;
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		puts("Failure of configuring interrupt.");
	}

	while(!done)
	{
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
			close(fd);
			puts("Failure of configuring interrupt.");
		}
		rt_task_sleep(rt_timer_ns2ticks(dutyns));
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
			close(fd);
			puts("Failure of configuring interrupt.");
		}
		rt_task_wait_period(NULL);
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
	rt_task_create(&demo_task, "trivial", 0, 99, T_JOINABLE | T_FPU );
	rt_task_create(&MoveMotor, "mmotor", 0, 99, T_JOINABLE | T_FPU );
	rt_task_start(&demo_task, &demo, NULL);
	rt_task_start(&MoveMotor, &move, NULL);
	rt_task_join(&demo_task);
	rt_task_join(&MoveMotor);
	rt_task_delete(&demo_task);
	rt_task_delete(&MoveMotor);

	/*
	 * Ixpio
	 */
	sigaction(MY_SIG, &act_old, NULL);
	close(fd);
	printf("%u",sig_counter);
	return 0;
}
