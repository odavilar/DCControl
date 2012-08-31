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
#define MY_SIG 40

#define SIZE 30000

int periodo = 1000000;
RT_TASK demo_task;
RT_TASK MoveMotor;
static unsigned sig_counter;
int done = FALSE;
int fd;
static float set = 1;
static float dista = 3;
float pid_val;
RTIME dutyns = 500000;
double vel;
int duty_to_ns(float duty);
float pid(float sp, float pv);

void sig_handler(int sig)
{
	++sig_counter;
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

	if(sp<=.5){

		sp = - 52.984 * (sp*sp*sp*sp*sp) - 407.74 * (sp*sp*sp*sp) + 835.42 * (sp*sp*sp) - 570.08 * (sp*sp) + 166.49 * sp - 17.565;

	}else if(sp>.5 && sp<=1.65){
		sp = -1.6172 * (sp*sp*sp*sp*sp*sp) + 12.723 * (sp*sp*sp*sp*sp) - 40.424 * (sp*sp*sp*sp) + 66.449 * (sp*sp*sp) - 59.489 * (sp*sp) + 28.653 * sp - 5.1674;
	}else{

		sp =  1.4898 * (sp*sp*sp*sp) - 11.638 * (sp*sp*sp) + 32.881 * (sp*sp) - 38.84 * sp + 17.792;
	}
	Kp = 200;
	Kd = 0.000001;
	Ki = 15047.605397115176;

	err_old = err;
	err = sp - pv;

	P_err = err;
	I_err = I_err + err_old;
	if ( I_err > 100 || I_err < -100)
	{
		I_err = 0;
	}
	D_err = err - err_old;
	pid = (Kp * P_err) + (Kd * D_err) + (Ki * I_err);
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
	int i = 0;
	static unsigned datosvel[10] = {0,0,0,0,0,0,0,0,0,0};
	static float dis_old;
	static float dis_new;
	int cont3 = 0;
	int cont4 = 0;
	int z;
	rt_task_set_periodic(NULL, TM_NOW, 100000);
	dis_old = 0;
	dis_new = 0;
	while(!done){

		for(i=10;i>0;i--)
		{
			datosvel[i]=datosvel[i-1];
		}
		datosvel[0] = sig_counter;
		/*
		 * Guardar y empujar
		 */
		dis_new = datosvel[0] * 0.000139509 * 2;
		dis_old = datosvel[10] * 0.000139509 * 2;
		vel = (dis_new - dis_old) * 1000.0 / 1.0;
		if(cont3 > 9 )
		{
						pid_val = pid(set,vel);
			cont4++;
			cont3 = 0;
		}
		cont3++;
		dutyns = duty_to_ns(pid_val);
		//printf("Velocidad: %f  dis_new: %f pid_val: %f duty: %f \n", vel, dis_new,pid_val,dutyns);

		if(dis_new >= dista)
		{
			done = TRUE;
		}
		z = rt_task_wait_period(NULL);
		if ( z != 0 )
		{
			switch(z)
			{
				case -ETIMEDOUT:
					printf("\nVETIMEOUT\n");
					break;
				case -EINTR:
					printf("\nVEINTR\n");
					break;
				case -EPERM:
					printf("\nVEPERM\n");
					break;
				case -EWOULDBLOCK:
					printf("\nVEWOULDBLOCK\n");
					break;
				default:
					break;
			}
		}
	}
}

void move(void *arg)
{
	ixpio_reg_t reg;
	unsigned short int data = 0;
	int bit = 0;
	int f, z;
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
		f = rt_task_sleep(dutyns);
		if ( f != 0 )
		{
			printf("\nsleep ERROR\n %d", f);
		}
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
			close(fd);
			puts("Failure of configuring interrupt.");
		}
		z = rt_task_wait_period(NULL);
		if ( z != 0 )
		{
			switch(z)
			{
				case -ETIMEDOUT:
					printf("\nETIMEOUT\n");
					break;
				case -EINTR:
					printf("\nEINTR\n");
					break;
				case -EPERM:
					printf("\nEPERM\n");
					break;
				case -EWOULDBLOCK:
					printf("\nEWOULDBLOCK\n");
					break;
				default:
					break;
			}
		}
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
	if (argc != 3)
	{
		return(1);
	}
	set = atof(argv[1]);
	dista = atof(argv[2]);
	printf("%f %f",set, dista);


	/*
	 * Xenomai
	 */
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/*
	 * Ixpio
	 */
	char *dev_file;
	ixpio_reg_t reg;
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
	rt_task_create(&demo_task, "trivial", 0, 90, T_JOINABLE );
	rt_task_create(&MoveMotor, "mmotor", 0, 99, T_JOINABLE );
	rt_task_start(&demo_task, &demo, NULL);
	rt_task_start(&MoveMotor, &move, NULL);
	rt_task_join(&demo_task);
	//	rt_task_join(&MoveMotor);
	rt_task_delete(&demo_task);
	rt_task_delete(&MoveMotor);

	/*
	 * Ixpio
	 */
	sigaction(MY_SIG, &act_old, NULL);
	close(fd);
	return 0;
}
