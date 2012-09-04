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
#define MotorZSignal 41

unsigned counterX, counterZ;

typedef struct motor{
	int * fd;
	int periodo;
	int done;
	float set;
	float distance;
	float vel;
	float pid_val;
	RTIME dutyns;
}Motor;

int duty_to_ns(float duty, float periodo);
float pid(float sp, float pv);

void sig_handler(int sig)
{
	if(sig == MotorXSignal)
	{
		++counterX;
	}else if(sig == MotorZSignal)
	{
		++counterZ;
	}
}

int duty_to_ns(float duty, float periodo)
{
	return duty * periodo / 100.0;
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

void controlX(void *arg)
{
	Motor *m = arg;
	int i = 0;
	static unsigned datosvel[10] = {0,0,0,0,0,0,0,0,0,0};
	static float dis_old = 0;
	static float dis_new = 0;
	int cont0 = 0;
	int cont1 = 0;
	int err;

	rt_task_set_periodic(NULL, TM_NOW, 100000);

	while( !m->done ){
		for(i = 10; i > 0; i--)
		{
			datosvel[i]=datosvel[i-1];
		}
		datosvel[0] = counterX;
		dis_new = datosvel[0] * 0.000139509 * 2;
		dis_old = datosvel[10] * 0.000139509 * 2;
		m->vel = (dis_new - dis_old) * 1000.0 / 1.0;
		if(cont0 > 9 )
		{
			m->pid_val = pid(m->set,m->vel);
			cont1++;
			cont0 = 0;
		}
		cont0++;

		m->dutyns = duty_to_ns(m->pid_val,m->periodo);
		//printf("Velocidad: %f  dis_new: %f pid_val: %f duty: %f \n", vel, dis_new,pid_val,dutyns);

		if(dis_new >= m->distance)
		{
			m->done = TRUE;
		}
		err = rt_task_wait_period(NULL);
		if ( err != 0 )
		{
			switch(err)
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

void controlZ(void *arg)
{
	Motor *m = arg;
	int i = 0;
	static unsigned datosvel[10] = {0,0,0,0,0,0,0,0,0,0};
	static float dis_old = 0;
	static float dis_new = 0;
	int cont0 = 0;
	int cont1 = 0;
	int err;

	rt_task_set_periodic(NULL, TM_NOW, 100000);

	while( !m->done ){
		for(i = 10; i > 0; i--)
		{
			datosvel[i]=datosvel[i-1];
		}
		datosvel[0] = counterX;
		dis_new = datosvel[0] * 0.000139509 * 2;
		dis_old = datosvel[10] * 0.000139509 * 2;
		m->vel = (dis_new - dis_old) * 1000.0 / 1.0;
		if(cont0 > 9 )
		{
			m->pid_val = pid(m->set,m->vel);
			cont1++;
			cont0 = 0;
		}
		cont0++;

		m->dutyns = duty_to_ns(m->pid_val,m->periodo);
		//printf("Velocidad: %f  dis_new: %f pid_val: %f duty: %f \n", vel, dis_new,pid_val,dutyns);

		if(dis_new >= m->distance)
		{
			m->done = TRUE;
		}
		err = rt_task_wait_period(NULL);
		if ( err != 0 )
		{
			switch(err)
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

void movex(void *arg)
{
	Motor *m = arg;

	ixpio_reg_t reg;
	unsigned short int data = 0;
	int bit = 0;
	int err;

	rt_task_set_periodic(NULL, TM_NOW, m->periodo);

	reg.id = IXPIO_P3;
	reg.value = data;
	if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
		close(*m->fd);
		puts("Failure of configuring interrupt.");
	}

	while( !m->done )
	{
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
			close(*m->fd);
			puts("Failure of configuring interrupt.");
		}
		err = rt_task_sleep(m->dutyns);
		if ( err != 0 )
		{
			printf("\nsleep ERROR\n %d", err);
		}
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
			close(*m->fd);
			puts("Failure of configuring interrupt.");
		}
		err = rt_task_wait_period(NULL);
		if ( err != 0 )
		{
			switch(err)
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
	if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
		close(*m->fd);
		puts("Failure of configuring interrupt.");
	}
}

void movez(void *arg)
{
	Motor *m = arg;

	ixpio_reg_t reg;
	unsigned short int data = 0;
	int bit = 0;
	int err;

	rt_task_set_periodic(NULL, TM_NOW, m->periodo);

	reg.id = IXPIO_P3;
	reg.value = data;
	if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
		close(*m->fd);
		puts("Failure of configuring interrupt.");
	}

	while( !m->done )
	{
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
			close(*m->fd);
			puts("Failure of configuring interrupt.");
		}
		err = rt_task_sleep(m->dutyns);
		if ( err != 0 )
		{
			printf("\nsleep ERROR\n %d", err);
		}
		data ^= ( 1 << bit );
		reg.value = data;
		if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
			close(*m->fd);
			puts("Failure of configuring interrupt.");
		}
		err = rt_task_wait_period(NULL);
		if ( err != 0 )
		{
			switch(err)
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
	if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
		close(*m->fd);
		puts("Failure of configuring interrupt.");
	}
}

void catch_signal(int sig)
{
}

int main(int argc, char* argv[])
{
	Motor MotorX;
	Motor MotorZ;

	if (argc != 5)
	{
		printf("Not enough arguments supplied");
		return(1);
	}

	MotorX.set = atof(argv[1]);
	MotorX.distance = atof(argv[2]);
	MotorZ.set = atof(argv[3]);
	MotorZ.distance = atof(argv[4]);

	RT_TASK MoveMotorX;
	RT_TASK MoveMotorZ;
	RT_TASK ControlX;
	RT_TASK ControlZ;

	Motor *MotorX_ptr;
	Motor *MotorZ_ptr;
	MotorX_ptr= &MotorX;
	MotorZ_ptr= &MotorZ;

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

	/* Enable signal Handler */
	act.sa_handler = sig_handler;
	sigemptyset(&act.sa_mask);
	sigaddset(&act.sa_mask, MotorXSignal);
	if (sigaction(MotorXSignal, &act, &act_old)) {
		close(fd);
		puts("Failure of signal action.");
		return FAILURE;
	}

	sigaddset(&act.sa_mask, MotorZSignal);
	if (sigaction(MotorZSignal, &act, &act_old)) {
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
		sigaction(MotorZSignal, &act_old, NULL);
		puts("Failure of configuring port.");
		return FAILURE;
	}

	reg.id = IXPIO_PCC;
	reg.value = 0x01; // Enable P6 as Output
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		sigaction(MotorZSignal, &act_old, NULL);
		puts("Failure of configuring port.");
		return FAILURE;
	}

	/* Interrupts Configuration */
	reg.id = IXPIO_IMCR;
	reg.value = 0x06; // Enable P5C0 and P8C0 Interrupts
	if (ioctl(fd, IXPIO_WRITE_REG, &reg)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		sigaction(MotorZSignal, &act_old, NULL);
		puts("Failure of configuring interrupt.");
		return FAILURE;
	}

	/* Signal Condiction */
	sig.sid = MotorXSignal;
	sig.pid = getpid();
	sig.is = 0x02;   /* Signal for the P5C0 channel */
	sig.edge = 0x02;  /* High level trigger */
	if (ioctl(fd, IXPIO_SET_SIG, &sig)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		sigaction(MotorZSignal, &act_old, NULL);
		puts("Failure of signal condiction.");
		return FAILURE;
	}
	sig.sid = MotorZSignal;
	sig.pid = getpid();
	sig.is = 0x04;   /* Signal for the P8C0 channel */
	sig.edge = 0x04;  /* High level trigger */
	if (ioctl(fd, IXPIO_SIG_ADD, &sig)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		sigaction(MotorZSignal, &act_old, NULL);
		puts("Failure of signal condiction.");
		return FAILURE;
	}

	MotorX.fd = &fd;
	MotorZ.fd = &fd;
	MotorX.periodo= 1000000;
	MotorZ.periodo= 1000000;
	MotorX.dutyns = 500000;
	MotorZ.dutyns = 500000;
	MotorX.distance = 3;
	MotorZ.distance = 3;
	MotorX.done = FALSE;
	MotorZ.done = FALSE;
	MotorX.set = 1;
	MotorZ.set = 1;

	/* Xenomai */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_create(&ControlX, "controlx", 0, 90, T_JOINABLE );
	rt_task_create(&MoveMotorX, "mmotorx", 0, 99, T_JOINABLE );
	rt_task_create(&ControlZ, "controlz", 0, 90, T_JOINABLE );
	rt_task_create(&MoveMotorZ, "mmotorz", 0, 99, T_JOINABLE );
	rt_task_start(&ControlX, &controlX, (void*)MotorX_ptr);
	rt_task_start(&ControlZ, &controlZ, (void*)MotorZ_ptr);
	rt_task_start(&MoveMotorX, &movex, (void*)MotorX_ptr);
	rt_task_start(&MoveMotorZ, &movez, (void*)MotorZ_ptr);
	rt_task_join(&ControlX);
	rt_task_join(&ControlZ);
	rt_task_delete(&ControlX);
	rt_task_delete(&ControlZ);
	rt_task_delete(&MoveMotorX);
	rt_task_delete(&MoveMotorZ);

	/* Ixpio */
	close(fd);
	sigaction(MotorXSignal, &act_old, NULL);
	sigaction(MotorZSignal, &act_old, NULL);
	return 0;
}
