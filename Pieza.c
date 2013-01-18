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
#include "ixpio.h"

#define TRUE 1
#define FALSE 0
#define MotorXSignal 40
#define MotorZSignal 41

unsigned counterX, counterZ;
char Exit;

typedef struct motor{
	int * fd;
	int periodo;
	int done;
	float set;
	float distance;
	float vel;
	float pid_val;
	int direction;
	RTIME dutyns;
}Motor;

int duty_to_ns(float duty, float periodo);
float pid(float sp, float pv, int axis);

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

float pid(float sp, float pv, int axis)
{
	static float err_old;
	static float P_err, I_err, D_err;
	static float err;
	float Kp, Kd, Ki;
	float pid;

	if(axis){
		sp = (239 * sp) - 65;
	}else{
		sp = (240 * sp) - 58.4;
	}
	/*	if(sp > 0.5){
		sp = - 121.87 * (sp*sp*sp*sp*sp) + 862.97 * (sp*sp*sp*sp) - 2301.2 * (sp*sp*sp) + 2852.8 * (sp*sp) - 1385.3 * sp + 213.91;
		}else{
		sp = 1.5333 * (sp*sp) + 0.3159 * sp - 0.1051;
		}*/

	Kp =0.1839235159018;
	Kd =0.000000001;
	Ki =0.0672091666664;

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

	while( !m->done && Exit == FALSE ){
		for(i = 9; i > 0; i--)
		{
			datosvel[i]=datosvel[i-1];
		}
		datosvel[0] = counterX;
		dis_new = datosvel[0] * 0.000139509 * 2 * 2 / 2.0;
		dis_old = datosvel[9] * 0.000139509 * 2 * 2 / 2.0;
		m->vel = (dis_new - dis_old) * 1000.0 / 1.0;
		if(cont0 > 9 )
		{
			m->pid_val = pid(m->set,m->vel,0);
			cont1++;
			cont0 = 0;
		}
		cont0++;

		m->dutyns = duty_to_ns(m->pid_val,m->periodo);
		//printf("VelocidadX: %f  dis_newX: %f pid_valX: %f dutyX: %f \n", m->vel, dis_new, dis_old, m->pid_val, m->dutyns);
		//printf("VelocidadX: %f \n", m->vel);
		//	printf("dis_newX: %f\n", dis_new);

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

	while( !m->done && Exit == FALSE ){
		for(i = 9; i > 0; i--)
		{
			datosvel[i] = datosvel[i - 1];
		}
		datosvel[0] = counterZ;
		dis_new = datosvel[0] * 0.000139509 * 2 * 2 / 2.0;
		dis_old = datosvel[9] * 0.000139509 * 2 * 2 / 2.0;
		m->vel = (dis_new - dis_old) * 1000.0 / 1.0;
		if(cont0 > 9 )
		{
			m->pid_val = pid(m->set,m->vel,1);
			cont1++;
			cont0 = 0;
		}
		cont0++;

		m->dutyns = duty_to_ns(m->pid_val,m->periodo);
		//printf("VelocidadZ: %f  dis_newZ: %f pid_valZ: %f dutyZ: %f \n", m->vel, dis_new, m->pid_val, m->dutyns);
		//printf("VelocidadZ: %f \n", m->vel);
		//printf("dis_newZ: %f\n", dis_new);

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
		}

	}
	m->done = FALSE;
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
	printf("%d",m->done);
	while( !m->done && Exit == FALSE )
	{	
		// No se necesita invertir dos veces el bit.
		if(m->direction == 0){
			data ^= ( 1 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
			err = rt_task_sleep(m->dutyns);
			if ( err != 0 )
			{
				printf("sleep ERROR\n %d", err);
			}
			data ^= ( 1 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
		}else{
			data ^= ( 2 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
			err = rt_task_sleep(m->dutyns);
			if ( err != 0 )
			{
				printf("sleep ERROR\n %d", err);
			}
			data ^= ( 2 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
		}
		err = rt_task_wait_period(NULL);
		if ( err != 0 )
		{
			switch(err)
			{
				case -ETIMEDOUT:
					printf("ETIMEOUTX\n");
					break;
				case -EINTR:
					printf("EINTR\n");
					break;
				case -EPERM:
					printf("EPERM\n");
					break;
				case -EWOULDBLOCK:
					printf("EWOULDBLOCK\n");
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
	m->done = FALSE;
}

void movez(void *arg)
{
	Motor *m = arg;

	ixpio_reg_t reg;
	unsigned short int data = 0;
	int bit = 0;
	int err;

	rt_task_set_periodic(NULL, TM_NOW, m->periodo);

	reg.id = IXPIO_P6;
	reg.value = data;
	if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
		close(*m->fd);
		puts("Failure of configuring interrupt.");
	}

	while( !m->done && Exit == FALSE )
	{
		if(m->direction == 0){
			data ^= ( 1 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
			err = rt_task_sleep(m->dutyns);
			if ( err != 0 )
			{
				printf("sleep ERROR\n %d", err);
			}
			data ^= ( 1 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
		}else{
			data ^= ( 2 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
			err = rt_task_sleep(m->dutyns);
			if ( err != 0 )
			{
				printf("sleep ERROR\n %d", err);
			}
			data ^= ( 2 << bit );
			reg.value = data;
			if (ioctl(*m->fd, IXPIO_WRITE_REG, &reg)) {
				close(*m->fd);
				puts("Failure of configuring interrupt.");
			}
		}
		err = rt_task_wait_period(NULL);
		if ( err != 0 )
		{
			switch(err)
			{
				case -ETIMEDOUT:
					printf("ETIMEOUTZ\n");
					break;
				case -EINTR:
					printf("EINTR\n");
					break;
				case -EPERM:
					printf("EPERM\n");
					break;
				case -EWOULDBLOCK:
					printf("EWOULDBLOCK\n");
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

	m->done = FALSE;
}

void catch_signal(int sig)
{
	if( sig == SIGTERM)
	{
		printf("TERM\n");
		Exit = TRUE;
	}else if( sig == SIGINT)
	{
		printf("INT\n");
	}
}

int main(int argc, char* argv[])
{
	Motor MotorX;
	Motor MotorZ;
	Exit = FALSE;

	RT_TASK MoveMotorX;
	RT_TASK MoveMotorZ;
	RT_TASK ControlX;
	RT_TASK ControlZ;

	Motor *MotorX_ptr;
	Motor *MotorZ_ptr;
	MotorX_ptr = &MotorX;
	MotorZ_ptr = &MotorZ;

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
	sig.bedge = 0x02;  /* High level trigger */
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
	sig.bedge = 0x04;  /* High level trigger */
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
	MotorX.done = FALSE;
	MotorZ.done = FALSE;
	MotorX.set = 1;
	MotorZ.set = 1;
	MotorX.distance = 1;
	MotorZ.distance = 1;
	MotorX.direction = 1;
	MotorZ.direction = 1;

	/* Xenomai */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	rt_task_create(&ControlX, "controlx", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_create(&MoveMotorX, "mmotorx", 0, 99, T_JOINABLE  | T_CPU(3));
	rt_task_create(&ControlZ, "controlz", 0, 90, T_JOINABLE  | T_CPU(2));
	rt_task_create(&MoveMotorZ, "mmotorz", 0, 99, T_JOINABLE  | T_CPU(1));
	MotorX_ptr->distance = 4;
	MotorZ_ptr->distance = 0;
	MotorX_ptr->direction = 1;
	MotorZ_ptr->direction = 0;
	rt_task_start(&ControlX, &controlX, (void*)MotorX_ptr);
	rt_task_start(&MoveMotorX, &movex, (void*)MotorX_ptr);
	rt_task_start(&ControlZ, &controlZ, (void*)MotorZ_ptr);
	rt_task_start(&MoveMotorZ, &movez, (void*)MotorZ_ptr);
	rt_task_join(&ControlX);
	rt_task_join(&ControlZ);
	rt_task_join(&MoveMotorX);
	rt_task_join(&MoveMotorZ);
	/* Borrar Tareas */
	rt_task_delete(&ControlX);
	rt_task_delete(&MoveMotorX);
	rt_task_delete(&MoveMotorZ);
	rt_task_delete(&ControlZ);

	rt_task_create(&ControlX, "controlx", 0, 90, T_JOINABLE | T_CPU(0));
	rt_task_create(&MoveMotorX, "mmotorx", 0, 99, T_JOINABLE  | T_CPU(3));
	rt_task_create(&ControlZ, "controlz", 0, 90, T_JOINABLE  | T_CPU(2));
	rt_task_create(&MoveMotorZ, "mmotorz", 0, 99, T_JOINABLE  | T_CPU(1));
	MotorX_ptr->distance = 2;
	MotorZ_ptr->distance = 0;
	MotorX_ptr->direction = 0;
	MotorZ_ptr->direction = 1;
	rt_task_start(&ControlX, &controlX, (void*)MotorX_ptr);
	rt_task_start(&MoveMotorX, &movex, (void*)MotorX_ptr);
	rt_task_start(&ControlZ, &controlZ, (void*)MotorZ_ptr);
	rt_task_start(&MoveMotorZ, &movez, (void*)MotorZ_ptr);
	rt_task_join(&ControlX);
	rt_task_join(&ControlZ);
	rt_task_join(&MoveMotorX);
	rt_task_join(&MoveMotorZ);

	/* Borrar Tareas */
	rt_task_delete(&ControlX);
	rt_task_delete(&MoveMotorX);
	rt_task_delete(&MoveMotorZ);
	rt_task_delete(&ControlZ);

	/* Ixpio */
	close(fd);
	sigaction(MotorXSignal, &act_old, NULL);
	sigaction(MotorZSignal, &act_old, NULL);
	return 0;
}
