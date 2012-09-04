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

int duty_to_ns(float duty, float periodo);
float pid(float sp, float pv);

void sig_handler(int sig)
{
	static unsigned sig_counter;
	++sig_counter;
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
}

void controlZ(void *arg)
{
}

void movex(void *arg)
{
}

void movez(void *arg)
{
}

void catch_signal(int sig)
{
}

int main(int argc, char* argv[])
{
	float setX,setZ,distaX,distaZ;

	if (argc != 5)
	{
		return(1);
	}
	setX = atof(argv[1]);
	distaX = atof(argv[2]);
	setZ = atof(argv[3]);
	distaZ = atof(argv[4]);

	RT_TASK MoveMotorX;
	RT_TASK MoveMotorZ;
	RT_TASK ControlX;
	RT_TASK ControlZ;

	/* Xenomai */
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	/* Ixpio */
	char *dev_file;
	ixpio_reg_t reg;
	ixpio_signal_t sig;
	static struct sigaction act, act_old;
	int fd;

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
	sig.is = 0x06;   /* Signal for the P5C0 and P8C0 channels */
	sig.edge = 0x06;  /* High level trigger */
	if (ioctl(fd, IXPIO_SET_SIG, &sig)) {
		close(fd);
		sigaction(MotorXSignal, &act_old, NULL);
		puts("Failure of signal condiction.");
		return FAILURE;
	}


	/* Xenomai */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	rt_task_create(&ControlX, "controlx", 0, 90, T_JOINABLE );
	rt_task_create(&MoveMotorX, "mmotorx", 0, 99, T_JOINABLE );
	rt_task_create(&ControlZ, "controlz", 0, 90, T_JOINABLE );
	rt_task_create(&MoveMotorZ, "mmotorz", 0, 99, T_JOINABLE );
	rt_task_start(&ControlX, &controlX, NULL);
	rt_task_start(&ControlZ, &controlZ, NULL);
	rt_task_start(&MoveMotorX, &movex, NULL);
	rt_task_start(&MoveMotorZ, &movez, NULL);
	rt_task_join(&ControlX);
	rt_task_join(&ControlZ);
	rt_task_delete(&ControlX);
	rt_task_delete(&ControlZ);
	rt_task_delete(&MoveMotorX);
	rt_task_delete(&MoveMotorZ);

	/* Ixpio */
	sigaction(MotorXSignal, &act_old, NULL);
	close(fd);
	return 0;
}
