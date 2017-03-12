#include <pigpio.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include "LSM6DS3_Collection.h"
#include "receiver.h"
pthread_t tid[2];


//-------------------- PID PARAMETER DEFS -----------------------
//Adjust KP,KI,KD
#define KP	50
#define KI	20
#define	KD	10
#define T_ROLL	0	//target tilt

//------------ MAIN FUNCTION ----------------------
int main(void) {
	wiringPiSetup();
	gpioInitialise();
	uint8_t *mArgs; // placeholder not used right now
	gArgs = malloc(sizeof(struct pCollection_args)); //allocate mem for gyro data - gArgs is updated by runCollection function

	// Start individual threads
	uint8_t err;
	err = pthread_create(&(tid[0]), NULL, &runCollection, (void*)gArgs);
	if (err != 0) {
		perror("IMU thread error");
	} else {
		printf("IMU thread created\n");
	}
	err = pthread_create(&(tid[1]), NULL, &receiveCmds, (void*)mArgs);
	if (err != 0) {
		perror("Motor thread error");
	} else {
		printf("Motor Control thread created\n");
	}
	float y_out;
	float y_err_p;
	float y_err_prev;
	float y_err_i=0;
	float y_err_d;
	delay(2000);
	while(1) {
		delay(100);
		
		//printf("TILT STATUS\n========================\n");
		printf("x: %0.4f\n", gArgs->x);//(float)*(pCollection+sizeof(float)));
		printf("y: %0.4f\n", gArgs->y);//(float)*(pCollection+sizeof(float)*2));

//			 ------ PID LOOP -------
		y_err_p = round(gArgs->y-T_ROLL*10)/10; // P
		y_err_i += y_err_p; 			// I
		y_err_d = y_err_prev - y_err_p;		// D
		y_err_prev = y_err_p;
		y_out =	KP*(y_err_p) + KI*(y_err_i) + KD*(y_err_d);
		printf("y-correction: %0.2f\n", y_out);

	}
	free(gArgs);
	pthread_exit(NULL);
	//return 0;
}
