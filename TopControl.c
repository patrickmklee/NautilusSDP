#include <pigpio.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include "LSM6DS3_Collection.h"
#include "MS5803_Collection.h"
#include "FeedbackControlTop.h"
//#include "FeedbackMotorController.h"
#include "receiver.h"
#include "IMU.h"

pthread_t tid[4];


	
//-------------------- PID PARAMETER DEFS -----------------------
//Adjust KP,KI,KD
#define ROLL_KP	2
#define ROLL_KI	0	
#define	ROLL_KD	1
#define T_ROLL	0	//target tilt

struct pCollection_args *gArgs;
struct pCollection_args *gDeltaArgs;
struct AbsolutePositionStruct	*AbsolutePosition;
struct CoordinateAxis *OrdinalAxis;
//struct motorControl_args *mArgs;
struct TopFeedbackStateStruct *fbArgs;
//------------ MAIN FUNCTION ----------------------
int main(void) {
	wiringPiSetup();
	gpioInitialise();
	
//	mArgs = malloc(sizeof(struct motorControl_args));	
	psArgs = malloc(sizeof(struct presCollection_args));
	gArgs = malloc(sizeof(struct pCollection_args)); //allocate mem for gyro data - gArgs is updated by runCollection function
	fbArgs = malloc(sizeof(struct TopFeedbackStateStruct));
	AbsolutePosition= malloc(sizeof(struct AbsolutePositionStruct));
	gDeltaArgs = malloc(sizeof(struct pCollection_args));
	fbArgs->rollState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->pitchState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->yawState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->VxState = malloc(sizeof(struct AxisFeedbackState));
	fbArgs->zState = malloc(sizeof(struct AxisFeedbackState));
	OrdinalAxis = malloc(sizeof(struct CoordinateAxis));

	OrdinalAxis->X=0.0;
	OrdinalAxis->Y=0.0;
	OrdinalAxis->Z=1.0;
	gArgs->CAL_COMPLETE=0;
	uint8_t *tpArgs;
	// Start individual threads
	uint8_t err;
	err = pthread_create(&(tid[0]), NULL, &runCollection, (void*)gArgs);
	if (err != 0) {
		perror("IMU thread error");
	} else {
		printf("IMU thread created\n");
	}
	err = pthread_create(&(tid[1]), NULL, &receiveCmds, (void*)fbArgs);
	if (err != 0) {
		perror("User input thread error");
	} else {
		printf("User input thread created\n");
	}
	err = pthread_create(&(tid[2]), NULL, &runPresCollection, (void*)psArgs);
	if (err != 0) {
		perror("Pressure thread error");
	} else {
		printf("Pressure thread created\n");
	}
	float x_out;
	float x_err_p;
	float x_err_prev;
	float x_err_i=0;
	float x_err_d;
	//printf("Waiting for Calibration...\n");
	while (gArgs->CAL_COMPLETE==0) {;}	
	float prevX=0, prevY=0, prevZ=0;
	fbArgs->rollState->currentValue = 0;//gArgs->x;
	fbArgs->zState->currentValue = 0;//gArgs->x;
	err = pthread_create(&(tid[3]), NULL, &TopControlLoop, (void*)fbArgs);
	if (err != 0) {
		perror("Feedback Control thread error");
	} else {
		printf("Feedback Control thread created\n");
	}
	//struct pCollection_args deltaAngles;
	while(1) {
		delay(10);
		fbArgs->rollState->currentValue = round(gArgs->x*10)/10.0;
		//fbArgs->zState->currentValue = 0;//gArgs->x;
		/*deltaAngles.x = prevX-gArgs->x;
		deltaAngles.y = prevY-gArgs->y;
		deltaAngles.z = prevZ-gArgs->z;
		prevX=gArgs->x;
		prevY=gArgs->y;
		prevZ=gArgs->z;*/
		/*ordinalAxis->Z = gArgs->xl_z;
		ordinalAxis->Y = gArgs->xl_y;
		ordinalAxis->X = gArgs->xl_x;*/
		//OrdinalAxis->Z = gArgs->xl_z;
		//updateXLCoordinateAxis(*gArgs,OrdinalAxis);
		//printf("TILT STATUS\n========================\n");
		printf("\rx: %6.3f", gArgs->x);//(float)*(pCollection+sizeof(float)));
		printf(" | y: %6.3f", gArgs->y);//(float)*(pCollection+sizeof(float)*2));
		printf(" | z: %6.3f", gArgs->z);
		printf(" | xl_x: %6.3f",gArgs->xl_x);
		printf(" | xl_y: %6.3f",gArgs->xl_y);
		printf(" | xl_z: %6.3f",gArgs->xl_z);
		printf(" | Vx: %6.3f", AbsolutePosition->vX);
		printf(" | OX: %6.3f",OrdinalAxis->X);
		printf(" | OY: %6.3f",OrdinalAxis->Y);
		printf(" | OZ: %6.3f",OrdinalAxis->Z);
		printf(" | Zset: %5.2f", fbArgs->zState->setpoint);
		fflush(stdout);
//			 ------ PID LOOP -------
		/*x_err_p = round(gArgs->x-T_ROLL*10)/10; // P
		x_err_i += x_err_p; 			// I
		x_err_d = x_err_prev - x_err_p;		// D
		x_err_prev = x_err_p;
		x_out =	ROLL_KP*(x_err_p) + ROLL_KI*(x_err_i) + ROLL_KD*(x_err_d);
		mArgs->roll_correction = x_out;*/
		
		//printf("x-correction: %0.2f\n================================\n", x_out);
		//printf("Pressure: %0.2f\nTemperature: %0.2f CEL\n============================================\n", psArgs->pres,psArgs->temp);

	}
	free(gArgs);
	free(psArgs);
	pthread_exit(NULL);
	return 0;
}
