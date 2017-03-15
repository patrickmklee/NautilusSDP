#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <string.h>
#include <pigpio.h>
#include "receiver.h"
#define MAX_LINE 6

#define NEUTRAL_THROTTLE	1500
#define PIN_LTHRUST		19
#define PIN_RTHRUST		16
#define PIN_LTOP		26
#define PIN_RTOP		20
#define PIN_FTOP		21
#define NUM_MOTORS		5

const char pinList[5] = {PIN_LTHRUST, PIN_RTHRUST, PIN_LTOP, PIN_RTOP, PIN_FTOP};

//Motor Functions

void goForward(char percent) { //Use negative to reverse
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, pulse);
}

void turnLeft(char percent) {
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, NEUTRAL_THROTTLE);
}

void turnRight(char percent) {
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, NEUTRAL_THROTTLE);
}

void dive(char percent) { //Use negative percent to rise
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTOP, pulse);
	gpioServo(PIN_RTOP, pulse);
	gpioServo(PIN_FTOP, pulse);
}

void initMotors() {
	uint8_t i;
	for (i=0;i<NUM_MOTORS;i++){
		gpioServo(pinList[i], NEUTRAL_THROTTLE);
	}

}
void *receiveCmds(void * mArgs) {
   initMotors(); 
   char wr_cmd[2];
   strcpy(wr_cmd, "FD");
   char keyup_cmd[2];
   char keydown_cmd[2];
   strcpy(keyup_cmd, "up");
   strcpy(keydown_cmd, "dn");

   char *line=malloc(sizeof(char)*6);
   char *sub_line;
   int pipe;
   int cmdVal;
   int bytes_read;
   //if (gpioInitialise()<0) return -1;
   //wiringPiSetup();

   // open the named pipe
   pipe = open("/var/www/html/myFIFO", O_RDONLY | O_NONBLOCK);

   int hAccel, vAccel = 0;
   int fwdHold, revHold, diveHold, riseHold = 0;
   while (1) {
      if ((bytes_read = read(pipe, line, MAX_LINE)) > 0){
        if (strncmp(line,keyup_cmd,2) == 0){
		//dir=0;
		printf("Stop ");
	 } else if (strncmp(line,keydown_cmd,2)==0){
		//dir=1;
		printf("Go ");
	 }
	 sub_line = (line+2*sizeof(char));		
	 cmdVal = atoi(sub_line);
	 //line[bytes_read-1]='\0'; //replace LF char
	 //printf("%d\n",strcmp(line,wr_cmd));	

	 if(cmdVal == 38) { //Arrow up
	 	if ( (line[0] == 'u') && (line[1] == 'p') ){
			riseHold = 0;
		} else if ( (line[0] == 'd') && (line[1] == 'n') && !diveHold ){
			riseHold = 1;
		}
		printf("Up\n");

	 } else if (cmdVal == 40) { //Arrow Down
		if ( (line[0] == 'u') && (line[1] == 'p') ){
			diveHold = 0;
		} else if ( (line[0] == 'd') && (line[1] == 'n') && !riseHold ){
			diveHold = 1;
		}
		printf("Down\n");

	 } else if (cmdVal == 39){ //Arrow right
		printf("Right\n");

	 } else if (cmdVal == 37) { //Arrow left
		printf("Left\n");

	 } else if (cmdVal == 32) { //Spacebar
		if ( (line[0] == 'u') && (line[1] == 'p') ) {
			fwdHold = 0;
		} else if ( (line[0] == 'd') && (line[1] == 'n') && !revHold) {
			fwdHold = 1;
		}
		printf("Forward\n");	

	 } else if (cmdVal == 82) { // R key
	 	if ( (line[0] == 'u') && (line[1] == 'p') ) {
			revHold = 0;
		} else if ( (line[0] == 'd') && (line[1] == 'n') && !fwdHold) {
			revHold = 1;
		}
		printf("Reverse\n");

	 } else
         	printf("Received: %s\n",line);
      }

      if (diveHold){
      	//Accelerate down
      	if (vAccel < 100) vAccel += 5;
      } else if (riseHold){
      	//Accelerate up
      	if (vAccel > -100) vAccel -= 5;
      } else {
      	//Decelerate
      	if (vAccel < 0) vAccel += 5;
      	if (vAccel > 0) vAccel -= 5;
      }

      if (fwdHold){
      	//Accelerate forward
      	if (hAccel < 100) hAccel += 5;
      } else if (revHold){
      	//Accelerate backward
      	if (hAccel > -100) hAccel -= 5;
      } else {
      	if (hAccel < 0) hAccel += 5;
      	if (hAccel > 0) hAccel -= 5;
      }

      if (vAccel != 0) dive(vAccel);
      if (hAccel != 0) goForward(hAccel);


      delay(150);
	// else
        // break;
   }	   
    // close the pipe
   close(pipe);
   free(line);
   pthread_exit(NULL);
}
