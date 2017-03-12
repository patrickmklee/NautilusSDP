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

void goForward(char percent) {
	int pulse;
	pulse = NEUTRAL_THROTTLE+(500*percent)/100;
	gpioServo(PIN_LTHRUST, pulse);
	gpioServo(PIN_RTHRUST, pulse);
}

void dive(char percent) {
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
	//gpioServo(PIN_LTHRUST, NEUTRAL_THROTTLE);
	//gpioServo(PIN_RTHRUST, NEUTRAL_THROTTLE);
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
	 if(cmdVal == 38) {
		printf("Up\n");
	 } else if (cmdVal == 39){
		printf("Right\n");
	 } else if (cmdVal == 37) {
		printf("Left\n");
	 } else if (cmdVal == 40) {
		if ( (line[0] == 'u') && (line[1] == 'p') ){
			dive(0);
		} else if ( (line[0] == 'd') && (line[1] == 'n') ){
			dive(20);
		}
		printf("Down\n");
	 } else if (cmdVal == 32) {
		if ( (line[0] == 'u') && (line[1] == 'p') ) {
			goForward(0);
		} else if ( (line[0] == 'd') && (line[1] == 'n') ) {
			goForward(20);
		}
		printf("Forward\n");	
	 } else
         	printf("Received: %s\n",line);
      }
      delay(150);
	// else
        // break;
   }	   
    // close the pipe
   close(pipe);
   free(line);
   pthread_exit(NULL);
}
