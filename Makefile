FLAGS	= -Wall -g 
LINKS	= -lpigpio -lwiringPi -lpthread -lm -lrt
SOURCES = TopControl.c LSM6DS3_Collection.c receiver.c LinkedListStruct.c ms5803.c MS5803_Collection.c IMU.c FeedbackControlTop.c FeedbackMotorController.c
OBJECTS = TopControl.o LSM6DS3_Collection.o receiver.o LinkedListStruct.o ms5803.o MS5803_Collection.o IMU.o FeedbackControlTop.o FeedbackMotorController.o
HEADERS = LSM6DS3_Collection.h LinkedListStruct.h receiver.h ms5803.h MS5803_Collection.h IMU.h FeedbackControlTop.h FeedbackMotorController.h
EXEBIN  = top

all:	$(EXEBIN)

$(EXEBIN) : $(OBJECTS) $(HEADERS)
	gcc -o $(EXEBIN) $(OBJECTS) $(LINKS)

$(OBJECTS) : $(SOURCES) $(HEADERS)
	gcc -c $(FLAGS) $(SOURCES)

clean : 
	rm -f $(EXEBIN) $(OBJECTS) 
