FLAGS	= -std=c99 -Wall -g
LINKS	= -lpigpio -lwiringPi -lpthread -lm
SOURCES = TopControl.c LSM6DS3_Collection.c receiver.c LinkedListStruct.c
OBJECTS = TopControl.o LSM6DS3_Collection.o receiver.o LinkedListStruct.o
HEADERS = LSM6DS3_Collection.h LinkedListStruct.h receiver.h
EXEBIN  = top

all:	$(EXEBIN)

$(EXEBIN) : $(OBJECTS) $(HEADERS)
	gcc -o $(EXEBIN) $(OBJECTS) $(LINKS)

$(OBJECTS) : $(SOURCES) $(HEADERS)
	gcc -c $(FLAGS) $(SOURCES)

clean : 
	rm -f $(EXEBIN) $(OBJECTS) 
