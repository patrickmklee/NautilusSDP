FLAGS	= -std=c99 -Wall -g
LINKS	= -lpigpio -lwiringPi -lpthread -lm
SOURCES = TopControl.c LSM6DS3_Collection.c receiver.c LinkedListStruct.c #ms5803.c
OBJECTS = TopControl.o LSM6DS3_Collection.o receiver.o LinkedListStruct.o #ms5803.o
HEADERS = LSM6DS3_Collection.h LinkedListStruct.h receiver.h #ms5803.h
EXEBIN  = top

all:	$(EXEBIN)

$(EXEBIN) : $(OBJECTS) $(HEADERS)
	gcc -o $(EXEBIN) $(OBJECTS) $(LINKS)

$(OBJECTS) : $(SOURCES) $(HEADERS)
	gcc -c $(FLAGS) $(SOURCES)

clean : 
	rm -f $(EXEBIN) $(OBJECTS) 
