CC            =  g++
CPPFLAGS      = -Wall -g

# Uncomment the following line for a verbose client
#CPPFLAGS      = -Wall -g -D __UDP_CLIENT_VERBOSE__

# Put here the name of your driver class
DRIVER_CLASS = Axel
# Put here the filename of your driver class header 
DRIVER_INCLUDE = '"$(DRIVER_CLASS).hpp"'
DRIVER_OBJ = $(DRIVER_CLASS).o
# Additional driver files
DRIVER_AI = robot_axel/Settings.o robot_axel/Random_Generator.o robot_axel/Genome.o robot_axel/Gym.o robot_axel/AI.o
EXTFLAGS = -D __DRIVER_CLASS__=$(DRIVER_CLASS) -D __DRIVER_INCLUDE__=$(DRIVER_INCLUDE)
OBJECTS = WrapperBaseDriver.o SimpleParser.o CarState.o CarControl.o $(DRIVER_AI) $(DRIVER_OBJ) 

all: $(OBJECTS) client

.SUFFIXES : .o .cpp .c

.cpp.o :
	$(CC) $(CPPFLAGS) -o $@  $(EXTFLAGS) -c $<

.c.o :
	$(CC) $(CPPFLAGS) $(EXTFLAGS) -c $<

client: client.cpp $(OBJECTS)
	$(CC) $(CPPFLAGS) $(EXTFLAGS) -o client client.cpp $(OBJECTS)

clean:
	rm -f *.o client  
 	