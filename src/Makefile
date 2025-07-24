



### Compiler, tools and options

CC          = gcc
CXX         = g++
DEFINES     = 
INCPATH     = -I./src 
LINK        = g++
CFLAGS      = -c -Wall
LFLAGS      = 
EXECUTABLE  = flightGearldp

SOURCES = flightGearLDP.cpp \
          LDPMaths.cpp \
          Tracking.cpp

OBJECTS = flightGearLDP.o \
          LDPMaths.o \
          Tracking.o

flightgearLDP: $(OBJECTS)
	$(CXX) -o flightgearLDP $(OBJECTS)

.cpp.o:
	$(CXX) $(CFLAGS) -o "$@" "$<"


.PHONY : clean
clean:
	rm -f *.o flightgearLDP
   
