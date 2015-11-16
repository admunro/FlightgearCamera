



### Compiler, tools and options

CC          = gcc
CXX         = g++
DEFINES     = 
INCPATH     = -I./src 
LINK        = g++
CFLAGS      = -c -Wall
LFLAGS      = 
EXECUTABLE  = flightGearldp

SOURCES = ./src/flightGearLDP.cpp \
          ./src/LDPMaths.cpp \
          ./src/Tracking.cpp

OBJECTS = ./build/flightGearLDP.o \
          ./build/LDPMaths.o \
          ./build/Tracking.o

flightgearLDP: $(OBJECTS)
	$(CXX) -o flightgearLDP $(OBJECTS)

build/%.o : src/%.cpp
	$(CXX) $(CFLAGS) -o "$@" "$<"


.PHONY : clean
clean:
	rm build/*.o flightGearLDP
   
