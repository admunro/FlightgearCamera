vpath %.hpp ./src
vpath %.h ./src

BUILD = ./build
SRC   = ./src

CXX      = g++
CXXFLAGS = -Wall

exec_name = manualSlewTest
objects   = $(addprefix $(BUILD)/, flightGearLDP.o Tracking.o LDPMaths.o)

flightGearLDP : $(objects)
	$(CXX) $(CXXFLAGS) -o $(exec_name) $(objects) 

$(BUILD)/flightGearLDP.o : Tracking.hpp LDPMaths.hpp Waypoints.hpp
	$(CXX) $(CXXFLAGS) -c $(SRC)/flightGearLDP.cpp -o $(BUILD)/flightGearLDP.o
   
$(BUILD)/Tracking.o : Tracking.hpp
	$(CXX) $(CXXFLAGS) -c $(SRC)/Tracking.cpp -o $(BUILD)/Tracking.o 

$(BUILD)/LDPMaths.o : LDPMaths.hpp
	$(CXX) $(CXXFLAGS) -c $(SRC)/LDPMaths.cpp -o $(BUILD)/LDPMaths.o 


clean :
	-rm $(BUILD)/* $(exec_name)
