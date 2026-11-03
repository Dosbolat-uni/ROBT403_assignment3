# ROBT402 Project 3 - Task 4
# Needs lgpio:  sudo apt install liblgpio-dev
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -pthread
CFLAGS   = -O2 -Wall

task4_control: task4_control.o IMUSensor.o I2CDevice.o MadgwickAHRS.o
	g++ $(CXXFLAGS) $^ -llgpio -lm -o $@

MadgwickAHRS.o: MadgwickAHRS.c MadgwickAHRS.h
	gcc $(CFLAGS) -c $< -o $@          # compiled as C (header is included via extern "C")

%.o: %.cpp
	g++ $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o task4_control
