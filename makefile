# change application name here (executable output name)
TARGET = app
# current directory
current_dir = $(shell pwd)
$(info $(current_dir))
# source code directory
SOURCE_DIR = $(current_dir)/source
# binaries directory
BINARIES_DIR = $(current_dir)/bin
# compiler
CC = gcc
LD = g++
# includes
INC = \
	-I$(SOURCE_DIR)/arm \
	-I$(SOURCE_DIR)/quat_lib \
	-I$(SOURCE_DIR)/math
# libraries
IMULIB= -L$(current_dir)/lib -lLpSensor -lstdc++ -lX11
# debug
DEBUG = -g
# optimisation
OPT = -O0
# warnings
WARN = -Wall
# flags
CCFLAGS = $(DEBUG) $(OPT) $(WARN)
CPPFLAGS = $(DEBUG) $(OPT) $(WARN)

OBJS =  quaternion.o arm.o vector3.o imu.o

all: $(OBJS) main.o
	$(info building target ...)
	$(LD) $(INC) $(BINARIES_DIR)/*.o -o $(TARGET) -lm -ldl

test: $(OBJS) 
	cd test && make && cd -

main.o: $(SOURCE_DIR)/main.c bin_dir
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/main.c -o $(BINARIES_DIR)/$@

quaternion.o: $(SOURCE_DIR)/quat_lib/Quaternion.c bin_dir
	$(CC) -c  $(CPPFLAGS) $(SOURCE_DIR)/quat_lib/Quaternion.c -o $(BINARIES_DIR)/$@ 

arm.o: $(SOURCE_DIR)/arm/arm.c vector3.o bin_dir
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/arm/arm.c -o $(BINARIES_DIR)/$@ 

vector3.o: $(SOURCE_DIR)/math/vector3.c bin_dir
	$(CC) -c  $(CPPFLAGS) $(SOURCE_DIR)/math/vector3.c -o $(BINARIES_DIR)/$@ 

imu.o: $(SOURCE_DIR)/imu/imu.cpp bin_dir
	$(LD) -c $(CCFLAGS) $(SOURCE_DIR)/imu/imu.cpp -o $(BINARIES_DIR)/$@ 

bin_dir:
	mkdir -p $(BINARIES_DIR)

clean:
	$(info cleaning up workspace ...)
	rm -rf $(BINARIES_DIR) $(TARGET)
	cd test && make clean && cd -