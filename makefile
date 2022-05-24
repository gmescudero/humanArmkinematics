# change application name here (executable output name)
TARGET = app
# current directory
current_dir = $(shell pwd)
$(info $(current_dir))
# source code directory
SOURCE_DIR = $(current_dir)/source
# binaries directory
BINARIES_DIR = $(current_dir)/bin
# log directories
LOGGING_DIR = $(current_dir)/log
DATA_DIR    = $(current_dir)/data
# compiler
CC = gcc
LD = g++
# includes
INC = \
	-I$(SOURCE_DIR)/general \
	-I$(SOURCE_DIR)/arm \
	-I$(SOURCE_DIR)/quat_lib \
	-I$(SOURCE_DIR)/math \
	-I$(SOURCE_DIR)/imu \
	-I$(SOURCE_DIR)/database \
	-I$(SOURCE_DIR)/calibration
	
# libraries
IMULIB= -L$(current_dir)/lib -lLpSensor -lstdc++ -lX11
PTHREAD = -pthread
PTHREADSERIAL = $(PTHREAD) -lserialport
# debug
DEBUG = -g
# optimisation
OPT = -O0
# warnings
WARN = -Wall
# flags
CCFLAGS = $(DEBUG) $(OPT) $(WARN)
CPPFLAGS = $(DEBUG) $(OPT) $(WARN)

OBJS =  quaternion.o arm.o vector3.o imu.o functions.o logging.o database.o calib.o

all: $(OBJS) main.o
	$(info building target ...)
	$(LD) $(INC) $(BINARIES_DIR)/*.o $(IMULIB) $(PTHREADSERIAL) -o $(TARGET) -lm -ldl

test: $(OBJS) all
	cd test && make && cd -
	./test/tests

main.o: $(SOURCE_DIR)/main.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/main.c -o $(BINARIES_DIR)/$@

functions.o: $(SOURCE_DIR)/general/functions.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/general/functions.c -o $(BINARIES_DIR)/$@ 

logging.o: $(SOURCE_DIR)/general/logging.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/general/logging.c -o $(BINARIES_DIR)/$@ 

quaternion.o: $(SOURCE_DIR)/quat_lib/Quaternion.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/quat_lib/Quaternion.c -o $(BINARIES_DIR)/$@

arm.o: $(SOURCE_DIR)/arm/arm.c vector3.o dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/arm/arm.c -o $(BINARIES_DIR)/$@ 

calib.o: $(SOURCE_DIR)/calibration/calib.c vector3.o dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/calibration/calib.c -o $(BINARIES_DIR)/$@ 

vector3.o: $(SOURCE_DIR)/math/vector3.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/math/vector3.c -o $(BINARIES_DIR)/$@ 

imu.o: $(SOURCE_DIR)/imu/imu.cpp dirs_create
	$(LD) -c $(CCFLAGS) $(INC) $(SOURCE_DIR)/imu/imu.cpp -o $(BINARIES_DIR)/$@ 

database.o: $(SOURCE_DIR)/database/database.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/database/database.c -o $(BINARIES_DIR)/$@ 

dirs_create:
	mkdir -p $(BINARIES_DIR)
	mkdir -p $(LOGGING_DIR)
	mkdir -p $(DATA_DIR)

clean:
	$(info cleaning up workspace ...)
	rm -rf $(BINARIES_DIR) $(LOGGING_DIR) $(DATA_DIR) $(TARGET)
	cd test && make clean && cd -