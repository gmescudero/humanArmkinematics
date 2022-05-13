# change application name here (executable output name)
TARGET = tests
# current directory
current_dir = $(shell pwd)
# compiler
CC = gcc
# source code directory
SOURCE_DIR = ../source
# binaries directory
BINARIES_DIR = ../bin
# includes
INC = \
	-I$(SOURCE_DIR)/general \
	-I$(SOURCE_DIR)/arm \
	-I$(SOURCE_DIR)/quat_lib \
	-I$(SOURCE_DIR)/math \
	-I$(SOURCE_DIR)/imu \
	-I$(SOURCE_DIR)/database \
	-Itst_lib
# libraries
IMULIB= -L../lib -lLpSensor -lstdc++ -lX11
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

# objects
# OBJS =  $(BINARIES_DIR)/quaternion.o $(BINARIES_DIR)/arm.o $(BINARIES_DIR)/vector3.o
OBJS = 	$(BINARIES_DIR)/arm.o  $(BINARIES_DIR)/database.o  $(BINARIES_DIR)/functions.o \
		$(BINARIES_DIR)/imu.o  $(BINARIES_DIR)/logging.o  $(BINARIES_DIR)/quaternion.o  $(BINARIES_DIR)/vector3.o

all: tst_lib.o tests.c $(OBJS)
	$(info building tests ... )
	$(CC) $(INC) $(OBJS) $(PTHREADSERIAL) $(IMULIB) tests.c tst_lib/*.o -o $(TARGET) -lm -ldl

tst_lib.o: tst_lib/tst_lib.c
	$(CC) $(INC) -c tst_lib/tst_lib.c -o tst_lib/$@ 

clean:
	rm -rf $(TARGET) $(LOGGING_DIR) $(DATA_DIR)
	rm -rf tst_lib/*.o