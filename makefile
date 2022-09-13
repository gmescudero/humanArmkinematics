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
	-I$(SOURCE_DIR)/calibration \
	-I$(SOURCE_DIR)/libGA100 \
	-I$(SOURCE_DIR)/launcher
	
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

OBJS =  quaternion.o arm.o vector3.o matrix.o imu.o functions.o logging.o database.o calib.o calib_two_axes_ga.o calib_two_axes_gn.o libGA.o svd.o boot.o

all: $(OBJS) main.o
	$(info building target ...)
	$(LD) $(INC) $(BINARIES_DIR)/*.o $(IMULIB) $(PTHREADSERIAL) -o $(TARGET) -lm -ldl

test: $(OBJS) all
	cd test && make && cd -
	./test/tests

test_nl: $(OBJS) all
	cd test && make && cd -

main.o: $(SOURCE_DIR)/main.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/main.c -o $(BINARIES_DIR)/$@

boot.o: $(SOURCE_DIR)/launcher/boot.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/launcher/boot.c -o $(BINARIES_DIR)/$@

functions.o: $(SOURCE_DIR)/general/functions.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/general/functions.c -o $(BINARIES_DIR)/$@ 

logging.o: $(SOURCE_DIR)/general/logging.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/general/logging.c -o $(BINARIES_DIR)/$@ 

quaternion.o: $(SOURCE_DIR)/quat_lib/Quaternion.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/quat_lib/Quaternion.c -o $(BINARIES_DIR)/$@

arm.o: $(SOURCE_DIR)/arm/arm.c vector3.o dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/arm/arm.c -o $(BINARIES_DIR)/$@ 

calib.o: $(SOURCE_DIR)/calibration/calib.c vector3.o libGA.o dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/calibration/calib.c -o $(BINARIES_DIR)/$@ 

calib_two_axes_gn.o: $(SOURCE_DIR)/calibration/calib_two_axes_gn.c vector3.o dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/calibration/calib_two_axes_gn.c -o $(BINARIES_DIR)/$@ 

calib_two_axes_ga.o: $(SOURCE_DIR)/calibration/calib_two_axes_ga.c libGA.o vector3.o dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/calibration/calib_two_axes_ga.c -o $(BINARIES_DIR)/$@ 

vector3.o: $(SOURCE_DIR)/math/vector3.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/math/vector3.c -o $(BINARIES_DIR)/$@ 

matrix.o: $(SOURCE_DIR)/math/matrix.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/math/matrix.c -o $(BINARIES_DIR)/$@ 

svd.o: $(SOURCE_DIR)/math/singular_value_decomposition.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/math/singular_value_decomposition.c -o $(BINARIES_DIR)/$@ 

imu.o: $(SOURCE_DIR)/imu/imu.cpp dirs_create
	$(LD) -c $(CCFLAGS) $(INC) $(SOURCE_DIR)/imu/imu.cpp -o $(BINARIES_DIR)/$@ 

database.o: $(SOURCE_DIR)/database/database.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/database/database.c -o $(BINARIES_DIR)/$@ 

libGA.o: $(SOURCE_DIR)/libGA100/libgaALL.c dirs_create
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/libGA100/libgaALL.c -o $(BINARIES_DIR)/$@ 


dirs_create:
	mkdir -p $(BINARIES_DIR)
	mkdir -p $(LOGGING_DIR)
	mkdir -p $(DATA_DIR)

clean:
	$(info cleaning up workspace ...)
	rm -rf $(BINARIES_DIR) $(LOGGING_DIR) $(DATA_DIR) $(TARGET)
	cd test && make clean && cd -