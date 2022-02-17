# change application name here (executable output name)
TARGET = app
# current directory
current_dir = $(shell pwd)
$(info $(current_dir))
# source code directory
SOURCE_DIR = $(current_dir)/source
# binaries directory
BINARIES_DIR = $(current_dir)/bin
# test directory
TEST_DIR = $(current_dir)/test
TESTS_IMPLEMENTATION_DIR = $(TEST_DIR)/tests_implementation
# get test files
TESTS = $(shell  ls $(TESTS_IMPLEMENTATION_DIR)) 

# compiler
CC = gcc
# includes
INC = -I$(SOURCE_DIR)/dk_arm \
	-I$(SOURCE_DIR)/quat_lib \
	-I$(SOURCE_DIR)/math

# debug
DEBUG = -g
# optimisation
OPT = -O0
# warnings
WARN = -Wall

CPPFLAGS = $(DEBUG) $(OPT) $(WARN)

OBJS =  quaternion.o directKin.o vector3.o

all: $(OBJS) main.o
	$(info building target ...)
	$(CC) $(INC) $(BINARIES_DIR)/*.o -o $(TARGET) -lm

tests: $(OBJS) $(TESTS_IMPLEMENTATION_DIR)/test001.c
	$(info building tests ...)
	$(CC) $(INC) $(BINARIES_DIR)/*.o $(TESTS_IMPLEMENTATION_DIR)/test001.c -o $(TEST_DIR)/test001 -lm

main.o: $(SOURCE_DIR)/main.c
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/main.c -o $(BINARIES_DIR)/$@

quaternion.o: $(SOURCE_DIR)/quat_lib/Quaternion.c 
	$(CC) -c  $(CPPFLAGS) $(SOURCE_DIR)/quat_lib/Quaternion.c -o $(BINARIES_DIR)/$@ 

directKin.o: $(SOURCE_DIR)/dk_arm/directKinematics.c vector3.o
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/dk_arm/directKinematics.c -o $(BINARIES_DIR)/$@ 

vector3.o: $(SOURCE_DIR)/math/vector3.c 
	$(CC) -c  $(CPPFLAGS) $(SOURCE_DIR)/math/vector3.c -o $(BINARIES_DIR)/$@ 

clean:
	$(info cleaning up workspace ...)
	rm -rf $(BINARIES_DIR)/* $(TARGET)