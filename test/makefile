# change application name here (executable output name)
TARGET = tests
# compiler
CC = gcc
# source code directory
SOURCE_DIR = ../source
# binaries directory
BINARIES_DIR = ../bin
# includes
INC = -I$(SOURCE_DIR)/dk_arm \
	-I$(SOURCE_DIR)/quat_lib \
	-I$(SOURCE_DIR)/math
# objects
OBJS =  $(BINARIES_DIR)/quaternion.o $(BINARIES_DIR)/directKin.o $(BINARIES_DIR)/vector3.o

all: tst_lib.o tests.c $(OBJS)
	$(info building tests ... )
	$(CC) $(INC) $(OBJS) tests.c tst_lib/*.o -o $(TARGET) -lm

tst_lib.o: tst_lib/tst_lib.c
	$(CC) -c tst_lib/tst_lib.c -o tst_lib/$@ 

clean:
	rm -rf $(TARGET)
	rm -rf tst_lib/*.o