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

all: main.o quaternion.o directKin.o
	$(info building target ...)
	$(CC) $(INC) $(BINARIES_DIR)/*.o -o $(TARGET) -lm

main.o: $(SOURCE_DIR)/main.c
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/main.c -o $(BINARIES_DIR)/$@

quaternion.o: $(SOURCE_DIR)/quat_lib/Quaternion.c 
	$(CC) -c  $(CPPFLAGS) $(SOURCE_DIR)/quat_lib/Quaternion.c -o $(BINARIES_DIR)/$@ 

directKin.o: $(SOURCE_DIR)/dk_arm/directKinematics.c 
	$(CC) -c  $(CPPFLAGS) $(INC) $(SOURCE_DIR)/dk_arm/directKinematics.c -o $(BINARIES_DIR)/$@ 

clean:
	$(info cleaning up workspace ...)
	rm -rf $(BINARIES_DIR)/* $(TARGET)