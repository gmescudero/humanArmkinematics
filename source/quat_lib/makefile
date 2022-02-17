# change application name here (executable output name)
TARGET = app
# current directory
current_dir = $(shell pwd)
$(info $(current_dir))
# source code directory
SOURCE_DIR = .

# compiler
CC = gcc

# debug
DEBUG = -g
# optimisation
OPT = -O0
# warnings
WARN = -Wall

CPPFLAGS = $(DEBUG) $(OPT) $(WARN)

all: test.o quaternion.o
	$(info building target ...)
	$(CC) test.o quaternion.o -o $(TARGET) -lm

test.o: TestQuaternion.c
	$(CC) -c  $(CPPFLAGS) TestQuaternion.c -o $@

quaternion.o: Quaternion.c 
	$(CC) -c  $(CPPFLAGS) Quaternion.c -o $@

clean:
	$(info cleaning up workspace ...)
	rm -f *.o $(TARGET)
