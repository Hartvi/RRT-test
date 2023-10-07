.SUFFIXES: .cc

CC = g++
CFLAGS  = -O2 -I. -I/usr/local/include/ -I../PQP/include/ -std=c++17 
CFLAGS += -Wmissing-include-dirs -Wall -Wextra -Wpedantic
CFLAGS += -I/usr/local/include/eigen3

LDFLAGS	= -L. -L/usr/local/lib/ -L../PQP/lib/ -lstdc++fs
LDLIBS  = -lPQP -lm -lflann -llz4

SRCS    = main.cc load.cc model.cc
OBJECTS	= main.o load.o model.o

SHARED_OBJECTS = load.o model.o
SHARED_TARGET = libmyshared.so

TARGET = load_obj_test

CLEAN   = $(OBJECTS) $(TARGET) $(SHARED_TARGET)

.cc.o:
	$(CC) ${CFLAGS} -fPIC -c $<   # Add -fPIC here for position independent code

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) -L. $(LDFLAGS) $(LDLIBS)

shared: $(SHARED_OBJECTS)       # New target for creating a shared library
	$(CC) -shared -o $(SHARED_TARGET) $(SHARED_OBJECTS) $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)
