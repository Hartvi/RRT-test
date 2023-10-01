
.SUFFIXES: .cc

CC = g++
CFLAGS  = -O2 -I. -I/usr/local/include/ -I../PQP/include/ -std=c++17 

CFLAGS += -Wmissing-include-dirs -Wall -Wextra 

CFLAGS += -I/usr/local/include/eigen3

LDFLAGS	= -L. -L/usr/local/lib/ -L../PQP/lib/ -lstdc++fs
LDLIBS  = -lPQP -lm

SRCS    = main.cc load.cc

OBJECTS	= main.o load.o

TARGET = load_obj_test

CLEAN   = $(OBJECTS) $(TARGET)

.cc.o:
	$(CC) ${CFLAGS} -c $<

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS) -L. $(LDFLAGS) $(LDLIBS)

run: $(TARGET)
	$(TARGET)

clean:
	/bin/rm -f $(CLEAN)

