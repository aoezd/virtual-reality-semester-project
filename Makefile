CC = g++
CPPFLAGS = -g -W -Wall -Wextra --std=c++11
SRCS = $(shell find . -type f -name '*.cpp')
HEDS = $(shell find . -type f -name '*.hpp')
OBJS = $(SRCS:%.cpp=%.o)
PROG = vr

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

.PHONY: clean all doc

$(PROG): .depend $(OBJS)
	$(CC) $(CPPFLAGS) -o $(PROG) $(OBJS) $(LIBS)

all: $(PROG)

clean: 
	rm -f $(PROG)
	rm -f $(OBJS)
	rm -f .depend

%.o : %.cpp
	$(CC) $(CPPFLAGS) -c $< -o $@

# Compute header file dependencies 
.depend : $(SRCS)
	$(CC) $(CPPFLAGS) -MM $^ > .depend

# check for undocumented Functions
# or generate Documentation
doc : Doxyfile $(SRCS)
	! ( cat Doxyfile ; echo "EXTRACT_ALL = NO" ) | doxygen - 2>&1 | grep -q "is not documented"
	doxygen

# Respect header file dependencies
include .depend
