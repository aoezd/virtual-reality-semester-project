CC		= g++
CPPFLAGS	= -g -W -Wall -Wextra --std=c++11
SRCS		= $(shell find . -type f -name '*.cpp')
HEDS		= $(shell find . -type f -name '*.hpp')
OBJS		= $(SRCS:%.cpp=%.o)
PROG		= mb-ar-app

OPENCV		= `pkg-config opencv --cflags --libs`
OPENGL		= `pkg-config gl glu --cflags --libs`

LDLIBS		= -lm  -lglut -lGLU -lGL 

.PHONY: clean all doc

$(PROG): .depend $(OBJS)
	$(CC) $(CPPFLAGS) -o $(PROG) $(OBJS) $(OPENCV) $(OPENGL) $(LDLIBS)

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
