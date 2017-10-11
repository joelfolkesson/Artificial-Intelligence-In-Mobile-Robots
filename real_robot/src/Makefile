#
# (c) 20016 Örebro University, Sweden
# Ali Abdul Khaliq
#

# Name of executable file
TARGET		:= ../lib/libepuck.a

# Toolchain (compiler, linker, ...)
CXX			:= g++

# Preparing files and flags

INC_DIR         := ../include
SRCS		:= $(wildcard *.c) $(wildcard *.cpp)
OBJS		:= $(SRCS:.c=.o) $(SRCS:.cpp=.o) 
MODS		:=
CFLAGS 		:= -I../include/
CXXFLAGS 	:= -I../include/
LIBS		:=
DEPS            := \
  $(INC_DIR)/epuck.hpp\
  $(INC_DIR)/epuckCamera.hpp\
  $(INC_DIR)/epuckInterface.hpp\
  $(INC_DIR)/epuckIR.hpp\
  $(INC_DIR)/epuckLEDs.hpp\
  $(INC_DIR)/epuckPosition2d.hpp\
  $(INC_DIR)/fuzzy.h\
  $(INC_DIR)/interface.h\
  $(INC_DIR)/lists.h\
  $(INC_DIR)/maps.h\
  $(INC_DIR)/rfid.hpp\
  $(INC_DIR)/serialPort.hpp\
  $(INC_DIR)/timer.hpp


$(TARGET): $(OBJS) $(MODS)
	@echo "=== Building $@"
	ar rsc $@ $(OBJS)
#	$(CXX) -o $@ $(OBJS) $(INCS) $(DEPS) $(CFLAGS) $(CXXFLAGS)
	@echo "=== Building $@"

.PHONY: all
all: $(TARGET)

.PHONY: start
start: all
	./$(TARGET)

.PHONY: debug
debug: all
	gdb $(TARGET)

.PHONY: clean
clean:
	@echo "=== Cleaning"
	rm -f *.o .depend $(TARGET) *~


ifeq (.depend,$(wildcard .depend))
include .depend 
endif
