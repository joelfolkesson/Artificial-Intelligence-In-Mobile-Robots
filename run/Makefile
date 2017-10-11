#
# (c) 20016 Örebro University, Sweden
# Ali Abdul Khaliq
#

none:
	@echo
	@echo "Choose an option to make (make real or make sim)"
	@echo

# Toolchain (compiler, linker, ...)
CC			:= g++

# Preparing files and flags
SRCS		:= main.c
OBJS		:= $(SRCS:.c=.o)
MODS		:=

CFLAGS 		:= -I../real_robot/include
FLAGS           := -I../simulator/include
LIN_FLAGS       := -Wall

INCS		:= 
LIBS		:= -lm
LIB_EPK		:= ../real_robot/lib/libepuck.a

INCLUDE=\
  ../real_robot/include/interface.h

SIM_DIR_C        := ../simulator/src
SIM_DIR_INC        := ../simulator/include

SIM_SRC=\
  $(SIM_DIR_C)/epuck.c\
  $(SIM_DIR_C)/fuzzy.c\
  $(SIM_DIR_C)/maps.c\
  $(SIM_DIR_C)/lists.c\
  $(SIM_DIR_C)/epuck_linux.c\
  $(SIM_DIR_C)/epuck_sim_linux.c\
  $(SIM_DIR_C)/UDPSocket/UDPSocket_C.c\
  $(SIM_DIR_INC)/epuck.h\
  $(SIM_DIR_INC)/interface.h\
  $(SIM_DIR_INC)/fuzzy.h\
  $(SIM_DIR_INC)/maps.h\
  $(SIM_DIR_INC)/lists.h


#--------------------------------------
# Rules for real robot
#--------------------------------------
real: $(OBJS) $(MODS)
	@echo "=== Building for real robot $@"
	$(CC) -o $@ $(OBJS) $(MODS) $(INCLUDE) $(LIBS) $(LIB_EPK) $(CFLAGS)

#~~~~~~~~~~~~~~~~~~
clean_real:
	rm -f *.o real
#~~~~~~~~~~~~~~~~~~


#--------------------------------------
# Rules for simulation
#--------------------------------------

sim: $(SRCS) $(SIM_SRC)
	@echo "=== Building for simularion $@"
	gcc -o $@ $(FLAGS) $(LIN_FLAGS) $(SRCS) $(SIM_SRC) $(LIBS)

#~~~~~~~~~~~~~~~~~~
clean_sim:
	rm -f *.o sim
#~~~~~~~~~~~~~~~~~~

