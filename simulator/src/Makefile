#==============================================================================
# Epuck simulator Makefile
#==============================================================================

#~~~~~~~~~~~~~~~~~~
none:
	@echo
	@echo
	@echo "  sim_lin     - linux simulator (../bin/sim_lin)"
	@echo "  clean_lin   - cleans all object files"
	@echo "  clobber_lin - cleans all object files and linux targets"
	@echo
	@echo
#~~~~~~~~~~~~~~~~~~

#==============================================================================
# epuck Makefile - Linux
#==============================================================================


#--------------------------------------
# Macros
#--------------------------------------

#~~~~~~~~~~~~~~~~~~
# general
#~~~~~~~~~~~~~~~~~~
LIN_SRC_DIR         = .
LIN_LIB_DIR         = ../lib
LIN_BIN_DIR         = ../bin
LIN_INC_DIR         = ../include
LIN_CXX             = gcc
LIN_LXX             = ar cr
LIN_FLAGS           = \
  -O2 \
  -Wall \
  -g \
  -I$(LIN_INC_DIR)
LIN_LIBS            =
#~~~~~~~~~~~~~~~~~~

#~~~~~~~~~~~~~~~~~~
# sim_lin
#~~~~~~~~~~~~~~~~~~
SIM_LIN_TARGET      = $(LIN_BIN_DIR)/sim_lin
SIM_LIN_SRC_DIR     = $(LIN_SRC_DIR)/SimLin
SIM_LIN_SRC         = \
  $(SIM_LIN_SRC_DIR)/graphics.c \
  $(SIM_LIN_SRC_DIR)/epuck_serial.c \
  $(SIM_LIN_SRC_DIR)/sim.c \
  $(SIM_LIN_SRC_DIR)/robot.c \
  $(SIM_LIN_SRC_DIR)/user.c \
  $(SIM_LIN_SRC_DIR)/world.c \
  $(LIN_SRC_DIR)/UDPSocket/UDPSocket_C.c
SIM_LIN_FLAGS       = \
  $(LIN_FLAGS) \
  -I$(SIM_LIN_SRC_DIR)
SIM_LIN_LIBS        = \
  $(LIN_LIBS) \
  -L/usr/X11R6/lib \
  -lm \
  -lpthread \
  -lX11
SIM_LIN_OBJ         = \
  $(patsubst %.c,%_sim_lin.o,$(filter %.c,$(SIM_LIN_SRC))) \
  $(filter %.a,$(SIM_LIN_SRC))
#~~~~~~~~~~~~~~~~~~


#--------------------------------------
# Rules
#--------------------------------------

#~~~~~~~~~~~~~~~~~~
sim_lin: $(SIM_LIN_OBJ)
	$(LIN_CXX) $(SIM_LIN_FLAGS) $(SIM_LIN_OBJ) $(SIM_LIN_LIBS) -o $(SIM_LIN_TARGET)

%_sim_lin.o: %.c
	$(LIN_CXX) $(SIM_LIN_FLAGS) -o $@ -c $^
#~~~~~~~~~~~~~~~~~~

#~~~~~~~~~~~~~~~~~~
clean_lin:
	rm -f *.o $(SIM_LIN_SRC_DIR)/*.o UDPSocket/*.o
#~~~~~~~~~~~~~~~~~~

#~~~~~~~~~~~~~~~~~~
clobber_lin:
	rm -f *.o $(SIM_LIN_SRC_DIR)/*.o $(KH_LIN_TARGET) $(KH_SIM_LIN_TARGET) $(SIM_LIN_TARGET)
#~~~~~~~~~~~~~~~~~~


