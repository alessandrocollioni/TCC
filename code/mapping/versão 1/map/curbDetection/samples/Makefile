################################################################################
#                           UNIVERSIDADE DE SAO PAULO                          #
#              INSTITUTO DE CIENCIAS MATEMATICAS E DE COMPUTACAO               #
#------------------------------------------------------------------------------#
# AUTOR:  HEITOR DE FREITAS VIEIRA                           Data: 19/05/2011  #
#------------------------------------------------------------------------------#
# Make File	                                                                   #
#------------------------------------------------------------------------------#
# TESTADO COM:                                                                 #
#           - UBUNTU 10.10 (64bits)                                            #
################################################################################


# COMPILADOR #
CC = g++

# FLAGS #
CFLAGS = -c -Wall -Wextra -march=native -O2
BFLAGS = -march=native -O2

# ARGUMENTOS #
PLAYER  = `pkg-config --cflags --libs playerc`
OPENMP  = -fopenmp

INCLUDE = -I../lib/include

all:
	make -C ../lib
	$(CC) $(CFLAGS) -o /tmp/sem_filtro.o sem_filtro.cpp  $(INCLUDE) $(PLAYER) $(OPENMP)
	$(CC) $(BFLAGS) -o sem_filtro /tmp/sem_filtro.o ../lib/objects/curbDetection.o ../lib/objects/means.o ../lib/objects/geometricFunctions.o $(PLAYER) $(OPENMP)
	
	$(CC) $(CFLAGS) -o /tmp/com_filtro.o com_filtro.cpp  $(INCLUDE) $(PLAYER) $(OPENMP)
	$(CC) $(BFLAGS) -o com_filtro /tmp/com_filtro.o ../lib/objects/curbDetection.o ../lib/objects/means.o ../lib/objects/geometricFunctions.o $(PLAYER) $(OPENMP)
#------------------------------------------------------------------------------#
# LIMPA OS RESIDUOS                                                            #
#------------------------------------------------------------------------------#  
clean:
	make clean -C ../lib
	find -name "*~" | xargs rm -rf
	rm -rf sem_filtro
	rm -rf com_filtro
	
.PHONY: clean
