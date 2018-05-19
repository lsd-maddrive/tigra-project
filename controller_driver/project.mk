PROJECT_MODULES = src/lld_break_sensor.c \
				  src/lld_clutch_lever.c
				  
PROJECT_TESTS	= tests/test_break_sensor.c \
				  tests/test_clutch_lever.c

PROJECT_CSRC 	= src/main.c src/common.c \
					$(PROJECT_MODULES) $(PROJECT_TESTS)

PROJECT_CPPSRC 	= 

PROJECT_INCDIR	= include tests

PROJECT_LIBS	= -lm
