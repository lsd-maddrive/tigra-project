PROJECT_MODULES = src/lld_break_sensor.c
PROJECT_TESTS	= tests/test_break_sensor.c

PROJECT_CSRC 	= src/main.c \
					$(PROJECT_MODULES) $(PROJECT_TESTS)

PROJECT_CPPSRC 	= 

PROJECT_INCDIR	= include tests

PROJECT_LIBS	=
