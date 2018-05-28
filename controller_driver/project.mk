
PROJECT_MODULES = lld_brake_sensor.c 		\
				  brake_unit_cs.c 			\
				  lld_clutch_lever.c 		\
				  lld_control.c 			\
				  lld_wheel_pos_sensor.c 
				  
PROJECT_MODULES := $(addprefix src/,$(PROJECT_MODULES))
				  
PROJECT_TESTS	= test_brake_sensor.c 		\
				  test_brake_unit_cs.c 		\
				  test_clutch_lever.c 		\
				  test_lld_control.c 		\
				  test_wheel_pos_sensor.c

PROJECT_TESTS := $(addprefix tests/,$(PROJECT_TESTS))

PROJECT_CSRC 	= src/main.c src/common.c \
					$(PROJECT_MODULES) $(PROJECT_TESTS)

PROJECT_CPPSRC 	= 

PROJECT_INCDIR	= include tests

PROJECT_LIBS	= -lm
