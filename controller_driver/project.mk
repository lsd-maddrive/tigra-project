
PROJECT_MODULES = src/lld_brake_sensor.c 		\
				  src/brake_unit_cs.c 			\
				  src/lld_clutch_lever.c 		\
				  src/lld_control.c 			\
				  src/lld_wheel_pos_sensor.c 
				  
PROJECT_TESTS	= tests/test_brake_sensor.c 	\
				  tests/test_brake_unit_cs.c 	\
				  tests/test_clutch_lever.c 	\
				  tests/test_lld_control.c 		\
				  tests/test_wheel_pos_sensor.c


PROJECT_CSRC 	= src/main.c src/common.c \
					$(PROJECT_MODULES) $(PROJECT_TESTS)

PROJECT_CPPSRC 	= 

PROJECT_INCDIR	= include tests

PROJECT_LIBS	= -lm
