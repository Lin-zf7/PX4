px4_add_module(
	MODULE modules__pwm_hw_out
	MAIN pwm_hw_out
	STACK_MAIN 2000
	SRCS
		pwm_hw_out.cpp
	DEPENDS
)
