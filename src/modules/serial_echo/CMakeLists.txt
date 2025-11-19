px4_add_module(
    MODULE modules__serial_echo
    MAIN serial_echo
	STACK_MAIN 2000
	SRCS
		serial_echo.cpp
	DEPENDS
)
