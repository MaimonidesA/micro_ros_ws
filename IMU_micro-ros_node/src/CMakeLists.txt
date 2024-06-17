add_executable(${ProjectName}
        main.cpp
		ICM20600.cpp
		AK09918.cpp
		SimpleKalmanFilter.cpp
		pico_uart_transport.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${ProjectName} 
	pico_stdlib
	hardware_gpio 
	hardware_pwm
	hardware_i2c
	micro_ros
	pico_multicore
	hardware_sync
	eigen
	)

# create map/bin/hex file etc.


# enable usb output, disable uart output
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

pico_add_extra_outputs(${ProjectName})