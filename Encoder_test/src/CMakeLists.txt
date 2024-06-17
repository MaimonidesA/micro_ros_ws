add_executable(${ProjectName}
        main.cpp

        #pico_uart_transport.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${ProjectName} 
	pico_stdlib 

	hardware_pwm


	micro_ros
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(${ProjectName})

# enable usb output, disable uart output
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)
#add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
#add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)