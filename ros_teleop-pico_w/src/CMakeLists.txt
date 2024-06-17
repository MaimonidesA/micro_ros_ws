add_executable(${ProjectName}
        main.c
        picow_udp_transports.c
        )
target_include_directories(${ProjectName} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_compile_definitions(${ProjectName} PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${ProjectName} 
	pico_stdlib 
	hardware_pwm
	hardware_i2c
	micro_ros
	hardware_dma
	hardware_adc
	hardware_gpio
	pico_multicore
	hardware_sync
	pico_cyw43_arch_lwip_threadsafe_background
	#pico_cyw43_arch_lwip_poll
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(${ProjectName})

# enable usb output, disable uart output
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)



add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)