#if (TARGET tinyusb_device) 
add_executable(${ProjectName}
    main.c
    pico_uart_transport.c
    )
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    )
    
target_link_libraries(${ProjectName} 
    pico_stdlib 
    FreeRTOS-Kernel-Heap4 
    hardware_pwm
    hardware_irq
    hardware_adc
    hardware_dma
    hardware_timer
    micro_ros
   
    )
pico_add_extra_outputs(${ProjectName})


# enable usb output, disable uart output
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)