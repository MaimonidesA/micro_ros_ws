
include_directories(include)

add_executable(${ProjectName}
    main.cpp
    Antonio.cpp
    
    BlinkAgent.cpp
    MotorsAgent.cpp
    MotorMgr.cpp

    Agent.cpp
    uRosBridge.cpp
    uRosEntities.cpp
    pico_usb_transport.c
    freertos_allocators.c
)

target_include_directories(${ProjectName} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    include   
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    pico_multicore
    FreeRTOS-Kernel-Heap4 
    micro_ros
    pico_rand
    hardware_pwm
    )
pico_add_extra_outputs(${ProjectName})

pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 1)
target_compile_definitions(${ProjectName} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=16
    PICO_DEFAULT_UART_TX_PIN=17
)

