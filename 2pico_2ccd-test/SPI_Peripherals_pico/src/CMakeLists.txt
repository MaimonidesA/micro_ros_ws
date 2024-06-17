  # if (TARGET tinyusb_device) 
    add_executable(${ProjectName}
        main.c
        )

    target_include_directories(${ProjectName} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

     target_link_libraries(${ProjectName} 
         pico_stdlib 
         hardware_pwm
         hardware_irq
         hardware_adc
         hardware_dma
         hardware_timer
         hardware_spi
         pico_binary_info
         )
#include(${CMAKE_CURRENT_SOURCE_DIR}/../../../../../cmale_Modules/ccd.cmake)


     pico_enable_stdio_usb(${ProjectName} 1) 
     pico_enable_stdio_uart(${ProjectName} 0) 

    pico_add_extra_outputs(${ProjectName})
#elseif(PICO_ON_DEVICE)
   # message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
#endif()
