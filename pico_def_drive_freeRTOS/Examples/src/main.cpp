


#include <stdio.h>
#include <pico/stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "BlinkAgent.h"

#include "uRosBridge.h"

#include "MotorsAgent.h"

#include "Antonio.h"

extern"C"{

#include "pico/stdio_usb.h"
#include "pico/stdio_uart.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
//#include <pico_usb_transports.h>
}

//Standard Task priority
#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

//LED PAD to use
#define BLINK_LED_PAD	25
#define CONN_LED_PAD	3

//Left Motor
#define LEFT_CW		15
#define LEFT_CCW	14
#define LEFT_PWM 	13
#define LEFT_ROTENV	8

//Right Motor
#define RIGHT_CW	12
#define RIGHT_CCW	11
#define RIGHT_PWM 	10
#define RIGHT_ROTENV 9

//PID
#define KP	0.55
#define KI	0.019
#define KD	0.24
 
/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void *params){
    BlinkAgent blink(BLINK_LED_PAD);

    blink.start("Blink", TASK_PRIORITY);


    MotorsAgent motors(RIGHT_CW, RIGHT_CCW, RIGHT_PWM, LEFT_ROTENV);
	//motors.addMotor(0, LEFT_CW, LEFT_CCW, LEFT_PWM, LEFT_ROTENV);
	//motors.addMotor(1, RIGHT_CW, RIGHT_CCW, RIGHT_PWM, RIGHT_ROTENV);
	//motors.configAllPID(KP, KI, KD);
	motors.start("motors", TASK_PRIORITY);

    //Antonio 
    Antonio antonio;
    antonio.setMotorsAgent(&motors);
    antonio.start("Antonio", TASK_PRIORITY);
    
    //Start up a uROS Bridge
    uRosBridge *bridge = uRosBridge::getInstance();
    
    bridge->setuRosEntities(&antonio);
    bridge->setLed(CONN_LED_PAD);
    bridge->start("Bridge",  TASK_PRIORITY+2);

	for(;;){
    	vTaskDelay(10000);  
        }
}
 
void vLaunch( void) {  //Launch the tasks and scheduler


    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}



int main( void ){
	//Setup serial over UART and give a few seconds to settle before we start
    stdio_init_all();
    stdio_filter_driver(&stdio_uart);
    sleep_ms(2000);
    printf("GO\n");

    //Start tasks and scheduler
    const char *rtos_name = "FreeRTOS";
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();


    return 0;
}
