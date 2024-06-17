#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <queue.h>
#include "main.h"



#include <math.h>
#include <stdio.h>

#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#include "ICM20600.h"

extern "C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "pico/multicore.h"

#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
}

#define timestamp float((int)time_us_32())

//Left Motor
#define LEFT_CW		    15
#define LEFT_CCW	    14
#define LEFT_PWM 	    13
#define LEFT_Encoder    20

//Right Motor
#define RIGHT_CW	    12
#define RIGHT_CCW	    11
#define RIGHT_PWM 	    10
#define RIGHT_Encoder 	21






const uint LED_PIN = 25;
// freertos Types.
QueueHandle_t irq_queue = NULL;
static QueueHandle_t xQueue = NULL;

// ros Types
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_publisher_t 		PubOdom;
nav_msgs__msg__Odometry OdomMsg;

SemaphoreHandle_t mutex;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist Twist_msg;

double Linear_speed_request;
double  Angular_speed_request;

int Ticks_per_cycle = 275;
int Wheel_Radius = 36;  // Units mm 
float Wheel_Circumference = 2*M_PI*Wheel_Radius; // Units mm
int Distance_between_wheels = 236; // Units mm

float Left_RPM;
float Left_velocity;
float Left_Encoder_Delta_Time = 0;
float Left_Encoder_priv_Time = 0;

float Right_RPM;
float Right_velocity;
float Right_Encoder_Delta_Time;
float Right_Encoder_priv_Time = 0;

struct Velocity_t {
   float Linear;
   float angular;
} Velocity;
//typedef struct Velocity_t Velocity;



void Left_Encoder_callback(uint gpio, uint32_t event){
  //  LEFT_enable_irq(false);
    Left_Encoder_Delta_Time = timestamp - Left_Encoder_priv_Time;
    Left_Encoder_priv_Time = timestamp;

}

void Right_Encoder_callback(uint gpio, uint32_t event){
   // Right_enable_irq(false);
    Right_Encoder_Delta_Time = timestamp - Right_Encoder_priv_Time;
    Right_Encoder_priv_Time = timestamp;
  //  xQueueSendToBackFromISR(irq_queue, &state, 0);
}

void LEFT_enable_irq(bool state){
    gpio_set_irq_enabled_with_callback(LEFT_Encoder,  GPIO_IRQ_LEVEL_LOW, state, &Left_Encoder_callback);
}

void Right_enable_irq(bool state){
    gpio_set_irq_enabled_with_callback(LEFT_Encoder,  GPIO_IRQ_LEVEL_LOW, state, &Left_Encoder_callback);
}


Velocity_t Robot_Velocity(float Left_Encoder_Delta_Time, float Right_Encoder_Delta_Time) {
    
    Left_velocity = (Wheel_Circumference / Ticks_per_cycle / 1000) / (Left_Encoder_Delta_Time / 1000000); // Units m/s
    if (!gpio_get(LEFT_CW)){Left_velocity  *= (-1);}
   // LEFT_enable_irq(true);
    Right_velocity = (Wheel_Circumference / Ticks_per_cycle / 1000) / (Right_Encoder_Delta_Time / 1000000); // Units m/s
    if (!gpio_get(RIGHT_CW)){Right_velocity *=  (-1);}
   // Right_enable_irq(true);
   if (Right_Encoder_Delta_Time == 0) {Right_velocity = 0;}
   if (Left_Encoder_Delta_Time == 0) {Left_velocity = 0;}

    Velocity.Linear = Left_velocity / 2 + Right_velocity / 2;
    Velocity.angular = Left_velocity / 2 * M_PI * Distance_between_wheels - Right_velocity / 2 * M_PI * Distance_between_wheels; //  Units radians/second.

    return Velocity;
}

void PID() {
    // todo
}


void IMU_init(ICM20600 &icm20600){
     ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;
    
    icm20600.initialize();
    icm20600.setPowerMode(ICM_6AXIS_LOW_NOISE);


}

void IMU_val(void *params){
    ICM20600 icm20600(true);
    float Linear_velocity;
    while (1)
    {
      int gyro_Z  = icm20600.getGyroscopeZ();
      int acceleration_X = icm20600.getAccelerationX();
      float acceleration_sum = 0;
      int Average = 100;
      
      int Previous_time = time_us_32();

      for (size_t i = 0; i < Average; i++)
      {
        acceleration_sum = acceleration_sum + icm20600.getAccelerationX();
      }
      int Second_time = time_us_32();
      Linear_velocity =(Second_time - Previous_time) * (acceleration_sum / 100);
      
     // printf("velocity_x : %f" ,Linear_velocity);
      
    }
    

}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;

    rcl_ret_t ret_odom = rcl_publish(&PubOdom, &OdomMsg, NULL); 
}

void subscription_callback(const void * msgin)
{

  // Cast received message to used type
  const geometry_msgs__msg__Twist * pTwistMsg = (const geometry_msgs__msg__Twist *) msgin;

    Linear_speed_request =  pTwistMsg->linear.x; 
    Angular_speed_request =  pTwistMsg->angular.z;
    double Right_throttle;
    double Left_throttle;

	//if (Linear_speed_request < -1.0 ) { Linear_speed_request = -1.0; }
   // if (Linear_speed_request > 1.0 ) { Linear_speed_request = 1.0; }

    //if (Angular_speed_request < -1.0 ) { Angular_speed_request = -1.0; }
    //if (Angular_speed_request > 1.0 ) { Angular_speed_request = 1.0; }

     if (Angular_speed_request == 0.0 )
    {
       Right_throttle = Linear_speed_request;
       Left_throttle = Linear_speed_request ;
    }
   
    if (Angular_speed_request < 0.0 ) 
    {
        Right_throttle = ((Linear_speed_request) - abs(Angular_speed_request));
        Left_throttle = ((Linear_speed_request) + abs(Angular_speed_request));
    }

    else  
    {
        Right_throttle = ((Linear_speed_request) + abs(Angular_speed_request));
        Left_throttle = ((Linear_speed_request) - abs(Angular_speed_request));
    }

   


//Left Motor
    int left_pwm = (int)((float)(0xffff) * abs(Left_throttle));
    pwm_set_gpio_level(LEFT_PWM, left_pwm);
	if (Left_throttle < 0 ){
		gpio_put(LEFT_CW, 1);
		gpio_put(LEFT_CCW, 0);
	}
    else// run counter clockwise
    {		
		gpio_put(LEFT_CW, 0);
		gpio_put(LEFT_CCW, 1);
    }
//Right Motor
    int right_pwm = (int)((float)(0xffff) * abs(Right_throttle));
    pwm_set_gpio_level(RIGHT_PWM, right_pwm);
	if (Right_throttle < 0 ){
		gpio_put(RIGHT_CW, 1);
		gpio_put(RIGHT_CCW, 0);
	}
    else // run counter clockwise
    {		
		gpio_put(RIGHT_CW, 0);
		gpio_put(RIGHT_CCW, 1);
    }


}

nav_msgs__msg__Odometry update_odom(){
     //TWIST
	OdomMsg.twist.twist.linear.x = Velocity.Linear;

	OdomMsg.twist.twist.angular.z = Velocity.angular;

    return OdomMsg;
}

void  setupOdomMsg(){
    nav_msgs__msg__Odometry__init(&OdomMsg);

    //POSE
	OdomMsg.pose.pose.position.x = 0.0;
	OdomMsg.pose.pose.position.y = 0.0;
	OdomMsg.pose.pose.position.z = 0.0;
	OdomMsg.pose.pose.orientation.x = 0.0;
	OdomMsg.pose.pose.orientation.y = 0.0;
	OdomMsg.pose.pose.orientation.z = 0.0;
	OdomMsg.pose.pose.orientation.w = 0.0;
	
    //TWIST
	OdomMsg.twist.twist.linear.x = 0.0;
	OdomMsg.twist.twist.linear.y = 0.0;
	OdomMsg.twist.twist.linear.z = 0.0;
	OdomMsg.twist.twist.angular.x = 0.0;
	OdomMsg.twist.twist.angular.y = 0.0;
	OdomMsg.twist.twist.angular.z = 0.0;
    
}

void Motor_init(){

    //Left Motor
    gpio_init(LEFT_PWM);
    gpio_set_function(LEFT_PWM, GPIO_FUNC_PWM);
    pwm_set_gpio_level(LEFT_PWM, 0);
    uint slice_num_LEFT = pwm_gpio_to_slice_num(LEFT_PWM);
    pwm_set_enabled(slice_num_LEFT, true);

    gpio_init(LEFT_CW);
    gpio_set_dir(LEFT_CW, GPIO_OUT);
    gpio_init(LEFT_CCW);
    gpio_set_dir(LEFT_CCW, GPIO_OUT);
    gpio_pull_up (LEFT_Encoder);
    gpio_set_dir(LEFT_Encoder, GPIO_IN);

    //Right Motor
    gpio_init(RIGHT_PWM);
    gpio_set_function(RIGHT_PWM, GPIO_FUNC_PWM);
    pwm_set_gpio_level(RIGHT_PWM, 0);
    uint slice_num_RIGHT = pwm_gpio_to_slice_num(RIGHT_PWM);
    pwm_set_enabled(slice_num_RIGHT, true);

    gpio_init(RIGHT_CW);
    gpio_set_dir(RIGHT_CW, GPIO_OUT);
    gpio_init(RIGHT_CCW);
    gpio_set_dir(RIGHT_CCW, GPIO_OUT);
    gpio_pull_up (RIGHT_Encoder);
    gpio_set_dir(RIGHT_Encoder, GPIO_IN);

}

void Encoder_tics_task(void *pvParameters)
{ 
    vTaskDelay(10000);  
    float Send_delta_RIGHT_Encoder = 0;
    bool RIGHT_Encoder_gpio = false;
    while (true) {
        
        if (gpio_get(RIGHT_Encoder) && !RIGHT_Encoder_gpio){
            Left_Encoder_Delta_Time = timestamp - Left_Encoder_priv_Time;
            Left_Encoder_priv_Time = timestamp;
            Send_delta_RIGHT_Encoder = Left_Encoder_Delta_Time;
             xQueueSend(xQueue, &Send_delta_RIGHT_Encoder, 0U);
            RIGHT_Encoder_gpio = true;
        }
        else if (!gpio_get(RIGHT_Encoder))
        {
             RIGHT_Encoder_gpio = false;
        }

    }
}

void Motor_handle(void *pvParameters){
    vTaskDelay(10000);
    float Received_delta_RIGHT_Encoder;

    for (;;) {
        xQueueReceive(xQueue, &Received_delta_RIGHT_Encoder, portMAX_DELAY);

         Robot_Velocity(Left_Encoder_Delta_Time, Received_delta_RIGHT_Encoder);
         OdomMsg = update_odom();
        vTaskDelay(1);
    }
}

void ROS_com(void *params){

 if(xSemaphoreTake(mutex, 0) == pdTRUE){
            xSemaphoreGive(mutex);
    }

    rclc_executor_t executor;
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;

   //geometry_msgs__msg__Twist__init(&Twist_msg);
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {    // Unreachable agent, exiting program.
       // return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_publisher_init_default(
        &PubOdom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");

    rcl_ret_t rc = rclc_subscription_init_default(
         &subscriber,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
         "/cmd_vel");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    //* Add subscription to the executor
    rc = rclc_executor_add_subscription(&executor, &subscriber, &Twist_msg,
                                        &subscription_callback, ON_NEW_DATA);
    gpio_put(LED_PIN, 1);
    msg.data = 0;

    bool RIGHT_Encoder_gpio = false;
    bool Left_Encoder_gpio = false;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

        if (gpio_get(RIGHT_Encoder) != 0 && !RIGHT_Encoder_gpio){
            Right_Encoder_Delta_Time = timestamp - Right_Encoder_priv_Time;
            Right_Encoder_priv_Time = timestamp;
            RIGHT_Encoder_gpio = true;
        }
        else if (gpio_get(RIGHT_Encoder) == 0)
        {
             RIGHT_Encoder_gpio = false;
        }

        if (gpio_get(LEFT_Encoder) != 0 && !RIGHT_Encoder_gpio){
            Left_Encoder_Delta_Time = timestamp - Left_Encoder_priv_Time;
            Left_Encoder_priv_Time = timestamp;
            Left_Encoder_gpio = true;
        }
        else if (gpio_get(RIGHT_Encoder) == 0)
        {
             Left_Encoder_gpio = false;
        }

        Robot_Velocity(Left_Encoder_Delta_Time, Right_Encoder_Delta_Time);
        OdomMsg = update_odom();
    }  //return 0;
}

int main()
{
    stdio_init_all();
    Motor_init();
    setupOdomMsg();
    // Define the task handles
    TaskHandle_t handleA;
    TaskHandle_t handleB;
    mutex = xSemaphoreCreateMutex();

    xTaskCreate(ROS_com, "ROS_com", 1024, NULL, 5, NULL);
    //xTaskCreate(Motor_handle, "Motor_handle", 512, NULL, 5, NULL);
    //xTaskCreate(Encoder_tics_task, "Encoder_tics_task", 256, NULL, 5, NULL);

     irq_queue = xQueueCreate(1, sizeof(uint8_t));

      // Pin Tasks
 
   // vTaskCoreAffinitySet(handleA, (1 << 0)); // Core 0
   // vTaskCoreAffinitySet(handleB, (1 << 1)); // Core 1

    vTaskStartScheduler(); 
   
}