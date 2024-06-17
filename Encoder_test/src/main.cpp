#include <stdio.h>
#include "hardware/pwm.h"
#include <math.h>
#include <time.h>
extern "C"{
#include "rcl/rcl.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
}
// === the fixed point macros (16.15) ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

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

double Linear_speed_request;
double  Angular_speed_request;

int Ticks_per_cycle = 275;
int Wheel_Radius = 36;  // Units mm 
float Wheel_Circumference = 2*M_PI*Wheel_Radius; // Units mm
int Distance_between_wheels = 236; // Units mm

float Left_RPM;
float Left_velocity;
float Left_Encoder_Delta_Time;
float Left_Encoder_priv_Time = 0;

float Right_RPM;
float Right_velocity;
float Right_Encoder_Delta_Time = 0;
float Right_Encoder_priv_Time = 0;

struct Velocity_t {
   float Linear;
   float angular;
} Velocity;


void Left_Encoder_callback(uint gpio, uint32_t event){
   // LEFT_enable_irq(false);
    Left_Encoder_Delta_Time = (timestamp - Left_Encoder_priv_Time);
    Left_Encoder_priv_Time = timestamp;

}

void Right_Encoder_callback(uint gpio, uint32_t event){
    //Right_enable_irq(false);
    Right_Encoder_Delta_Time = (timestamp - Right_Encoder_priv_Time);
    Right_Encoder_priv_Time = timestamp;

}

void LEFT_enable_irq(bool state){
    gpio_set_irq_enabled_with_callback(LEFT_Encoder,  GPIO_IRQ_LEVEL_LOW, state, &Left_Encoder_callback);
}

void Right_enable_irq(bool state){
    gpio_set_irq_enabled_with_callback(LEFT_Encoder,  GPIO_IRQ_LEVEL_LOW, state, &Left_Encoder_callback);
}


Velocity_t Robot_Velocity(float Left_Encoder_Delta_Time, float Right_Encoder_Delta_Time) {
    
    Left_velocity = (Left_Encoder_Delta_Time / 1000000) * (Wheel_Circumference / Ticks_per_cycle / 1000); // Units m/s
    if (!gpio_get(LEFT_CW)){Left_velocity  *= (-1);}
   // LEFT_enable_irq(true);
    Right_velocity =   (Wheel_Circumference / Ticks_per_cycle / 1000) / (Right_Encoder_Delta_Time / 1000000); // Units m/s
    if (gpio_get(RIGHT_CW)){Right_velocity *=  (-1);}
   // Right_enable_irq(true);
    Velocity.Linear = Left_velocity / 2 + Right_velocity / 2;
    Velocity.angular = Left_velocity / 2 * M_PI * Distance_between_wheels - Right_velocity / 2 * M_PI * Distance_between_wheels; //  Units radians/second.

    return Velocity;
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
    gpio_set_irq_enabled_with_callback(LEFT_Encoder,  GPIO_IRQ_EDGE_FALL, true, &Left_Encoder_callback);

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
    gpio_set_irq_enabled_with_callback(RIGHT_Encoder,  GPIO_IRQ_EDGE_FALL, true, &Right_Encoder_callback);

}

int main() {
    
    stdio_init_all();
    Motor_init();


    sleep_ms(4000);

    
 
    while (1)
    {
        Velocity = Robot_Velocity( Left_Encoder_Delta_Time, Right_Encoder_Delta_Time);
        printf("Linear: %f,  angular: %f, Right velocity; %f, Right_Delta_Time; %f  timestamp; %f \n",
                Velocity.Linear, Velocity.angular, Right_velocity,Right_Encoder_Delta_Time, timestamp );
        sleep_ms(400);

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

   //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   Set up DMA channels   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   /* uint sample_channel = dma_claim_unused_channel(true);
    uint control_channel = dma_claim_unused_channel(true);
    dma_channel_config c2 = dma_channel_get_default_config(sample_channel);
    dma_channel_config c3 = dma_channel_get_default_config(control_channel);

    //----------------------------Channel 2.-------------------------
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);

    channel_config_set_dreq(&c2, DREQ_I2C0_RX);

    dma_channel_configure(
        sample_channel,
        &c2,
        Pixel_array_buffer, 
        &i2c_hw->fifo,
        Number_of_pixels,
        false
        );//_____________________________________________

    //--------------------------Channel 3.------------------
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);
    channel_config_set_read_increment(&c3, false);
    channel_config_set_write_increment(&c3, false);
    channel_config_set_chain_to(&c3, sample_channel);

    dma_channel_configure(
       control_channel,
       &c3,
       &dma_hw->ch[sample_channel].write_addr, 
       &First_pixel_buffer_pointer,
       1,
       true
       );//____________________________________________*/
