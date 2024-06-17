
#include "FreeRTOS.h"
#include "semphr.h"

#include "task.h"
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

// micro-ros includes
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_uart_transports.h"

#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/pose.h>

// === the fixed point macros (16.15) ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)

//#define DMA_channel_2_write_address_pointer 0x58400000

#define ADC_GPIO 26   //pin 31  ADC

const uint LED_PIN = 25;

#define WRAPVAL 65502
#define CLKDIV 14.5f
#define Number_of_pixels 3694


rcl_publisher_t ccd_pub;
geometry_msgs__msg__Pose position;

std_msgs__msg__Int32 msg;
rcl_subscription_t subscriber;
bool Set_zero;
int Z_init = 14;

uint slice_num_0 = 0;       // pin 1 Main timer
uint slice_num_5 = 2;       // pin 7 ICG
uint slice_num_6 = 3;       // pin 9 SH

bool ADC_Status = false;

int Laser_height_array[10];
int Count = 0;

uint sample_channel ;
uint control_channel;

uint8_t Pixel_array_buffer[Number_of_pixels];
uint8_t * First_pixel_buffer_pointer = &Pixel_array_buffer[0];

bool Pixel_array_buffer_1_flag = false; //**************************If true writing to buffer one. if false writing to  buffer 2. 

uint8_t Average_pixels = 0;
uint32_t sum_pixels = 0;
int Laser_coverants_pixels_count = 0;
int Laser_index_sum = 0;
int Laser_height = 0;
int Laser_height_average = 0;

void on_pwm_5_wrap() // PWM interrupt
{ 
    pwm_set_counter(slice_num_0, 0);
    pwm_clear_irq(slice_num_5);
    
    pwm_set_counter(slice_num_6, 0);

     if (!ADC_Status) { adc_run(true);
        ADC_Status = true;
    }
    
    pwm_retard_count(slice_num_0);
    pwm_retard_count(slice_num_0);
    for (size_t i = 0; i < 750; i++){
        pwm_retard_count(slice_num_6);
    }  

    Pixel_array_buffer_1_flag = !Pixel_array_buffer_1_flag;
}

void CCD_Timers()
{  
   // printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Arming CCD_Timers>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");


    //**************Main timer: 2 megahertz *************//
    gpio_set_function(0, GPIO_FUNC_PWM);
    
    pwm_set_enabled(slice_num_0, true);
    pwm_set_wrap(slice_num_0, 62);
    pwm_set_chan_level(slice_num_0, 0, 31);

     pwm_set_output_polarity(slice_num_0, true, true);
    //___________________________________________________//
    
    //************  ICG timer: 133 hertz********************//
    gpio_set_function(5, GPIO_FUNC_PWM);
    
    pwm_clear_irq(slice_num_5);
    pwm_set_irq_enabled(slice_num_5, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_5_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_set_wrap(slice_num_5, WRAPVAL);
    pwm_set_clkdiv(slice_num_5, CLKDIV);
    pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 86);
    //_________________________________________________//
    
    //***************SH timer ******************************//
    gpio_set_function(6, GPIO_FUNC_PWM);
    
    pwm_set_enabled(slice_num_6, true);
    pwm_set_wrap(slice_num_6, 7500);   //Typical  1250
    pwm_set_chan_level(slice_num_6, 0, 500);   //Typical  500

     pwm_set_output_polarity(slice_num_6, true, true);
    //_______________________________________________________//

   // printf("<<<<<<<<all the timers are ready>>>>>>>>.\n ");
   
}

void ADC_DMA_config(){

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  ADC configuration. >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   adc_gpio_init(ADC_GPIO);
   adc_init();

   adc_select_input(0);
   adc_fifo_setup(true, true, 1, false, true);
   adc_set_clkdiv(0);
   //_________________________________________

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   Set up DMA channels   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    sample_channel = dma_claim_unused_channel(true);
    control_channel = dma_claim_unused_channel(true);
    dma_channel_config c2 = dma_channel_get_default_config(sample_channel);
    dma_channel_config c3 = dma_channel_get_default_config(control_channel);

    //----------------------------Channel 2.-------------------------
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);

    channel_config_set_dreq(&c2, DREQ_ADC);

    dma_channel_configure(
        sample_channel,
        &c2,
        Pixel_array_buffer, 
        &adc_hw->fifo,
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
       );
}//_____________________________________________________________________________*/

void Data_manipulation(){

    Average_pixels = 0;
    sum_pixels = 0;
    Laser_coverants_pixels_count = 0;
    Laser_index_sum = 0;
    Laser_height = 0;
    Laser_height_average = 0;

    dma_channel_wait_for_finish_blocking(sample_channel);
    adc_run(false);
    adc_fifo_drain();
    for (size_t i = 0; i < Number_of_pixels; i++)
    {
        if (  i > 31 && i < 3648 ){
         sum_pixels = sum_pixels + Pixel_array_buffer[i]; 
        }  
    }
    Average_pixels = sum_pixels / 3648;
    for (int i = 0; i < Number_of_pixels; i++)
    {
        if (((int)(Pixel_array_buffer[i]) + 30) < ((int)(Average_pixels))) {
            Laser_index_sum = Laser_index_sum + i;
            Laser_coverants_pixels_count++;
        }
    }
     Laser_height = (Laser_index_sum / Laser_coverants_pixels_count);
     if (Count < 10 && Laser_height != 0) {
        Laser_height_array[Count] = Laser_height;
        Count++;
     }else if(Count == 10){ 
        for (size_t i = 0; i < 10; i++)
        {
            Laser_height_average = Laser_height_average + Laser_height_array[i];
        }
        position.position.z = ((double)(Laser_height_average) * 8)/10000;
        Count = 0;
     }

    dma_channel_start(control_channel) ;
    ADC_Status = false;
   }

void Set_Z(){
   position.position.z = (Laser_height_average * 8)/10000;
}

void set_zero_callback(){
    //todo
}

int main()
{
    stdio_init_all();

    ADC_DMA_config();
    //*************Enable DMA  *****************
    dma_start_channel_mask((1u << sample_channel));
    
    CCD_Timers();
    //*************Enable pwm  *****************
    pwm_set_mask_enabled((1u << slice_num_0) | (1u << slice_num_5) | (1u << slice_num_6)); 

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

    rclc_executor_t executor;
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {  // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "ccd_node", "", &support);

    rclc_publisher_init_default(
        &ccd_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
        "ccd_Z");        

    rcl_ret_t rc = rclc_subscription_init_default(
         &subscriber,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
         "/height_calibration");


    rclc_executor_init(&executor, &support.context, 2, &allocator);
   // rclc_executor_add_timer(&executor, &timer);
    //* Add subscription to the executor
    rc = rclc_executor_add_subscription(
      &executor, &subscriber, &Set_zero,
      &set_zero_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);
 
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
       
        Data_manipulation();
        rcl_ret_t ret_Pose = rcl_publish(&ccd_pub, &position, NULL);
  
    };
    return 0;
   
}