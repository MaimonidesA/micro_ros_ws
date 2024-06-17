#include "main.h"

#include <math.h>
#include <stdio.h>

#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "ICM20600.h"

extern "C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/pose.h>
}
#include <Eigen/Core>
#include <Eigen/Geometry>

#define timestamp double((int)time_us_32())

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

using namespace Eigen;

const uint LED_PIN = 25;
// SMP Types.
int Spinlock_num_odom;
spin_lock_t *spinlock_odom;
int Spinlock_num_ticks;
spin_lock_t *spinlock_ticks;

// ros Types
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_subscription_t subscriber;

rcl_publisher_t 		PubOdom;
nav_msgs__msg__Odometry OdomMsg;

rcl_publisher_t 		PubPose;
geometry_msgs__msg__Pose position;



geometry_msgs__msg__Twist Twist_msg;

geometry_msgs__msg__Transform odom_transform;

double Linear_request;
double  Angular_request;
bool cmd_val_msg_Arrive;
double Last_cmd_val_arrive;

// PID parameters.
double LKP = 1, LKI = 0.000, LKD = 0.10; //Linear Pid [0 < coefficients < 1]
double AKP = 0.1, AKI = 0.0000, AKD = 0.00; //Angular Pid [0 < coefficients < 1]
double L_Continuous_error;
double A_Continuous_error;
double Last_Linear_err;
double Last_Angular_err;
double Linear_PID;
double Angular_PID;
double Angular_err;
double Linear_err;

// Wheels parameters.
double Ticks_per_cycle = 275;
double Wheel_Radius = 0.0375;  //0.036 Meter.
double Wheel_Circumference = 2*M_PI*Wheel_Radius; // Units M
double Distance_between_wheels = 0.241; // Units M

double Right_throttle;
double Left_throttle;

bool Left_Forward;
double Left_RPM;
double Left_velocity;
double Left_Encoder_Delta_Time =0;
double Left_Encoder_priv_Time = 0;
double xLeft_ticks = 0;

bool Right_Forward;
double Right_RPM;
double Right_velocity;
double Right_Encoder_Delta_Time = 0;
double Right_Encoder_priv_Time = 0;
double xRight_ticks = 0;

double Average_distance;
double Previous_average_distance;

struct Velocity_t {
   double Linear = 0;
   double angular = 0;
} Velocity;

struct position_t {
    double X;
    double Y;
    double Angle;
};

geometry_msgs__msg__Pose Robot_position(double Right_ticks, double Left_ticks){
    double Right_distance = (Wheel_Circumference / Ticks_per_cycle) * Right_ticks;
    double Left_distance = (Wheel_Circumference / Ticks_per_cycle) * Left_ticks;
    position.orientation.z += (Right_distance - Left_distance) /  Distance_between_wheels;
    //if(position.orientation.z > 2*M_PI){position.orientation.z -= 2*M_PI;}else if(position.orientation.z < 0){position.orientation.z += 2*M_PI;}
    Average_distance = (Right_distance + Left_distance) / 2;
    xRight_ticks = 0;
    xLeft_ticks = 0;
    Right_distance = 0;
    Left_distance = 0;
    position.position.x += cos(position.orientation.z) * Average_distance ;
    position.position.y += sin(position.orientation.z) * Average_distance ;
    Previous_average_distance = Average_distance;
    return position;
}

Velocity_t Robot_Velocity(double Left_Encoder_Delta_Time, double Right_Encoder_Delta_Time) {
    
    Left_velocity = (Wheel_Circumference / Ticks_per_cycle) / (Left_Encoder_Delta_Time / 1000000); // Units m/s
    if (!Left_Forward){Left_velocity  *= (-1);}

    Right_velocity = (Wheel_Circumference / Ticks_per_cycle) / (Right_Encoder_Delta_Time / 1000000); // Units m/s
    if (!Right_Forward){Right_velocity *=  (-1);}

    if (Right_Encoder_Delta_Time == 0) {Right_velocity = 0;}
    if (Left_Encoder_Delta_Time == 0) {Left_velocity = 0;}
    Velocity.Linear = Left_velocity / 2 + Right_velocity / 2;
    Velocity.angular =  Right_velocity / (Distance_between_wheels) - Left_velocity / (Distance_between_wheels); //  Units radians/second.

    return Velocity;
}

void PID(double Linear_request, double Angular_request, double Angular_velocity, double Linear_velocity) {
    Linear_err =  Linear_request - Linear_velocity;
    Angular_err = Angular_request - Angular_velocity;
    double LP, LI, LD;   //Linear PID.
    double AP, AI, AD;   //Angular PID.
    L_Continuous_error = L_Continuous_error + Linear_err;
    A_Continuous_error = A_Continuous_error + Angular_err;

    LP = Linear_err * LKP;
    LI = (Linear_err + L_Continuous_error) * LKI;
    LI = (abs(LI) > 0.01) ? (LI/LI)* 0.01 : LI;
    LD = (Linear_err - Last_Linear_err) * LKD;
    Last_Linear_err = Linear_err;
    Linear_PID = (LP + LI + LD);
    Linear_PID = (Linear_PID > 0.2) ? 0.2 : Linear_PID ;
    Linear_PID = (Linear_PID < -0.2) ? -0.2 : Linear_PID ;

    AP = Angular_err * AKP ;
    AI = (Angular_err + A_Continuous_error) * AKI;
    AI = (abs(AI) > 0.01) ? (AI/AI)* 0.01 : AI;
    AD = (Angular_err - Last_Angular_err) * AKD;
    Last_Angular_err = Angular_err;
    Angular_PID = (AP + AI + AD)/2;
    Angular_PID = (Angular_PID > 0.2) ? 0.2 : Angular_PID ;
    Angular_PID = (Angular_PID < -0.2) ? -0.2 : Angular_PID ;

 
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
    double Linear_velocity;
    while (1)
    {
      int gyro_Z  = icm20600.getGyroscopeZ();
      int acceleration_X = icm20600.getAccelerationX();
      double acceleration_sum = 0;
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
}

void set_throttle(){

 PID(Linear_request, Angular_request, Velocity.angular, Velocity.Linear);
    if (abs(Angular_err/10) > abs(Linear_err)){
        if (Angular_err > 0){
            Right_throttle = Right_throttle  + abs(Angular_PID) ;
            Left_throttle = Left_throttle - abs(Angular_PID) ;
        }
        else if(Angular_err < 0){
            Right_throttle = Right_throttle - abs(Angular_PID) ;
            Left_throttle = Left_throttle + abs(Angular_PID) ;
        }
    }
        else{
            Right_throttle = Right_throttle + Linear_PID;
            Left_throttle = Left_throttle + Linear_PID;
        }
    int Right_throttle_to_low = 1; 
    int Left_throttle_to_low = 1; 
    Right_throttle = ((Right_throttle) > 1) ? 1 : Right_throttle; Right_throttle = (Right_throttle < -1) ? -1 : Right_throttle;
    Right_throttle_to_low = (abs(Right_throttle) < 0.15) ?  0 : 1;
    Left_throttle = (Left_throttle > 1) ? 1 : Left_throttle; Left_throttle = (Left_throttle < -1) ? -1 : Left_throttle;
    Left_throttle_to_low = (abs(Left_throttle) < 0.15) ? 0 :  1;
    //Left Motor
    int left_pwm = (int)((double)(0xffff) * abs(Left_throttle) * Right_throttle_to_low);
    pwm_set_gpio_level(LEFT_PWM, left_pwm);
	if (Left_throttle > 0 ){
		gpio_put(LEFT_CW, 0);
		gpio_put(LEFT_CCW, 1);
        Left_Forward = true;
	}
    else// run counter clockwise
    {		
		gpio_put(LEFT_CW, 1);
		gpio_put(LEFT_CCW, 0);
        Left_Forward = false;
    }
    //Right Motor
    int right_pwm = (int)((double)(0xffff) * abs(Right_throttle) * Left_throttle_to_low);
    pwm_set_gpio_level(RIGHT_PWM, right_pwm);
	if (Right_throttle > 0 ){
		gpio_put(RIGHT_CW, 0);
		gpio_put(RIGHT_CCW, 1);
        Right_Forward = true;
	}
    else // run counter clockwise
    {		
		gpio_put(RIGHT_CW, 1);
		gpio_put(RIGHT_CCW, 0);
        Right_Forward = false;
    }
}

void subscription_callback(const void * msgin)
{
    // Cast received message to used type
    const geometry_msgs__msg__Twist * pTwistMsg = (const geometry_msgs__msg__Twist *) msgin;

    Linear_request = pTwistMsg->linear.x;
    Angular_request =  pTwistMsg->angular.z; 
    Linear_request = (Linear_request > 0.26) ? 0.26 : Linear_request;
    Linear_request = (Linear_request < -0.26) ? -0.26 : Linear_request;
    Angular_request = (Angular_request > 2.4) ? 2.4 : Angular_request;
    Angular_request = (Angular_request < -2.4) ? -2.4 : Angular_request;
    cmd_val_msg_Arrive = true;
    Last_cmd_val_arrive = timestamp;
}

nav_msgs__msg__Odometry update_odom(double Linear, double angular, double position_x,double position_y, double Angle){

//Update header   
    int64_t time = rmw_uros_epoch_nanos();
    OdomMsg.header.stamp.sec = time / 1000000000;
    OdomMsg.header.stamp.nanosec = time;
    rosidl_runtime_c__String__assign(&OdomMsg.header.frame_id, "Wheels/odom");
    rosidl_runtime_c__String__assign(&OdomMsg.child_frame_id, "base_link");
//TWIST
     
	OdomMsg.twist.twist.linear.x = Linear;
	OdomMsg.twist.twist.angular.z = angular;

//POSE
    OdomMsg.pose.pose.position.x = position_x;
    OdomMsg.pose.pose.position.y = position_y;
    Quaterniond q;
	Matrix3d m;
	m = AngleAxisd(0.0, 	    	        Vector3d::UnitX())
	  * AngleAxisd(0.0,  		            Vector3d::UnitY())
	  * AngleAxisd(position.orientation.z, 	Vector3d::UnitZ());
	q = m;
	OdomMsg.pose.pose.orientation.x = q.x();
	OdomMsg.pose.pose.orientation.y = q.y();
	OdomMsg.pose.pose.orientation.z = q.z();
	OdomMsg.pose.pose.orientation.w = q.w();
    //OdomMsg.pose.pose.orientation.z = position.orientation.z;

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
}

void core_1(){
   // spin_lock_unsafe_blocking(spinlock_odom);

    bool RIGHT_Encoder_gpio = false;
    bool Left_Encoder_gpio = false;
    double Detect_motion = 0;
    //spin_unlock_unsafe(spinlock_odom);
    while (1)
    {
        if (gpio_get(RIGHT_Encoder) == 0 && !RIGHT_Encoder_gpio){
            Right_Encoder_Delta_Time = timestamp - Right_Encoder_priv_Time;
            Right_Encoder_priv_Time = timestamp;
            RIGHT_Encoder_gpio = true;
            Detect_motion = timestamp;
    spin_lock_unsafe_blocking(spinlock_ticks);
            if(Right_Forward){ xRight_ticks++;}else{xRight_ticks-- ;}
    spin_unlock_unsafe(spinlock_ticks);
        }
        else if (gpio_get(RIGHT_Encoder) != 0)
        {
             RIGHT_Encoder_gpio = false;
        }

        if (gpio_get(LEFT_Encoder) == 0 && !Left_Encoder_gpio){
            Left_Encoder_Delta_Time = timestamp - Left_Encoder_priv_Time;
            Left_Encoder_priv_Time = timestamp;
            Left_Encoder_gpio = true;
            Detect_motion = timestamp;
    spin_lock_unsafe_blocking(spinlock_ticks);
            if(Left_Forward){ xLeft_ticks++;} else{xLeft_ticks--;}
    spin_unlock_unsafe(spinlock_ticks);
        }
        else if (gpio_get(LEFT_Encoder) != 0)
        {
             Left_Encoder_gpio = false;
        }
        if ((timestamp - Detect_motion) > 57000){
            spin_lock_unsafe_blocking(spinlock_odom);  
            Left_Encoder_Delta_Time = 0;
            Right_Encoder_Delta_Time = 0;
            spin_unlock_unsafe(spinlock_odom);
        }

    

    }   
}

int main(){
   
    stdio_init_all();
    setupOdomMsg();
    Motor_init();

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    Spinlock_num_odom = spin_lock_claim_unused(true);
    spinlock_odom = spin_lock_init(Spinlock_num_odom);
 
    Spinlock_num_ticks = spin_lock_claim_unused(true);
    spinlock_ticks = spin_lock_init(Spinlock_num_odom);

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

    rclc_node_init_default(&node, "pico_def_drive_node", "", &support);

    rclc_publisher_init_default(
        &PubOdom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "Wheels/odom");

    rclc_publisher_init_default(
        &PubPose,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
        "Wheels/Pose");        

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
    rc = rclc_executor_add_subscription(&executor, &subscriber, &Twist_msg, &subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);
    msg.data = 0;

    multicore_launch_core1(core_1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    spin_lock_unsafe_blocking(spinlock_odom);      
        Robot_Velocity(Left_Encoder_Delta_Time, Right_Encoder_Delta_Time);
   // spin_lock_unsafe_blocking(spinlock_ticks);
        position = Robot_position(xRight_ticks, xLeft_ticks);          
   // spin_unlock_unsafe(spinlock_ticks);
        OdomMsg = update_odom(Velocity.Linear, Velocity.angular, position.position.x, position.position.y ,position.orientation.z);
    spin_unlock_unsafe(spinlock_odom);
        rcl_ret_t ret_odom = rcl_publish(&PubOdom, &OdomMsg, NULL); 
        rcl_ret_t ret_Pose = rcl_publish(&PubPose, &position, NULL); 
       
        if (cmd_val_msg_Arrive){
            set_throttle();
            
            if((timestamp - Last_cmd_val_arrive) > 2000000){
                pwm_set_gpio_level(RIGHT_PWM, 0);
                pwm_set_gpio_level(LEFT_PWM, 0);
                Right_throttle =0;
                Left_throttle =0;
                cmd_val_msg_Arrive = false;
            }
        }
    };
    return 0;
}
