
#include "hardware/pwm.h"
#include <math.h>
#include "hardware/i2c.h"
#include "ICM20600.h"
#include <time.h>
#include "SimpleKalmanFilter.h"
#include "AK09918.h"

#include "pico/multicore.h"

extern "C"{
#include "rcl/rcl.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

}

#include <Eigen/Core>
#include <Eigen/Geometry>
// === the fixed point macros (16.15) ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)

using namespace Eigen;

// Arrays in which raw measurements will be stored
clock_t clock()
{
    return (clock_t) time_us_32() / 1000000;
}
const uint LED_PIN = 25;
int Spinlock_num_odom;
spin_lock_t *spinlock_odom;

AK09918_err_type_t err;
AK09918 ak09918;
ICM20600 icm20600(true);

rcl_publisher_t 		PubOdom;
sensor_msgs__msg__Imu ImuMsg;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

int acceleration_X, acceleration_Y, acceleration_Z;

int gyro_X, gyro_Y, gyro_Z;

int32_t Magnetic_X, Magnetic_Y, Magnetic_Z;

int32_t Magnetic_offset_X, Magnetic_offset_Y, Magnetic_offset_Z;

// orientation
double Pitch = 1, roll =1, yaw = 1;
// Accelerometer orientation.
double Accelerometer_Pitch, Accelerometer_roll;
// gyro orientation.
double gyro_Pitch, gyro_roll, gyro_yaw;

// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_shenzhen =  5.16* M_PI / 180;

double startTime;
double endTime ;
double Delta_Time;

double priv_gyro_Pitch;
double Linear_velocity = 0;
double priv_Linear_velocity = 0;
double Distance = 0;

double Acc_X;
double prev_Acc_X;
double Sum_Acc_X;
double chige_betwin_megur_A;
double clibrate_Acc_X;
double kalman_Acc_X;
double X_position;
int cont_Sum_Acc_X;
double G ;
SimpleKalmanFilter simpleKalmanFilter(10, 2, 0.01);

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

void calibrate_magnometer(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
    int32_t value_x_min = 0;
    int32_t value_x_max = 0;
    int32_t value_y_min = 0;
    int32_t value_y_max = 0;
    int32_t value_z_min = 0;
    int32_t value_z_max = 0;
    uint32_t timeStart = 0;

    ak09918.getData(&Magnetic_X, &Magnetic_Y, &Magnetic_Z);

    value_x_min = Magnetic_X;
    value_x_max = Magnetic_X;
    value_y_min = Magnetic_Y;
    value_y_max = Magnetic_Y;
    value_z_min = Magnetic_Z;
    value_z_max = Magnetic_Z;
    sleep_ms(100);

    timeStart = time_us_32();

    while ((time_us_32() - timeStart) < timeout) {
        ak09918.getData(&Magnetic_X, &Magnetic_Y, &Magnetic_Z);

        /* Update x-Axis max/min value */
        if (value_x_min > Magnetic_X) {
            value_x_min = Magnetic_X;

        } else if (value_x_max < Magnetic_X) {
            value_x_max = Magnetic_X;
        }
        /* Update y-Axis max/min value */
        if (value_y_min > Magnetic_Y) {
            value_y_min = Magnetic_Y;

        } else if (value_y_max < Magnetic_Y) {
            value_y_max = Magnetic_Y;
        }
        /* Update z-Axis max/min value */
        if (value_z_min > Magnetic_Z) {
            value_z_min = Magnetic_Z;

        } else if (value_z_max < Magnetic_Z) {
            value_z_max = Magnetic_Z;
        }
        sleep_ms(100);
    }
    *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
    *offsety = value_y_min + (value_y_max - value_y_min) / 2;
    *offsetz = value_z_min + (value_z_max - value_z_min) / 2;
}

sensor_msgs__msg__Imu update_IMU(double Pitch, double roll, double yaw){

//Update header   
    int64_t time = rmw_uros_epoch_nanos();
    ImuMsg.header.stamp.sec = time / 1000000000;
    ImuMsg.header.stamp.nanosec = time;
    rosidl_runtime_c__String__assign(&ImuMsg.header.frame_id,"imu_link");

//orientation
    Quaterniond q;
	Matrix3d m;
	m = AngleAxisd(roll, 	    	Vector3d::UnitX())
	  * AngleAxisd(Pitch,  		    Vector3d::UnitY())
	  * AngleAxisd(yaw,             Vector3d::UnitZ());
	q = m;
	ImuMsg.orientation.x = q.x();
	ImuMsg.orientation.y = q.y();
	ImuMsg.orientation.z = q.z();
	ImuMsg.orientation.w = q.w();

    ImuMsg.angular_velocity.x = gyro_roll * M_PI / 180;
	ImuMsg.angular_velocity.y = gyro_Pitch * M_PI / 180;
	ImuMsg.angular_velocity.z = gyro_yaw * M_PI / 180;

    ImuMsg.linear_acceleration.x = acceleration_X;
	ImuMsg.linear_acceleration.y = acceleration_Y;
	ImuMsg.linear_acceleration.z = acceleration_Z;

    return ImuMsg;
}

void  setupImuMsg(){
    sensor_msgs__msg__Imu__init(&ImuMsg);
    //POSE
	ImuMsg.linear_acceleration.x = 0.0;
	ImuMsg.linear_acceleration.y = 0.0;
	ImuMsg.linear_acceleration.z = 0.0;

	ImuMsg.angular_velocity.x = 0.0;
	ImuMsg.angular_velocity.y = 0.0 - 0.017148675091778014;
	ImuMsg.angular_velocity.z = 0.0;

	ImuMsg.orientation.x = 0.0;
	ImuMsg.orientation.y = 0.0;
	ImuMsg.orientation.z = 0.0;  
    ImuMsg.orientation.w = 0.0;
}

void core_1() {
   /////////////////////////// IMU Set up /////////////////
    icm20600.initialize();
    icm20600.setPowerMode(ICM_6AXIS_LOW_NOISE);
    icm20600.setAccScaleRange(RANGE_2G);
    icm20600.setAccOutputDataRate(ACC_RATE_1K_BW_218);
    icm20600.setAccAverageSample(ACC_AVERAGE_32);
    
    //err = ak09918.initialize();
    //ak09918.switchMode(AK09918_POWER_DOWN);
    //ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

    //err = ak09918.isDataReady();
   //while (err != AK09918_ERR_OK)
   //{
   //    sleep_ms(10);
   //    err = ak09918.isDataReady();
   //}
   // sleep_ms(1000);
   // calibrate_magnometer( 10000000, &Magnetic_offset_X, &Magnetic_offset_Y, &Magnetic_offset_Z);
    
    for (size_t i = 0; i < 4000; i++)
    {

    G = G + sqrt(sqrt( (double)icm20600.getAccelerationZ() * (double)icm20600.getAccelerationZ() + (double)icm20600.getAccelerationY() * (double)icm20600.getAccelerationY()) *
             sqrt( (double)icm20600.getAccelerationZ() * (double)icm20600.getAccelerationZ() + (double)icm20600.getAccelerationY() * (double)icm20600.getAccelerationY()) + (double)icm20600.getAccelerationX() * (double)icm20600.getAccelerationX());
             sleep_ms(1);
    }
    G = G / 4000;

    for (size_t i = 0; i < 4000; i++)
    {
         //Pitch
        Delta_Time = (time_us_32() - endTime) / 1000000;
        endTime = time_us_32();
        acceleration_Z = icm20600.getAccelerationZ();
        acceleration_X = icm20600.getAccelerationX();
        acceleration_Y = icm20600.getAccelerationY();

        Accelerometer_Pitch = atan(acceleration_X / sqrt( acceleration_Z * acceleration_Z + acceleration_Y * acceleration_Y));
        Pitch = Accelerometer_Pitch ;
        
        //roll
        Accelerometer_roll = atan(acceleration_Y / sqrt( acceleration_Z * acceleration_Z + acceleration_X * acceleration_X));
        roll = Accelerometer_roll ;
        
        Acc_X =  (icm20600.getAccelerationX() - sin(Pitch) * G) ;
        Sum_Acc_X = Sum_Acc_X + Acc_X;
        chige_betwin_megur_A = chige_betwin_megur_A + abs(Acc_X - prev_Acc_X);
        prev_Acc_X = Acc_X;
    }

    clibrate_Acc_X = Sum_Acc_X / 4000;
    chige_betwin_megur_A = chige_betwin_megur_A / 4000;
    
    while (1)
    {
        
        Delta_Time = (time_us_32() - endTime) / 1000000;
        endTime = time_us_32();
        acceleration_Z = icm20600.getAccelerationZ() * 8;
        acceleration_X = icm20600.getAccelerationX() * 8;
        acceleration_Y = icm20600.getAccelerationY() * 8;

        //ak09918.getData(&Magnetic_X, &Magnetic_Y, &Magnetic_Z);
        //Magnetic_X = Magnetic_X - Magnetic_offset_X;
        //Magnetic_Y = Magnetic_Y - Magnetic_offset_Y;
        //Magnetic_Z = Magnetic_Z - Magnetic_offset_Z;
        //Pitch
        Accelerometer_Pitch = atan(acceleration_X / sqrt( acceleration_Z * acceleration_Z + acceleration_Y * acceleration_Y));
        gyro_Pitch = (icm20600.getGyroscopeY() - 0) ;   
        Pitch = (Pitch + gyro_Pitch * Delta_Time * M_PI / 180) * 0.999 - Accelerometer_Pitch * 0.001;
        //roll
        Accelerometer_roll = atan(acceleration_Y / sqrt( acceleration_Z * acceleration_Z + acceleration_X * acceleration_X));
        gyro_roll =  icm20600.getGyroscopeX() ;
        roll = (roll + gyro_roll * Delta_Time * M_PI / 180) * 0.995 + Accelerometer_roll * 0.005;  
        //yaw
        //double Magnetic_roll = atan2(acceleration_Y, acceleration_Z);
        //double Magnetic_pitch = atan2(-acceleration_X, sqrt(acceleration_Y * acceleration_Y + acceleration_Z * acceleration_Z));
        gyro_yaw = icm20600.getGyroscopeZ();
        //double Xheading = Magnetic_X * cos(Pitch) + Magnetic_Y * sin(roll) * sin(Pitch) + Magnetic_Z * cos(roll) * sin(Pitch);
        //double Yheading = Magnetic_Y * cos(roll) - Magnetic_Z * sin(Pitch);
        //double heading = M_PI - atan2(Yheading, Xheading) + declination_shenzhen;
        
        
        //yaw =  ((yaw + gyro_yaw * Delta_Time * M_PI / 180) * 0.999 - heading * 0.001) + M_PI;

    //     if  (yaw > heading){
    //      if  (yaw - heading > M_PI)
    //       {
    //         yaw = yaw * 0.999  + (heading + 2*M_PI)* 0.001;
    //       }
    //      else
    //      { 
    //        yaw = yaw * 0.999 - heading * 0.001;
    //      } 
    //    }
    //   if( yaw < heading )
    //    {   
    //      if (heading - yaw > M_PI)
    //        {
    //          yaw = yaw * 0.999 - (heading + 2*M_PI) * 0.001;
    //        }
    //       else
    //       {
    //          yaw = yaw * 0.999 + heading  * 0.001;
    //       } 
    //    }
        if (yaw < 0){yaw += 2*M_PI;}
        if (yaw > 2*M_PI){yaw -= 2*M_PI;}
        yaw =  (yaw + gyro_yaw * Delta_Time * M_PI / 180);

        
        
        ImuMsg = update_IMU(Pitch, roll, yaw);

    }
   
}

int main(){
   
    stdio_init_all();
    setupImuMsg();

    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

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
 
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
   
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

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

    rclc_node_init_default(&node, "pico_IMU_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_publisher_init_default(
        &PubOdom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "IMU_msgs");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);
    msg.data = 0;

    multicore_launch_core1(core_1);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  //  spin_lock_unsafe_blocking(spinlock_odom);      
   //     ImuMsg = update_IMU(Pitch, roll, yaw);
   // spin_unlock_unsafe(spinlock_odom);
        rcl_ret_t ret_odom = rcl_publish(&PubOdom, &ImuMsg, NULL); 
       
    }
    return 0;
}
