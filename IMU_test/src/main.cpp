#include <iostream>
#include <stdio.h>
#include "hardware/pwm.h"
#include <math.h>
#include "hardware/i2c.h"
#include "ICM20600.h"
#include <time.h>
#include "SimpleKalmanFilter.h"
#include "AK09918.h"

//extern "C"{
//#include "rcl/rcl.h"
//#include <rcl/error_handling.h>
//#include <rclc/rclc.h>
//#include <rclc/executor.h>
//#include <std_msgs/msg/int32.h>
//#include <rmw_microros/rmw_microros.h>
//
#include "pico/stdlib.h"
////#include "pico_uart_transports.h"
//
//#include <rmw_microros/time_sync.h>
//#include <nav_msgs/msg/odometry.h>
//#include "rosidl_runtime_c/string_functions.h"
//#include "rosidl_runtime_c/primitives_sequence_functions.h"
//#include <geometry_msgs/msg/twist.h>
//}
// === the fixed point macros (16.15) ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)

#define BAT 21
// Arrays in which raw measurements will be stored
clock_t clock()
{
    return (clock_t) time_us_32() / 1000000;
}

ICM20600 icm20600(true);
AK09918_err_type_t err;
AK09918 ak09918;

// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_shenzhen = 5.16* M_PI / 180;

double av_heading = 0;

int acceleration_X;
int acceleration_Y;
int acceleration_Z;

int gyro_X;
int gyro_Y;
int gyro_Z;

int32_t Magnetic_X, Magnetic_Y, Magnetic_Z;

int32_t Magnetic_offset_X, Magnetic_offset_Y, Magnetic_offset_Z;

// orientation
double Pitch = 0, roll =0, yaw = 0;
// Accelerometer orientation.
double Accelerometer_Pitch, Accelerometer_roll;
// gyro orientation.
double gyro_Pitch, gyro_roll, gyro_yaw;

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
         printf(".");
        sleep_ms(100);
    }
    *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
    *offsety = value_y_min + (value_y_max - value_y_min) / 2;
    *offsetz = value_z_min + (value_z_max - value_z_min) / 2;
}

int main() {
    
    stdio_init_all();

    gpio_init(BAT);
    gpio_set_dir(BAT, GPIO_IN);

    startTime = clock();

    ICM20600 icm20600(true);

     ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

   /////////////////////////// IMU Set up /////////////////
    err = ak09918.initialize();
    
    icm20600.initialize();
    icm20600.setPowerMode(ICM_6AXIS_LOW_NOISE);
    icm20600.setAccScaleRange(RANGE_2G);
    icm20600.setAccOutputDataRate(ACC_RATE_1K_BW_218);
    //icm20600.setAccAverageSample(ACC_AVERAGE_32);
   // icm20600.setSampleRateDivier(2);

    ak09918.switchMode(AK09918_POWER_DOWN);
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
     


    err = ak09918.isDataReady();
   while (err != AK09918_ERR_OK)
   {
       sleep_ms(100);
       err = ak09918.isDataReady();
       printf("err = ak09918.isDataReady..............................\n1");
   }
   printf("start clibration magnometer..............................\n1"); 
    sleep_ms(2000);
    calibrate_magnometer( 10000000, &Magnetic_offset_X, &Magnetic_offset_Y, &Magnetic_offset_Z);

    for (size_t i = 0; i < 2000; i++)
    {

    G = G + sqrt(sqrt( (double)icm20600.getAccelerationZ() * (double)icm20600.getAccelerationZ() + (double)icm20600.getAccelerationY() * (double)icm20600.getAccelerationY()) *
             sqrt( (double)icm20600.getAccelerationZ() * (double)icm20600.getAccelerationZ() + (double)icm20600.getAccelerationY() * (double)icm20600.getAccelerationY()) + (double)icm20600.getAccelerationX() * (double)icm20600.getAccelerationX());
             sleep_ms(1);
    }
    G = G / 2000;

//    for (size_t i = 0; i < 4000; i++)
//    {
//         //Pitch
//        Delta_Time = (time_us_32() - endTime) / 1000000;
//        endTime = time_us_32();
//        acceleration_Z = icm20600.getAccelerationZ();
//        acceleration_X = icm20600.getAccelerationX();
//        acceleration_Y = icm20600.getAccelerationY();
//
//        Accelerometer_Pitch = atan(acceleration_X / sqrt( acceleration_Z * acceleration_Z + acceleration_Y * acceleration_Y));
//        Pitch = Accelerometer_Pitch ;
//        
//        //roll
//        Accelerometer_roll = atan(acceleration_Y / sqrt( acceleration_Z * acceleration_Z + acceleration_X * acceleration_X));
//        roll = Accelerometer_roll ;
//        
//        Acc_X =  (icm20600.getAccelerationX() - sin(Pitch) * G) ;
//        Sum_Acc_X = Sum_Acc_X + Acc_X;
//        chige_betwin_megur_A = chige_betwin_megur_A + abs(Acc_X - prev_Acc_X);
//        prev_Acc_X = Acc_X;
//        printf("%f,\n ", Acc_X);//******************7*/
//    }
//
//    clibrate_Acc_X = Sum_Acc_X / 4000;
//    chige_betwin_megur_A = chige_betwin_megur_A / 4000;
    
 
    while (1)
    {
    
        acceleration_Z = icm20600.getAccelerationZ();
        acceleration_X = icm20600.getAccelerationX();
        acceleration_Y = icm20600.getAccelerationY();
        
        Delta_Time = (time_us_32() - endTime) / 1000000;
        endTime = time_us_32();

        //Pitch

        Accelerometer_Pitch = atan(acceleration_X / sqrt( acceleration_Z * acceleration_Z + acceleration_Y * acceleration_Y));
 
        gyro_Pitch = gyro_Pitch - (icm20600.getGyroscopeY()+2) * Delta_Time * M_PI / 180 ;
    
        Pitch = (Pitch - (icm20600.getGyroscopeY()+2) * Delta_Time * M_PI / 180) * 0.999 + Accelerometer_Pitch * 0.001;
        
        //Roll

        Accelerometer_roll = atan(acceleration_Y / sqrt( acceleration_Z * acceleration_Z + acceleration_X * acceleration_X));
       
        gyro_roll = gyro_roll + icm20600.getGyroscopeX() * Delta_Time * M_PI / 180;
        
        roll = (roll + icm20600.getGyroscopeX() * Delta_Time * M_PI / 180) * 0.995 + Accelerometer_roll * 0.005;
        
        //Yaw
        gyro_yaw = gyro_yaw + icm20600.getGyroscopeZ() * Delta_Time * M_PI / 180;
        yaw = 

        Acc_X = ((double)icm20600.getAccelerationX() - sin(Pitch) * G) ;
        kalman_Acc_X = simpleKalmanFilter.updateEstimate(Acc_X);
       // Acc_X = prev_Acc_X * 0.95 + Acc_X * 0.05;
        //prev_Acc_X = Acc_X;

        Linear_velocity = Linear_velocity  + kalman_Acc_X * Delta_Time  ;
       
        bool bat = gpio_get(BAT);

        if (bat == 0 ){
            Acc_X = 0 ;
            Linear_velocity = 0;
        }
        X_position = X_position  + Linear_velocity * Delta_Time + kalman_Acc_X * Delta_Time * Delta_Time/2  ;
        if (abs(X_position) > 20){X_position = 0;}
        //if (abs(Linear_velocity) > 20){Linear_velocity = 0;}

        //Magnetometer

        ak09918.getData(&Magnetic_X, &Magnetic_Y, &Magnetic_Z);
        Magnetic_X -= Magnetic_offset_X;
        Magnetic_Y -= Magnetic_offset_Y;
        Magnetic_Z -= Magnetic_offset_Z;

        double Xheading = Magnetic_X * cos(Pitch) + Magnetic_Y * sin(roll) * sin(Pitch) + Magnetic_Z * cos(roll) * sin(Pitch);
        double Yheading = Magnetic_Y * cos(roll) - Magnetic_Z * sin(Pitch);
    
        double heading = M_PI + atan2(Yheading, Xheading); + declination_shenzhen;
        
        if  (av_heading > heading){
          if  (av_heading - heading > M_PI)
           {
             av_heading = av_heading * 0.999  + (heading + 2*M_PI)* 0.001;
           }
          else
          { 
            av_heading = av_heading * 0.999 - heading * 0.001;
          } 
        }
       if( av_heading < heading )
        {   
          if (heading - av_heading > M_PI)
            {
              av_heading = av_heading * 0.999 - (heading + 2*M_PI) * 0.001;
            }
           else
           {
              av_heading = av_heading * 0.999 + heading  * 0.001;
           } 
        }
        if (av_heading < 0){av_heading += 2*M_PI;}
        if (av_heading > 2*M_PI){av_heading -= 2*M_PI;}

//180 + 57.3 * 

        printf("av_heading: %f, heading :%f \n", av_heading, heading); 
        
        //printf("Pitch");
        //printf("%f", Pitch);
        //printf("roll");
        //printf("%f\n", roll);
        //std::cout << "roll";
        //std::cout << int(roll*32768.0);
        //std::cout << "\n";
        //printf("%f, %f, %f, %f, %f, %f,  %f, %f, %f, %f, %f\n"
        //,G, Delta_Time, X_position, Linear_velocity, Acc_X, Pitch, roll, acceleration_Z, clibrate_Acc_X, chige_betwin_megur_A, kalman_Acc_X);
    }
}


 //       if  (av_heading > heading){
 //         if  (av_heading - heading > M_PI)
 //          {
 //            av_heading = av_heading * 0.999  + (heading - 2*M_PI)* 0.001;
 //          }
 //         else
 //         { 
 //           av_heading = av_heading * 0.999 - heading * 0.001;
 //         } 
 //       }
 //      if( av_heading < heading )
 //       {   
 //         if (heading - av_heading > M_PI)
 //           {
 //             av_heading = av_heading   - heading * 0.001;
 //           }
 //          else
 //          {
 //             av_heading = av_heading  +  0.0001;
 //          } 
 //       }
 //       if (av_heading < 0){av_heading += 2*M_PI;}
 //       if (av_heading > 2*M_PI){av_heading -= 2*M_PI;}
