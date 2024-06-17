#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_fu
#include "rosidl_runtime_c/primitive
#include <geometry_msgs/msg/twist.h>


struct antonio_dom {
	double x;
	double y;
	double a;
};

typedef struct antonio_dom antonio_dom_t;

void setupTwistMsg();