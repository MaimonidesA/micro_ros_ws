/*
 * Antonio.h
 *
 *  Created on: 22 Nov 2023
 *      Author: M
 */

#ifndef FIRMWARE_SRC_Antonio_H_
#define FIRMWARE_SRC_Antonio_H_

#include "Agent.h"
#include "MotorsAgent.h"
#include "uRosEntities.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
}

#define TWIST_DEBUG

#define WHEEL_RADIUS   0.039
#define WHEEL_DEPTH    0.055
#define WHEELS_SEP     0.24
#define WHEELS_OFFSET  0.24 //To Y center of Robot

#define MAX_TWIST_TIME_MS 1500

struct AntonioOdom {
	double x;
	double y;
	double a;
};

typedef struct AntonioOdom AntonioOdom_t;

class Antonio : public Agent, public uRosEntities {
public:
	Antonio();
	virtual ~Antonio();

	void setMotorsAgent(MotorsAgent *p);
	/***
	 * Create the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);
	/***
	 * Destroy the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);
	/***
	 * Provide a count of the number of entities
	 * @return number of entities >=0
	 */
	virtual uint getCount();
	/***
	 * Return the number of handles needed by the executor
	 * @return
	 */
	virtual uint getHandles();
	/***
	 * Add subscribers, guards and timers to the executor
	 * @param executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);
	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);

protected:
	/***
	 * Run loop for the agent.
	 */
	virtual void run();
	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:

	void updateOdom();
	void publishOdom();
	void setupOdomMsg();

	void setupTwistMsg();
	void robotStop();

	MotorsAgent *pMotorsAgent = NULL;

	AntonioOdom_t xMotorsOdom;
	AntonioOdom_t xAntonioOdom;
	AntonioOdom_t xAntonioVelocity;

	uint32_t xLastVelocityTime = 0;

	rcl_publisher_t 			xPubOdom;
	nav_msgs__msg__Odometry 	xOdomMsg;

	rcl_subscription_t 			xSubTwist;
	uRosSubContext_t   			xSubTwistContext;
	geometry_msgs__msg__Twist 	xTwistMsg;

	uint32_t xLastTwistTimestamp = 0;
};

#endif /* FIRMWARE_SRC_Antonio_H_ */
