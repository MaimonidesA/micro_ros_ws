/*
 * MotorsAgent.h
 *
 *  Created on: 6 Aug 2023
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_MOTORSAGENT_H_
#define FIRMWARE_SRC_MOTORSAGENT_H_

#include "pico/stdlib.h"
#include "Agent.h"
#include "MotorMgr.h"
#include "uRosEntities.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
}

#ifndef NUM_MOTORS
#define NUM_MOTORS 2
#endif

class MotorsAgent : public Agent, public uRosEntities, public MotorMgr{
public:
    MotorsAgent(uint8_t gpCW, uint8_t gpCCW, uint8_t gpPWM, uint8_t gpIN);

	virtual ~MotorsAgent();
	/***
 * Add Motor
 * @param index - Index of the Motor
 * @param gpCW - CW enable terminal on driver
 * @param gpCCW - CCW enable terminal on driver
 * @param PWM - voltage control
 * @param gpB - encoder input - not in use
 */
	void addMotor(uint index, uint8_t gpCW, uint8_t gpCCW, uint8_t PWM, uint8_t gpIN);
	/***
	 * Return specific motor or NULL if none
	 * @param index
	 * @return
	 */
	MotorMgr * getMotor(uint index);
	
	void setSpeed(uint index, float percent, bool cw);
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
	 * Configure PID for motor
	 * @param index - of the motor
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	/*void configPID(uint index,
			float kP, float kI, float kD);

	/***
	 * Configure PID for all the motors
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	/*void configAllPID(float kP, float kI, float kD);

	/***
	 * Set the speed of the motor to be controlled
	 * @param index of the motor
	 * @param rpm rev per minute
	 * @param cw direction - true if clockwise
	 */
	/*void setSpeedRPM(uint index,
			float rpm, bool cw);

	/***
	 * Set the speed of the motor to be controlled
	 * @param index of the motor
	 * @param rps radians per second
	 * @param cw direction - true if clockwise
	 */
	/*void setSpeedRadPS(uint index, float rps, bool cw);*/

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
	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);


private:

	/*void initJointState();
	void pubJointState();*/
	float xThrottle_percent =0.1;
	bool xCW = true;
	uint xIndex;
	MotorMgr *pMotors[NUM_MOTORS];

	//rcl_publisher_t xPubJoint;
	//sensor_msgs__msg__JointState xJointStateMsg;
};

#endif /* FIRMWARE_SRC_MOTORSAGENT_H_ */
