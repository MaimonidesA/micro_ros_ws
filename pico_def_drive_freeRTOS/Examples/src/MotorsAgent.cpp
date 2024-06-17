/*
 * MotorsAgent.cpp
 *
 *  Created on: 6 Aug 2023
 *      Author: jondurrant
 */

#include "MotorsAgent.h"

#include "uRosBridge.h"

#include <inttypes.h>
#include <cmath>


MotorsAgent::MotorsAgent(uint8_t gpCW, uint8_t gpCCW, uint8_t gpPWM, uint8_t gpIN) : MotorMgr( gpCW, gpCCW, gpPWM, gpIN){
	for (uint i=0; i < NUM_MOTORS; i++){
		pMotors[i] = NULL;
	}
}

MotorsAgent::~MotorsAgent() {
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			delete pMotors[i];
		}
	}
}

void MotorsAgent::run(){
	
	for (;;){

		 
		 setThrottle(xThrottle_percent, true);
		 vTaskDelay(200); 
		}
		
	}


void MotorsAgent::setSpeed(uint index, float percent, bool cw){
	xIndex = index;
	xThrottle_percent = percent;
	//xCW = cw;
}

configSTACK_DEPTH_TYPE MotorsAgent::getMaxStackSize(){
	return 1024;
}

void MotorsAgent::createEntities(rcl_node_t *node, rclc_support_t *support){
	/*rclc_publisher_init_default(
		&xPubJoint,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"joint_states");*/
}

void MotorsAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support){

	/*rcl_publisher_fini(&xPubJoint, node);*/
}

uint MotorsAgent::getCount(){
	return 0;
}

uint MotorsAgent::getHandles(){
	return 1;
}

void MotorsAgent::addToExecutor(rclc_executor_t *executor){
	//NOP
}

void MotorsAgent::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){
	//NOP
}

MotorMgr * MotorsAgent::getMotor(uint index){
	if (index >= NUM_MOTORS){
		return NULL;
	}
	return pMotors[index];
}

void MotorsAgent::addMotor(uint index, uint8_t gpCW, uint8_t gpCCW, uint8_t PWM, uint8_t gpIN){
	if (index < NUM_MOTORS)
	{
		pMotors[index] = new MotorMgr(gpCW, gpCCW, PWM, gpIN);
	}
}

/*void MotorsAgent::initJointState(){
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	char name[32];

	//Possition
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.position, NUM_MOTORS);
	xJointStateMsg.position.data[0] = 0.0;
	xJointStateMsg.position.size = NUM_MOTORS;
	xJointStateMsg.position.capacity = NUM_MOTORS;

	//Velocity
	rosidl_runtime_c__double__Sequence__init(&xJointStateMsg.velocity, NUM_MOTORS);
	xJointStateMsg.velocity.data[0] = 0.0;
	xJointStateMsg.velocity.size = NUM_MOTORS;
	xJointStateMsg.velocity.capacity = NUM_MOTORS;

	//Name
	rosidl_runtime_c__String__Sequence__init(&xJointStateMsg.name, NUM_MOTORS);
	for (uint i=0; i < NUM_MOTORS; i++){
		sprintf(name, "motor_%u", i);
		if (!rosidl_runtime_c__String__assign(&xJointStateMsg.name.data[i], name)){
			printf("ERROR: Joined assignment failed\n");
		}
	}
	xJointStateMsg.name.size=NUM_MOTORS;
	xJointStateMsg.name.capacity=NUM_MOTORS;
}*/

/*void MotorsAgent::pubJointState(){
	//Populate the Joint possition message
	int64_t time = rmw_uros_epoch_nanos();

	xJointStateMsg.header.stamp.sec = time / 1000000000;
	xJointStateMsg.header.stamp.nanosec = time % 1000000000;

	for (uint i=0; i < NUM_MOTORS; i++)
	{
		if (pMotors[i] != NULL) 
		{
			xJointStateMsg.position.data[i] = pMotors[i]->getRadians() - M_PI;
			xJointStateMsg.velocity.data[i] = pMotors[i]->getAvgRadPerSec();
		}
	}

	if (!uRosBridge::getInstance()->publish(&xPubJoint, &xJointStateMsg, this, NULL))
	{
		printf("Joint Pub failed\n");
	}
}
*/

/*void MotorsAgent::configPID(uint index,
		float kP, float kI, float kD){
	if (pMotors[index] != NULL){
		pMotors[index]->configPID(kP, kI, kD);
	}
}*/

/*void MotorsAgent::configAllPID(float kP, float kI, float kD){
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			pMotors[i]->configPID(kP, kI, kD);
		}
	}
}*/

/*void MotorsAgent::setSpeedRPM(uint index, float rpm, bool cw){
	if (pMotors[index] != NULL){
		pMotors[index]->setSpeedRPM(rpm, cw);
	}
}*/

/*void MotorsAgent::setSpeedRadPS(uint index, float rps, bool cw){
	if (pMotors[index] != NULL)
	{
		if (rps >= 0.0)
		{
			pMotors[index]->setSpeedRadPS(rps, cw);
		} 

		else 
		{
			pMotors[index]->setSpeedRadPS(fabs(rps), !cw);
		}
	}
}*/
