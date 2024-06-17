

/*
 * Antonio.cpp
 *
 *  Created on: 7 Aug 2023
 *      Author: jondurrant
 */

#include "Antonio.h"
#include <cstdio>
#include <cmath>
#include "uRosBridge.h"

//using namespace Eigen;

Antonio::Antonio() {
	xMotorsOdom.x=0.0;
	xMotorsOdom.y=0.0;
	xMotorsOdom.a=0.0;

	xAntonioOdom.x=0.0;
	xAntonioOdom.y=0.0;
	xAntonioOdom.a=0.0;
}

Antonio::~Antonio() {
	// TODO Auto-generated destructor stub
}

void Antonio::setMotorsAgent(MotorsAgent *p){
	pMotorsAgent = p;
}
 
void Antonio::run(){
	setupOdomMsg();
	for (;;){
		if (pMotorsAgent != NULL){
			uint32_t timeSinceTwise = to_ms_since_boot (get_absolute_time ()) - xLastTwistTimestamp;
			if (timeSinceTwise > MAX_TWIST_TIME_MS)
            {
				robotStop();
			}
			//updateOdom();
			//publishOdom();
		}
		vTaskDelay(500);
	}
}

configSTACK_DEPTH_TYPE Antonio::getMaxStackSize(){
	return 1024;
}

void Antonio::setupOdomMsg(){
	nav_msgs__msg__Odometry__init(&xOdomMsg);
	if (!rosidl_runtime_c__String__assign(
			&xOdomMsg.header.frame_id, "odom")){
			printf("ERROR: Odom frameID assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(
			&xOdomMsg.child_frame_id, "base_link")){
			printf("ERROR: Odom frameID assignment failed\n");
	}

	//POSE
	xOdomMsg.pose.pose.position.x = 0.0;
	xOdomMsg.pose.pose.position.y = 0.0;
	xOdomMsg.pose.pose.position.z = 0.0;
	xOdomMsg.pose.pose.orientation.x = 0.0;
	xOdomMsg.pose.pose.orientation.y = 0.0;
	xOdomMsg.pose.pose.orientation.z = 0.0;
	xOdomMsg.pose.pose.orientation.w = 0.0;

	//TWIST
	xOdomMsg.twist.twist.linear.x = 0.0;
	xOdomMsg.twist.twist.linear.y = 0.0;
	xOdomMsg.twist.twist.linear.z = 0.0;
	xOdomMsg.twist.twist.angular.x = 0.0;
	xOdomMsg.twist.twist.angular.y = 0.0;
	xOdomMsg.twist.twist.angular.z = 0.0;

}

void Antonio::setupTwistMsg(){
	geometry_msgs__msg__Twist__init(&xTwistMsg);
}

void Antonio::createEntities(rcl_node_t *node, rclc_support_t *support){
	if (pMotorsAgent != NULL){
		pMotorsAgent->createEntities(node, support);
	}
	rclc_publisher_init_default(
			&xPubOdom,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
			"/Antonio/odom");

	rclc_subscription_init_default(
			  &xSubTwist,
			  node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
			  "/cmd_vel");
}

void Antonio::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	if (pMotorsAgent != NULL){
		pMotorsAgent->destroyEntities(node, support);
	}
	rcl_subscription_fini(&xSubTwist, 	node);
}

uint Antonio::getCount(){
	uint res = 2;
	if (pMotorsAgent != NULL){
		res += pMotorsAgent->getCount();
	}
	return res;
}

uint Antonio::getHandles(){
	uint res = 1;
	if (pMotorsAgent != NULL){
		res += pMotorsAgent->getHandles();
	}
	return res;
}

void Antonio::addToExecutor(rclc_executor_t *executor){
	if (pMotorsAgent != NULL){
		pMotorsAgent->addToExecutor(executor);
	}

	buildContext(&xSubTwistContext, NULL);
	rclc_executor_add_subscription_with_context(
			executor,
			&xSubTwist,
			&xTwistMsg,
			uRosEntities::subscriptionCallback,
			&xSubTwistContext,
			ON_NEW_DATA);
}

void Antonio::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){

	if (pMotorsAgent == NULL){
		return;
	}

	if (context == &xSubTwistContext){
		geometry_msgs__msg__Twist * pTwistMsg = (geometry_msgs__msg__Twist *) msg;
		double circum = WHEEL_RADIUS * 2.0 * M_PI;

#ifdef TWIST_DEBUG
		printf("TWIST x: %.3f  rz: %.3f\n",
				pTwistMsg->linear.x,
				pTwistMsg->angular.z
				);
#endif //TWIST_DEBUG

		xLastTwistTimestamp = to_ms_since_boot (get_absolute_time ());

		//Stop
		if (pTwistMsg->linear.x == 0.0 && pTwistMsg->angular.z == 0.0){
			//Have not move linearly to turn
			pMotorsAgent->setSpeed(0, 0.0, true);
			pMotorsAgent->setSpeed(1, 0.0, false);
			return;
		}

		// FWD and Backwards
		if (pTwistMsg->angular.z == 0.0){
			float rps = pTwistMsg->linear.x;
			bool cw = true;
			if (rps < 0.0){
				cw = false;
				rps = rps * -1;
			}
			pMotorsAgent->setSpeed(0, rps, !cw);
			pMotorsAgent->setSpeed(1, rps, cw);

#ifdef TWIST_DEBUG
			printf("LINEAR TWIST %.3f mps becomes %.3f Rad ps\n",
					pTwistMsg->linear.x,
					rps
					);
#endif //TWIST_DEBUG
		} 
        else {
			//ARC
			bool fwd = (pTwistMsg->linear.x > 0.0);
			bool cw = (pTwistMsg->angular.z > 0.0);
			float a = fabs(pTwistMsg->angular.z);
			float arc = a/ (M_PI * 2);
			float fullCircleCircum = (pTwistMsg->linear.x / arc);
			float radius = fullCircleCircum / ( 2.0 * M_PI);

			float speedA = (radius + WHEELS_SEP/4) * (2 * M_PI) * arc;
			float speedB = (radius - WHEELS_SEP/4) * (2 * M_PI) * arc;

			float rpsA = (speedA / circum) * (2 * M_PI);
			float rpsB = (speedB / circum) * (2 * M_PI);

			if (fwd){
				if (!cw){
					pMotorsAgent->setSpeed(0, rpsA, !fwd);
					pMotorsAgent->setSpeed(1, rpsB,  fwd);
				} else {
					pMotorsAgent->setSpeed(0, rpsB, !fwd);
					pMotorsAgent->setSpeed(1, rpsA,  fwd);
				}
			} else {
				if (cw){
					pMotorsAgent->setSpeed(0, rpsA,  fwd);
					pMotorsAgent->setSpeed(1, rpsB, !fwd);
				} else {
					pMotorsAgent->setSpeed(0, rpsB,  fwd);
					pMotorsAgent->setSpeed(1, rpsA, !fwd);
				}
			}

#ifdef TWIST_DEBUG
			printf("ROTATE TWIST %.3f mps at %.3f rad ps "
					"becomes %.3f and %.3f Rad ps\n",
					pTwistMsg->linear.x,
					pTwistMsg->angular.z,
					rpsA,
					rpsB
					);
			printf("ROTATE Detail: Radius %.3f Full Circum %.3f Arc %.3f speed A %.3f B %.3f\n",
					radius,
					fullCircleCircum,
					arc,
					speedA,
					speedB);
#endif //TWIST_DEBUG

		}


	}
}

void Antonio::robotStop(){
	xLastTwistTimestamp = to_ms_since_boot (get_absolute_time ());

	if (pMotorsAgent != NULL){
		pMotorsAgent->setSpeed(0, 0.0, true);
		pMotorsAgent->setSpeed(1, 0.0, false);
	}
}

/*void Antonio::updateOdom(){
	double l = pMotorsAgent->getMotor(0)->getDeltaRadians();
	double r = pMotorsAgent->getMotor(1)->getDeltaRadians();

	l=l*WHEEL_RADIUS;
	r=r*WHEEL_RADIUS* -1.0;

	double avgDist = (r+l)/2.0;
	double angle = asin((r-l)/WHEELS_SEP);
	double deltaX = cos(angle)* avgDist;
	double deltaY = sin(angle)* avgDist;

	xMotorsOdom.x += deltaX;
	xMotorsOdom.y += deltaY;
	xMotorsOdom.a += angle;

	xAntonioOdom.x = xMotorsOdom.x + (cos(angle) * WHEELS_OFFSET);
	xAntonioOdom.y = xMotorsOdom.y + (sin(angle) * WHEELS_OFFSET);
	xAntonioOdom.a = angle;

	uint32_t now = to_ms_since_boot( get_absolute_time()    );
	double seconds = (double)(now - xLastVelocityTime) / 1000;
	xLastVelocityTime = now;
	xAntonioVelocity.x = deltaX /seconds;
	xAntonioVelocity.y = deltaY /seconds;
	xAntonioVelocity.a = angle /seconds;
}*/

/*void Antonio::publishOdom(){
	//Update header
	int64_t time = rmw_uros_epoch_nanos();
	xOdomMsg.header.stamp.sec = time / 1000000000;
	xOdomMsg.header.stamp.nanosec = time % 1000000000;

	//POSE
	xOdomMsg.pose.pose.position.x = xAntonioOdom.x;
	xOdomMsg.pose.pose.position.y = xAntonioOdom.y;
	Quaterniond q;
	Matrix3d m;
	m = AngleAxisd(0.0, 		Vector3d::UnitX())
	  * AngleAxisd(0.0,  		Vector3d::UnitY())
	  * AngleAxisd(xAntonioOdom.a, 	Vector3d::UnitZ());
	q = m;
	xOdomMsg.pose.pose.orientation.x = q.x();
	xOdomMsg.pose.pose.orientation.y = q.y();
	xOdomMsg.pose.pose.orientation.z = q.z();
	xOdomMsg.pose.pose.orientation.w = q.w();

	//TWIST
	xOdomMsg.twist.twist.linear.x 	= xAntonioVelocity.x;
	xOdomMsg.twist.twist.linear.y 	= xAntonioVelocity.y;
	xOdomMsg.twist.twist.angular.z 	= xAntonioVelocity.a;


	if (!uRosBridge::getInstance()->publish(&xPubOdom,
			&xOdomMsg,
			this,
			NULL)){
		printf("Odom Pub failed\n");
	}

}*/

