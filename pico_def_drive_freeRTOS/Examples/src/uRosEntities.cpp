/*
 * uRosEntities.cpp
 *
 * `Abstrac
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#include "uRosEntities.h"

uRosEntities::uRosEntities() {
	// TODO Auto-generated constructor stub

}

uRosEntities::~uRosEntities() {
	// TODO Auto-generated destructor stub
}

void uRosEntities::pubComplete(
		void *msg,
		void *args,
		uRosPubStatus status){
	//NOP
}

uint uRosEntities::getHandles(){
	return 0;
}

void uRosEntities::addToExecutor(rclc_executor_t *executor){
	//NOP
}

void uRosEntities::subscriptionCallback(const void* msg, void* context){
	uRosSubContext_t *con = (uRosSubContext_t *) context;
	con->subHandler->handleSubscriptionMsg(msg, con);
}

void uRosEntities::buildContext(uRosSubContext_t *context, void *localContext){
	context->subHandler = this;
	context->localContext = localContext;
}

void uRosEntities::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){
	//NOP
}

