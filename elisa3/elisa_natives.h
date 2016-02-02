
#ifndef _ELISA_NATIVES_H_
#define _ELISA_NATIVES_H_

#include <vm/vm.h>
#include <vm/natives.h>

extern AsebaNativeFunctionDescription AsebaNativeDescription_prox_network;
void prox_network(AsebaVMState *vm);
// extern AsebaNativeFunctionDescription AsebaNativeDescription_calibrate;
// void calibrate(AsebaVMState *vm);
extern AsebaNativeFunctionDescription AsebaNativeDescription_setObstacleAvoidance;
void setObstacleAvoidance(AsebaVMState *vm);
extern AsebaNativeFunctionDescription AsebaNativeDescription_setCliffAvoidance;
void setCliffAvoidance(AsebaVMState *vm);
extern AsebaNativeFunctionDescription AsebaNativeDescription_resetOdom;
void resetOdom(AsebaVMState *vm);
extern AsebaNativeFunctionDescription AsebaNativeDescription_isVertical;
void isVertical(AsebaVMState *vm);

#define ELISA_NATIVES_DESCRIPTIONS \
	&AsebaNativeDescription_prox_network, \
	&AsebaNativeDescription_setObstacleAvoidance, \
	&AsebaNativeDescription_setCliffAvoidance, \
	&AsebaNativeDescription_resetOdom, \
	&AsebaNativeDescription_isVertical
	//&AsebaNativeDescription_calibrate
		
#define ELISA_NATIVES_FUNCTIONS \
	prox_network, \
	setObstacleAvoidance, \
	setCliffAvoidance, \
	resetOdom, \
	isVertical
	//calibrate

#endif

