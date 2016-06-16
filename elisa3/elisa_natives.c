
#include "elisa_natives.h"
#include "irCommunication.h"

AsebaNativeFunctionDescription AsebaNativeDescription_prox_network = {
	"prox.comm.enable",
	"Enable/disable local communication",
	{
		{1, "state"},
		{0,0},
	}
};
		
void prox_network(AsebaVMState * vm) {
	int enable = vm->variables[AsebaNativePopArg(vm)];
	if(enable) {
		irCommInit();
	} else {
		irCommDeinit();
	}
}

AsebaNativeFunctionDescription AsebaNativeDescription_setObstacleAvoidance = {
	"behavior.oa.enable",
	"Enable/disable obstacle avoidance",
	{
		{1, "state"},
		{0,0},
	}
};

void setObstacleAvoidance(AsebaVMState * vm) {
	int enable = vm->variables[AsebaNativePopArg(vm)];
	if(enable) {
		enableObstacleAvoidance();
	} else {
		disableObstacleAvoidance();
	}
}

AsebaNativeFunctionDescription AsebaNativeDescription_setCliffAvoidance = {
	"behavior.cliff.enable",
	"Enable/disable cliff avoidance",
	{
		{1, "state"},
		{0,0},
	}
};

void setCliffAvoidance(AsebaVMState * vm) {
	int enable = vm->variables[AsebaNativePopArg(vm)];
	if(enable) {
		enableCliffAvoidance();
	} else {
		disableCliffAvoidance();
	}
}

// The "calibrateSensors()" function is blocking and thus create problems in the communication with Aseba.
// AsebaNativeFunctionDescription AsebaNativeDescription_calibrate = {
// 	"calibrate",
// 	"Calibrate sensors",
// 	{
// 		{0,0},
// 	}
// };
// 
// void calibrate(AsebaVMState * vm) {
// 	calibrateSensors();	
// }

AsebaNativeFunctionDescription AsebaNativeDescription_resetOdom = {
	"reset.odometry",
	"Reset odometry",
	{
		{0,0},
	}
};

void resetOdom(AsebaVMState * vm) {
	resetOdometry();
}

AsebaNativeFunctionDescription AsebaNativeDescription_isVertical = {
	"robot.isVertical",
	"dest = 1 (vertical) or 0 (horizontal)",
	{
		{-1, "dest" },
		{0,0},
	}
};

void isVertical(AsebaVMState * vm) {
	if(robotPosition == HORIZONTAL_POS) {
		vm->variables[AsebaNativePopArg(vm)] = 0;
	} else {
		vm->variables[AsebaNativePopArg(vm)] = 1;
	}
}

