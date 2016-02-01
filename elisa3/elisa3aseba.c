/*
	Aseba - an event-based framework for distributed robot control
	Copyright (C) 2007--2010:
		Stephane Magnenat <stephane at magnenat dot net>
		(http://stephane.magnenat.net)
		and other contributors, see authors.txt for details
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, version 3 of the License.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
	WARNING: you have to change the size of the UART 1 e-puck reception buffer to hold
	at the biggest possible packet (probably bytecode + header). Otherwise if you set
	a new bytecode while busy (for instance while sending description), you might end up
	in a dead-lock.
*/

#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include "utility.h"

#include <string.h>

#define ASEBA_ASSERT
#include <vm/vm.h>
#include <common/consts.h>
#include <vm/natives.h>
#include <transport/buffer/vm-buffer.h>
#include "elisa_natives.h"
#include "variables.h"

#define vmVariablesSize (sizeof(struct EPuckVariables) / sizeof(sint16))
#define vmStackSize 32
#define argsSize 32

#define CLAMP(v, vmin, vmax) ((v) < (vmin) ? (vmin) : (v) > (vmax) ? (vmax) : (v))

#define LEFT 0
#define RIGHT 1

/*
History:
0: First production firmware
*/
#define FW_VERSION 0


// data

/*** In your code, put "SET_EVENT(EVENT_NUMBER)" when you want to trigger an 
	 event. This macro is interrupt-safe, you can call it anywhere you want.
***/
#define SET_EVENT(event) do {events_flags |= 1 << event;} while(0)
#define CLEAR_EVENT(event) do {events_flags &= ~(1 << event);} while(0)
#define IS_EVENT(event) (events_flags & (1 << event))

/* VM */

/* The number of opcode an aseba script can have */
#define VM_BYTECODE_SIZE 1024
#define VM_STACK_SIZE 32

int EEMEM bytecode_version;
unsigned char EEMEM eeprom_bytecode[VM_BYTECODE_SIZE*2];

struct Elisa3Variables
{
	// NodeID
	sint16 id;
	// source
	sint16 source;
	// args
	sint16 args[argsSize];
	// fwversion
	sint16 fwversion;
	
	// ACTUATORS
	// motors (range is -127..127, resolution is 5 mm/s)
	sint16 targetSpeed[2];
	sint16 measSpeed[2];
	// green leds
	sint16 greenLeds[8];
	// rgb leds
	sint16 rgbLeds[3];	//0=red, 1=green, 2=blue
	// IR transmitters
	sint16 irTxFront;
	sint16 irTxBack;
	// button
	sint16 button;
	
	// SENSORS
	// prox
	sint16 prox[8];
	sint16 proxAmbient[8];
	// ground
	sint16 ground[4];
	sint16 groundAmbient[4];
	// acc
	sint16 acc[3];
	// selector
	sint16 selector;
	// tv remote
	sint16 tvRemote;
	// battery (adc, percentage)
	sint16 batteryAdc;
	sint16 batteryPercent;
	
	// local communication
	sint16 irRxData;
	sint16 irTxData;

//	// Charge state (0 => robot not in charge; 1 => robot in charge).
//	// Only meaningful if radio communication is used (when cable attached the state is always in charge).
//	sint16 chargeState;

	// Odometry.
	sint16 thetaDeg;
	sint16 xPosMm;
	sint16 yPosMm;

	// timer
	sint16 timer;
	
	// Free space, reserved for user variables in the script.
	sint16 freeSpace[100];
	
} elisa3Variables;

char name[] = "elisa3-0";

AsebaVMDescription vmDescription = {
	name, 	// Name of the microcontroller
	{
		{ 1, "id" },			// Do not touch it
		{ 1, "source" }, 		// nor this one
		{ argsSize, "args" },	// neither this one
		{1, "_fwver"},
		{1, "motor.left.target"},
		{1, "motor.right.target"},
		{1, "motor.left.speed"},
		{1, "motor.right.speed"},
		{8, "led.green"},
		{3, "led.rgb"},
		{1, "ir.tx.front"},
		{1, "ir.tx.back"},
		{1, "button"},
	    {8, "prox"},
		{8, "prox.ambient"},
		{4, "ground"},
		{4, "ground.ambient"},
		{3, "acc"},
		{1, "selector"},
		{1, "rc5"},
		{1, "_bat.adc"},
		{1, "bat.percent"},
		{1, "prox.comm.rx"},
		{1, "prox.comm.tx"},
		{1, "odom.theta"},
		{1, "odom.x"},
		{1, "odom.y"},
//		{1, "charge"},
		{1, "timer.period"},
		{ 0, NULL }				// null terminated
	}
};

static uint16 vmBytecode[VM_BYTECODE_SIZE];

static sint16 vmStack[VM_STACK_SIZE];

static AsebaVMState vmState = {
	0x1,
	
	VM_BYTECODE_SIZE,
	vmBytecode,
	
	sizeof(elisa3Variables) / sizeof(sint16),
	(sint16*)&elisa3Variables,

	VM_STACK_SIZE,
	vmStack
};

const AsebaVMDescription* AsebaGetVMDescription(AsebaVMState *vm)
{
	return &vmDescription;
}	

static unsigned int events_flags = 0;
enum Events
{
	EVENT_IR_SENSORS = 0,
	EVENT_ACC,
	EVENT_BUTTON,
	EVENT_DATA,
	EVENT_RC5,
	EVENT_SELECTOR,
	EVENT_TIMER,
//	EVENT_CHARGE,
	EVENTS_COUNT
};

static const AsebaLocalEventDescription localEvents[] = { 
	{"ir.sensors", "Proximity and ground sensors updated"},
	{"acc", "Accelerometer values updated"},
	{"button", "Button status changed"},
	{"prox.comm", "Data received on local communication"},
	{"rc5", "RC5 message received"},
	{"sel", "Selector status changed"},
//	{"charge", "Charge status changed"},
	{"timer", "Timer"},
	{ NULL, NULL }
};

const AsebaLocalEventDescription * AsebaGetLocalEventsDescriptions(AsebaVMState *vm)
{
	return localEvents;
}


static const AsebaNativeFunctionDescription* nativeFunctionsDescription[] = {
	ASEBA_NATIVES_STD_DESCRIPTIONS,
	ELISA_NATIVES_DESCRIPTIONS,
	0	// null terminated
};

static AsebaNativeFunctionPointer nativeFunctions[] = {
	ASEBA_NATIVES_STD_FUNCTIONS,
	ELISA_NATIVES_FUNCTIONS
};

void AsebaPutVmToSleep(AsebaVMState *vm)
{
}

void AsebaNativeFunction(AsebaVMState *vm, uint16 id)
{
	nativeFunctions[id](vm);
}

const AsebaNativeFunctionDescription * const * AsebaGetNativeFunctionsDescriptions(AsebaVMState *vm)
{
	return nativeFunctionsDescription;
}	

void uartSendUInt8(uint8 value)
{
	usart0Transmit(value, 0);
}

void uartSendUInt16(uint16 value)
{
	usart0Transmit(value&0xFF, 0);
	usart0Transmit((value>>8)&0xFF, 0);
}

void AsebaSendBuffer(AsebaVMState *vm, const uint8* data, uint16 length)
{
	uartSendUInt16(length - 2);
	uartSendUInt16(vmState.nodeId);
	uint16 i;
	for (i = 0; i < length; i++)
		uartSendUInt8(*data++);
}	

uint8 uartGetUInt8()
{
	unsigned char c=1;
	unsigned int i=0;

	while(c) {

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {	// handle concurrent byteCount access
			if(byteCount > 0) {				// (accessed here and within ISR rx interrupt)
				c=0;
			}
		}
		i++;
		if(i>100) {							// timeout
			commError=1;
			return 0;
		}
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {		// handle concurent uartBuff and byteCount access
		c = uartBuff[currByteIndex];
		byteCount--;
	}
	currByteIndex++;
	if(currByteIndex==UART_BUFF_SIZE) {		// circular buffer
		currByteIndex = 0;
	}

	return c;
}

uint16 uartGetUInt16()
{
	uint16 value;
	// little endian
	value = uartGetUInt8();
	if(commError) {
		return 0;
	}
	value |= (uartGetUInt8() << 8);
	if(commError) {
		return 0;
	}
	return value;
}

uint16 AsebaGetBuffer(AsebaVMState *vm, uint8* data, uint16 maxLength, uint16* source)
{

	uint16 ret = 0;

	uint16 len = uartGetUInt16() + 2;	// msg type + data

	if(commError) {
		commError = 0;
		return 0;
	}
	if(len <= 2) {
		return 0;
	}

	*source = uartGetUInt16();
	if(commError) {
		commError = 0;
		return 0;
	}	
	
	uint16 i;
	for (i = 0; i < len; i++) {
		data[i] = uartGetUInt8();
		if(commError) {
			commError = 0;
			return 0;
		}
	}
	
	ret = len;

	return ret;

}	

void updateRobotVariables() {

	unsigned i;
	static int accState = 1;
	static uint32_t batteryTick = 0;
	static char btnState = -1;
	static char selectorState = -1;
//	static char chargeState = -1;
	static uint32_t timerTick = 0;

	// motor
	static int leftSpeed = 0, rightSpeed = 0;

	if (elisa3Variables.targetSpeed[LEFT] != leftSpeed) {
		leftSpeed = CLAMP(elisa3Variables.targetSpeed[LEFT], -127, 127);
		setLeftSpeed(leftSpeed);
	}
	if (elisa3Variables.targetSpeed[RIGHT] != rightSpeed) {
		rightSpeed = CLAMP(elisa3Variables.targetSpeed[RIGHT], -127, 127);
		setRightSpeed(rightSpeed);
	}
	elisa3Variables.measSpeed[LEFT] = speedLeftFromEnc/5;	// Divide by 5 to get the same scale as target speed (1 unit = 5 mm/s).
	elisa3Variables.measSpeed[RIGHT] = speedRightFromEnc/5;
	handleMotorsWithSpeedController();

	if(proxUpdated) {
		proxUpdated = 0;
		// leds and prox
		for (i = 0; i < 8; i++) {
			setGreenLed(i, elisa3Variables.greenLeds[i] ? 1 : 0);
			elisa3Variables.proxAmbient[i] = proximityValue[i*2];
			elisa3Variables.prox[i] =  proximityResultLinear[i];
		}
		for(i=0; i<4; i++) {
			elisa3Variables.groundAmbient[i] = proximityValue[(i+8)*2];
			elisa3Variables.ground[i] = proximityResult[i+8];
		}
		SET_EVENT(EVENT_IR_SENSORS);
	}
	
	// read acc
	// I2C communication isn't time constrained, thus split the data reception in two in
	// order to avoid big delays
	if(accState) {
		readAccelXYZ_1();
	} else {
		readAccelXYZ_2();
		elisa3Variables.acc[0] = accX;
		elisa3Variables.acc[1] = accY;
		elisa3Variables.acc[2] = accZ;
		SET_EVENT(EVENT_ACC);
		computeAngle();
		elisa3Variables.thetaDeg = (signed int)(theta*RAD_2_DEG);
		elisa3Variables.xPosMm = (signed int)xPos;
		elisa3Variables.yPosMm = (signed int)yPos;
	}
	accState = 1 - accState;

	// rgb leds
	updateRedLed(255-CLAMP(elisa3Variables.rgbLeds[0], 0, 255));
	updateGreenLed(255-CLAMP(elisa3Variables.rgbLeds[1], 0, 255));
	updateBlueLed(255-CLAMP(elisa3Variables.rgbLeds[2], 0, 255));

	// selector
	elisa3Variables.selector = getSelector();
	if(selectorState != elisa3Variables.selector) {
		SET_EVENT(EVENT_SELECTOR);
	}
	selectorState = elisa3Variables.selector;

	// ir transmitters
	if(elisa3Variables.irTxFront) {
		LED_IR2_LOW;
	} else {
		LED_IR2_HIGH;
	}
	if(elisa3Variables.irTxBack) {
		LED_IR1_LOW;
	} else {
		LED_IR1_HIGH;
	}

	if(command_received) {
		elisa3Variables.tvRemote = ir_remote_get_data();
		command_received = 0;
		SET_EVENT(EVENT_RC5);
	}
	
	elisa3Variables.button = BUTTON0;
	if(btnState != elisa3Variables.button) {
		SET_EVENT(EVENT_BUTTON);
	}
	btnState = elisa3Variables.button;
	
	if((getTime100MicroSec()-batteryTick) >= (PAUSE_2_SEC)) {
		readBatteryLevel();				// The battery level is updated every two seconds.
		elisa3Variables.batteryAdc = batteryLevel;
		if(batteryLevel >= 934) {           // 934 is the measured adc value when the battery is charged.
			elisa3Variables.batteryPercent = 100;
		} else if(batteryLevel <= 780) {    // 780 is the measrued adc value when the battery is discharged.
			elisa3Variables.batteryPercent = 0;
		} else {
			elisa3Variables.batteryPercent = (unsigned int)(((sint32)batteryLevel-(sint32)780.0)*(sint32)100/(sint32)154);
		}
		batteryTick = getTime100MicroSec();
	}
	
	if(irCommEnabled != IRCOMM_MODE_SENSORS_SAMPLING) {
		irCommTasks();
		if(irCommDataSent()==1) {
			irCommSendData((unsigned char)elisa3Variables.irTxData);
		}
		if(irCommDataAvailable()==1) {
			elisa3Variables.irRxData = irCommReadData();
			SET_EVENT(EVENT_DATA);
		}
	}

	if(elisa3Variables.timer > 0) {
		if(((getTime100MicroSec()-timerTick)/10) >= elisa3Variables.timer) {	// This is divided by 10 to get about 1 ms.
			SET_EVENT(EVENT_TIMER);
			timerTick = getTime100MicroSec();
		}
	}

// 	elisa3Variables.chargeState = CHARGE_ON;
// 	if(chargeState != elisa3Variables.chargeState) {
// 		SET_EVENT(EVENT_CHARGE);
// 	}

}



void AsebaAssert(AsebaVMState *vm, AsebaAssertReason reason)
{
	turnOnGreenLeds();
	updateRedLed(0);
	updateGreenLed(255);
	updateBlueLed(255);
	while (1);
}

void initRobot() {
	initPeripherals();
}


void AsebaWriteBytecode(AsebaVMState *vm) {

	turnOnGreenLeds();

	int i=0;
	uint16_t* EE_addr = (uint16_t*)&bytecode_version;

	i = ASEBA_PROTOCOL_VERSION;

	eeprom_write_word(EE_addr,i);

	EE_addr = (uint16_t*)eeprom_bytecode;

	for(i = 0; i < 2048; i+=2) {
		
		eeprom_write_word(EE_addr, vm->bytecode[i/2]);
		EE_addr ++;
	}

	turnOffGreenLeds();

}
void AsebaResetIntoBootloader(AsebaVMState *vm) {
	asm("jmp 0x0"); // no reset instruction
}

void initAseba() {
	// VM
	int selector = getSelector();
	vmState.nodeId = selector + 1;
	AsebaVMInit(&vmState);
	elisa3Variables.id = selector + 1;
	elisa3Variables.rgbLeds[0] = 0;
	elisa3Variables.rgbLeds[1] = 0;
	elisa3Variables.rgbLeds[2] = 0;
	name[7] = '0' + selector;
	turnOffGreenLeds();
	elisa3Variables.fwversion = FW_VERSION;
}

int main()
{	
	int i=0;
	
	initRobot();

	initAseba();

	calibrateSensors();
	

	uint16_t* EE_addr = (uint16_t*)&bytecode_version;
	i = eeprom_read_word(EE_addr);

	// ...only load bytecode if version is the same as current one
	if(i == ASEBA_PROTOCOL_VERSION)
	{
		EE_addr = (uint16_t*)eeprom_bytecode;
			
		for( i = 0; i< 2048; i += 2) {
			vmState.bytecode[i/2] = eeprom_read_word(EE_addr);
			EE_addr++;
		}
		
		// Init the vm
		AsebaVMSetupEvent(&vmState, ASEBA_EVENT_INIT);	
	}
	

	if(MCUSR & (1 << 0)) {	// if power on reset does nothing...wait for the serial connection to be opened
		MCUSR &= ~(1<<0);	// clear flag
	}

	if(MCUSR & (1 << 1)) {	// external reset event (caused by the serial connection opened) => send description to aseba
		MCUSR &= ~(1<<1);
		AsebaSendDescription(&vmState);
	}

	while (1) {

		AsebaProcessIncomingEvents(&vmState);
		updateRobotVariables();
		AsebaVMRun(&vmState, 100);

		if (AsebaMaskIsClear(vmState.flags, ASEBA_VM_STEP_BY_STEP_MASK) || AsebaMaskIsClear(vmState.flags, ASEBA_VM_EVENT_ACTIVE_MASK))
		{
			unsigned i=0, k=0;

			// Find first bit from right (LSB) 
			for(k=0; k<EVENTS_COUNT; k++) {
				if(IS_EVENT(k)) {
					i=k+1;
					break;
				}
			}

			if(i)
			{
				i--;
				CLEAR_EVENT(i);
				elisa3Variables.source = vmState.nodeId;
				AsebaVMSetupEvent(&vmState, ASEBA_EVENT_LOCAL_EVENTS_START - i);
			}

		}

	}
	
	return 0;
}

