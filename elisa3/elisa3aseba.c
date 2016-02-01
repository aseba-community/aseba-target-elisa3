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
#include "variables.h"
#include "utility.h"

#include <string.h>

#define ASEBA_ASSERT
#include <vm/vm.h>
#include <common/consts.h>
#include <vm/natives.h>
#include <transport/buffer/vm-buffer.h>

#define vmVariablesSize (sizeof(struct EPuckVariables) / sizeof(sint16))
#define vmStackSize 32
#define argsSize 32

// we receive the data as big endian and read them as little, so we have to acces the bit the right way
#define CAM_RED(pixel) ( 3 * (((pixel) >> 3) & 0x1f))

#define CAM_GREEN(pixel) ((3 * ((((pixel) & 0x7) << 3) | ((pixel) >> 13))) >> 1)

#define CAM_BLUE(pixel) ( 3 * (((pixel) >> 8) & 0x1f))

#define CLAMP(v, vmin, vmax) ((v) < (vmin) ? (vmin) : (v) > (vmax) ? (vmax) : (v))

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
	// ACTUATORS
	// motor
	sint16 leftSpeed;
	sint16 rightSpeed;
	// green leds
	sint16 greenLeds[8];
	// rgb leds
	sint16 rgbLeds[3];	//0=red, 1=green, 2=blue
	// IR transmitters
	sint16 irFront;
	sint16 irBack;
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
	sint16 irCmd ;
	// free space
	sint16 freeSpace[128];
} elisa3Variables;

char name[] = "elisa3-0";

AsebaVMDescription vmDescription = {
	name, 	// Name of the microcontroller
	{
		{ 1, "id" },			// Do not touch it
		{ 1, "source" }, 		// nor this one
		{ argsSize, "args" },	// neither this one
		{1, "leftSpeed"},
		{1, "rightSpeed"},
		{8, "greenLeds"},
		{3, "rgbLeds"},
		{1, "irFront"},
		{1, "irBack"},
	    {8, "prox"},
		{8, "proxAmbient"},
		{4, "ground"},
		{4, "groundAmbient"},
		{3, "acc"},
		{1, "selector"},
		{1, "irCmd"},
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
	EVENT_ACCELEROMETER,
	EVENTS_COUNT
};

static const AsebaLocalEventDescription localEvents[] = { 
	{"ir_sensors", "New IR (prox and ground) sensors values available"},
	{"accelerometer", "New accelerometer values available"},
	{ NULL, NULL }
};

const AsebaLocalEventDescription * AsebaGetLocalEventsDescriptions(AsebaVMState *vm)
{
	return localEvents;
}


static const AsebaNativeFunctionDescription* nativeFunctionsDescription[] = {
	ASEBA_NATIVES_STD_DESCRIPTIONS,
	0	// null terminated
};

static AsebaNativeFunctionPointer nativeFunctions[] = {
	ASEBA_NATIVES_STD_FUNCTIONS,
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

	// motor
	static int leftSpeed = 0, rightSpeed = 0;

	if (elisa3Variables.leftSpeed != leftSpeed) {
		leftSpeed = CLAMP(elisa3Variables.leftSpeed, -100, 100);
		setLeftSpeed(leftSpeed);
	}
	if (elisa3Variables.rightSpeed != rightSpeed) {
		rightSpeed = CLAMP(elisa3Variables.rightSpeed, -100, 100);
		setRightSpeed(rightSpeed);
	}
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
		SET_EVENT(EVENT_ACCELEROMETER);
	}
	accState = 1 - accState;

	// rgb leds
	updateRedLed(CLAMP(elisa3Variables.rgbLeds[0], 0, 255));
	updateGreenLed(CLAMP(elisa3Variables.rgbLeds[1], 0, 255));
	updateBlueLed(CLAMP(elisa3Variables.rgbLeds[2], 0, 255));

	// selector
	elisa3Variables.selector = getSelector();

	// ir transmitters
	if(elisa3Variables.irFront) {
		LED_IR2_LOW;
	} else {
		LED_IR2_HIGH;
	}
	if(elisa3Variables.irBack) {
		LED_IR1_LOW;
	} else {
		LED_IR1_HIGH;
	}

	if(command_received) {
		elisa3Variables.irCmd = ir_remote_get_data();
		command_received = 0;
	}

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

	// init stuff
	cli();			// disable global interrupts (by default it should already be disabled)
	
	// reset all registers touched by arduino in the "init()" functions (wiring.c) not used by the robot
	TCCR0A = 0;
	TCCR0B = 0;
	TIMSK0 = 0;
	TCCR5A = 0;
	TCCR5B = 0;

	rfAddress = eeprom_read_word((uint16_t*)4094);

	// some code parts change based on hardware revision
	if(rfAddress >= 3201 && rfAddress <= 3203) {
		hardwareRevision = HW_REV_3_0;
	}

	if(rfAddress == 3200) {
		hardwareRevision = HW_REV_3_0_1;
	}

	if(rfAddress > 3203) {
		hardwareRevision = HW_REV_3_1;
	}

	initPortsIO();
	initAdc();
	initMotors();
	initRGBleds();
	initUsart0();
	initAccelerometer();
	init_ir_remote_control();

	sei();			// enable global interrupts

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
	elisa3Variables.rgbLeds[0] = 255;
	elisa3Variables.rgbLeds[1] = 255;
	elisa3Variables.rgbLeds[2] = 255;
	name[7] = '0' + selector;
	turnOffGreenLeds();

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

