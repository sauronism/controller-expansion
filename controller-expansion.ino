// references
//https://youtu.be/pdFtoTRiQ-M
//https://youtu.be/nPNFbN80pSg

#include <ArduinoJson.h>
#include <DmxSimple.h>


// command constants
#define TRUE   1
#define FALSE  0
#define NO_OP -1

// Beam DMX channels
#define COLOR_CHANNEL           1
#define STROBE_CHANNEL          2
#define DIMMER_CHANNEL          3
#define GOBO_CHANNEL            4
#define FOCUS_CHANNEL           9
#define PAN_X_AXIS_CHANNEL      10
#define TILT_Y_AXIS_CHANNEL     12
#define PAN_TILT_SPEED_CHANNEL  14
#define FROST_CHANNEL           15
#define LAMP_CTRL_RESET_CHANNEL 16

// Beam constants
#define COLOR_WHITE             0
#define STROBE_OPEN             255
#define DIMMER_MAX_BRIGHTNESS   255
#define GOBO_DISABLED           0
#define GOBO_BOBBING_MOTION     80
#define GOBO_SUPER_FOCUSED      60
#define FOCUS_MAX               255
#define PAN_MIN_ANGLE           0
#define PAN_MAX_ANGLE           540
#define TILT_MIN_ANGLE          0
#define TILT_MAX_ANGLE          270
#define FROST_DISABLED          0
#define FROST_BLUR              130
#define LAMP_OFF                100
#define LAMP_ON                 200
#define LAMP_RESET              255

// Smoke machine constants
#define SMOKE_MACHINE_OFF       0
#define SMOKE_MACHINE_ON        255

// configuration
const boolean DEBUG_ENABLED = true;                     // enables debug prints
const boolean BEAM_FROST_ENABLED = true;                // enables an aesthetic fade effect in the edges of the beam
const boolean BEAM_GOBO_ENABLED = true;                 // enables a light bobbing motion of the beam which can be pretty
const boolean MOTOR_CTRL_NON_BLOCKING_ENABLED = true;   // enables sending commands while the motor accelerates/decelerates

// DMX Shield constants
// RX(0) is connected to pin 3 by default (see DMX_RX_PIN) - jumper connected to RX-io
// TX(1) is connected to GND (Arduino is the master)
// DE(2) is connected to 5V - jumper connected to DE
// EN jumper is connceted to EN (default is ~EN)
const int DMX_TX_PIN = 3;                               // default pin is 3, used 7 for Nano
const int DMX_MODE_SELECT_PIN = 2;                      // default pin is 2

const int DMX_MASTER_CHANNELS = 512;                    // default is 512
const int SMOKE_MACHINE_CHANNEL = 22;

// Motor constants
// R_EN and L_EN connected to VCC (IBT-2 pins 3 & 4)
// L_IS and R_IS not connected    (IBT-2 pins 5 & 6)
const int MOTOR_RPWM_OUTPUT = 5;                        // Arduino PWM output pin D9; connect to IBT-2 pin 1 (RPWM)
const int MOTOR_LPWM_OUTPUT = 6;                        // Arduino PWM output pin D10; connect to IBT-2 pin 2 (LPWM)
const int MOTOR_MAX_PWM_VALUE = 255;                    // Adjust this delay to control the maximum speed of the motor
const int MOTOR_ACCELERATION_DELAY_MILLIS = 30;         // Adjust this delay to control the speed of acceleration/deceleration

// motor state
short motorCurrentPwmValue = 0;                         // tracking the current PWM value to ensure smooth transitions in case of duplicate commands
short motorTargetPwmValue = 0;                          // allowing small incremets with short delays in a non blocking manner until the desired state is reached

typedef StaticJsonDocument<256> Packet;

typedef struct {
	short motor_on;
	short smoke_on;
	short beam_on;
	short beam_x;
	short beam_y;
	short beam_speed;  
} cmd_t;

short getJsonProperty(Packet pkt, char* propertyName) {
	return pkt.containsKey(propertyName) ? pkt[propertyName].as<short>() : NO_OP;
}

cmd_t parseCommand(Packet pkt) {
	cmd_t command;

	command.motor_on = getJsonProperty(pkt, "m");
	command.smoke_on = getJsonProperty(pkt, "s");
	command.beam_on = getJsonProperty(pkt, "b");
	command.beam_x = getJsonProperty(pkt, "x");
	command.beam_y = getJsonProperty(pkt, "y");
	command.beam_speed = getJsonProperty(pkt, "v");

	if (DEBUG_ENABLED) {
		char msg[256];
		sprintf(msg,
			"Command:\nmotor_on: %d \nsmoke_on: %d \nbeam_on: %d \nbeam_x: %d \nbeam_y: %d \nbeam_speed: %d \n",
			command.motor_on,
			command.smoke_on,
			command.beam_on,
			command.beam_x,
			command.beam_y,
			command.beam_speed);
		Serial.println(msg);
	}
	return command;
}

boolean readCommandJson(Packet& pkt) {
	if (Serial.available()) {
		String pktString = Serial.readStringUntil('\0');    
		if (DEBUG_ENABLED) {
			Serial.println(pktString);
		}

		DeserializationError error = deserializeJson(pkt, pktString);
		if (error) {
			if (DEBUG_ENABLED) {
				Serial.print("Error parsing JSON: ");
				Serial.println(error.c_str());
			}
		} else {
			return true;
		}
	}
	return false;
}

void accelerateMotorSingleStep() {
	short delta = (motorCurrentPwmValue > motorTargetPwmValue) ? -1 : 1;
	motorCurrentPwmValue += delta;
	analogWrite(MOTOR_RPWM_OUTPUT, motorCurrentPwmValue);
	delay(MOTOR_ACCELERATION_DELAY_MILLIS); 
	if (DEBUG_ENABLED) {
		Serial.print(motorCurrentPwmValue);
		Serial.print("/");
		Serial.println(motorTargetPwmValue);
	}
}

void accelerateMotor() {
	if (MOTOR_CTRL_NON_BLOCKING_ENABLED) {
		return;
	}
	while (motorCurrentPwmValue != motorTargetPwmValue) {
		accelerateMotorSingleStep();
	}
}

void startMotor() {
	motorTargetPwmValue = MOTOR_MAX_PWM_VALUE;
	accelerateMotor();
}

void stopMotor() {
	motorTargetPwmValue = 0;
	accelerateMotor();
}

short normalizeValue(short value, short maxValue) {
	short normalized = map(value, 0, maxValue, 0, 255);
	return normalized;
}

void executeCommand(cmd_t command) {
	if (command.motor_on == TRUE) {
		startMotor();
	}
	if (command.motor_on == FALSE) {
		stopMotor();
	}
	if (command.smoke_on == TRUE) {
		DmxSimple.write(SMOKE_MACHINE_CHANNEL, SMOKE_MACHINE_ON);
	}
	if (command.smoke_on == FALSE) {
		DmxSimple.write(SMOKE_MACHINE_CHANNEL, SMOKE_MACHINE_OFF);
	}
	if (command.beam_on == TRUE) {
		DmxSimple.write(LAMP_CTRL_RESET_CHANNEL, LAMP_RESET); 
		// DmxSimple.write(LAMP_CTRL_RESET_CHANNEL, LAMP_ON); // TODO: lamp doesn't respond to ON, only reset, even manually
		DmxSimple.write(DIMMER_CHANNEL, DIMMER_MAX_BRIGHTNESS);
	}
	if (command.beam_on == FALSE) {
		DmxSimple.write(LAMP_CTRL_RESET_CHANNEL, LAMP_OFF);
		DmxSimple.write(DIMMER_CHANNEL, 0);
	}
	if (command.beam_speed != NO_OP) {
		DmxSimple.write(PAN_TILT_SPEED_CHANNEL, command.beam_speed);
	}
	if (command.beam_x != NO_OP) {
		short normalizedVal = normalizeValue(command.beam_x, PAN_MAX_ANGLE);
		DmxSimple.write(PAN_X_AXIS_CHANNEL, normalizedVal);
	}
	if (command.beam_y != NO_OP) {
		short normalizedVal = normalizeValue(command.beam_y, TILT_MAX_ANGLE);
		DmxSimple.write(PAN_X_AXIS_CHANNEL, normalizedVal);
	}
}

void initMotor() {
	pinMode(MOTOR_RPWM_OUTPUT, OUTPUT);
	pinMode(MOTOR_LPWM_OUTPUT, OUTPUT);
	analogWrite(MOTOR_LPWM_OUTPUT, 0);
}

void initDmx() {
	pinMode(DMX_MODE_SELECT_PIN, OUTPUT);
	delay(10);
	digitalWrite(DMX_MODE_SELECT_PIN, HIGH);
	DmxSimple.maxChannel(DMX_MASTER_CHANNELS);
	DmxSimple.usePin(DMX_TX_PIN); 
}

void initBeam() {
	DmxSimple.write(LAMP_CTRL_RESET_CHANNEL, LAMP_RESET);
	DmxSimple.write(COLOR_CHANNEL, COLOR_WHITE);
	DmxSimple.write(STROBE_CHANNEL, STROBE_OPEN);
	DmxSimple.write(GOBO_CHANNEL, BEAM_GOBO_ENABLED ? GOBO_DISABLED : GOBO_BOBBING_MOTION);
	DmxSimple.write(FOCUS_CHANNEL, FOCUS_MAX);
	DmxSimple.write(FROST_CHANNEL, BEAM_FROST_ENABLED ? FROST_DISABLED : FROST_BLUR);
}

void setup() {
	initMotor();
	initDmx();
	initBeam();

	delay(500);
	Serial.begin(9600);
	Serial.println("Setup complete.");
}

void test() {
	const int DMX_CMD_DELAY_MILLIS = 100;
	for (int value = 0; value <= 255; value++) {
		DmxSimple.write(PAN_X_AXIS_CHANNEL, value);
		DmxSimple.write(TILT_Y_AXIS_CHANNEL, value);
		delay(DMX_CMD_DELAY_MILLIS); 
	}
}

void executeNonBlockingOperations() {
	if (MOTOR_CTRL_NON_BLOCKING_ENABLED && (motorCurrentPwmValue != motorTargetPwmValue)) {
		accelerateMotorSingleStep();
	}
}

void dmxTest() {
	if (!DEBUG_ENABLED) {
		return;
	}
	
	for (int val = 0; val < 256; val += 127) {
		delay(1000);
		Serial.println(val);
		for (int chan = 1; chan <= DMX_MASTER_CHANNELS; chan++) {
			DmxSimple.write(chan, val);  
		}
	}
}

void readAndExecuteCommand() {
	Packet pkt;
	int success = readCommandJson(pkt);
	if (success) {
		cmd_t command = parseCommand(pkt);
		executeCommand(command);
	}
	executeNonBlockingOperations();
}

void loop() {
  readAndExecuteCommand();
//   dmxTest();
}
