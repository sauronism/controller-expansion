
#include <Arduino.h>
#include "avr/wdt.h"
#include <DmxSimple.h>


/*
 * Arduino UNO + KeyeStudio DMX Shield
 * Jumper settings:

  EN
  DE
  TX-UART
  RX-IO - Actually don't care, but this allows flashing the device

 * */

#define DMX_MASTER_CHANNELS      16
#define RXEN_PIN                 2

#define LED_DELAY                100
#define DMX_SIGNAL_DELAY         3
#define DMX_SIGNAL_REPEAT        100
#define DMX_SIGNAL_VALUE_DELTA   10

#define DIGIT_1_LED_PIN          13
#define DIGIT_2_LED_PIN          A0

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
#define DIMMER_MIN_BRIGHTNESS   50
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

const boolean BEAM_FROST_ENABLED = true;                // enables an aesthetic fade effect in the edges of the beam
const boolean BEAM_GOBO_ENABLED = true;                 // enables a light bobbing motion of the beam which can be pretty

const bool WRITE_ALL_DMX_STATE = false;               // writes all channels in one


typedef struct BeamState {
  byte color = 0;
  byte strobe = 0;
  byte dimmer = 0;
  byte gobo = 0;
  byte focus = 0;
  byte pan = 0;
  byte tilt = 0;
  byte pan_tilt_speed = 0;
  byte frost = 0;
  byte lamp_ctrl_reset = 0;
} BeamState;


void setup() {
  /* The most common pin for DMX output is pin 3, which DmxSimple
  ** uses by default. If you need to change that, do it here. */
  Serial.begin(9600);
  DmxSimple.usePin(10);
  pinMode(RXEN_PIN, OUTPUT);
  digitalWrite(RXEN_PIN, HIGH);

  /* DMX devices typically need to receive a complete set of channels
  ** even if you only need to adjust the first channel. You can
  ** easily change the number of channels sent here. If you don't
  ** do this, DmxSimple will set the maximum channel number to the
  ** highest channel you DmxSimple.write() to. */
  DmxSimple.maxChannel(20);
  Serial.print("DMX initialized\n");
}


void writeDmxValue(const BeamState &state) {
  DmxSimple.write(COLOR_CHANNEL, state.color);
  DmxSimple.write(STROBE_CHANNEL, state.strobe);
  DmxSimple.write(DIMMER_CHANNEL, state.dimmer);
  DmxSimple.write(GOBO_CHANNEL, state.gobo);
  DmxSimple.write(FOCUS_CHANNEL, state.focus);
  DmxSimple.write(PAN_X_AXIS_CHANNEL, state.pan);
  DmxSimple.write(TILT_Y_AXIS_CHANNEL, state.tilt);
  DmxSimple.write(PAN_TILT_SPEED_CHANNEL, state.pan_tilt_speed);
  DmxSimple.write(FROST_CHANNEL, state.frost);
  DmxSimple.write(LAMP_CTRL_RESET_CHANNEL, state.lamp_ctrl_reset);
}


void loop() {
  wdt_reset();

  static BeamState state;
  for (auto i = 0; i < 256; i++) {
    state.color = 0;
    state.strobe = STROBE_OPEN;
    state.dimmer = DIMMER_MIN_BRIGHTNESS;
    state.gobo = GOBO_DISABLED;
    state.focus = 0;
    state.pan = 30;
    state.tilt = 0;
    state.pan_tilt_speed = 10;
    state.frost = 10;
    state.lamp_ctrl_reset = LAMP_ON;
  }
  writeDmxValue(state);
  delay(500);
  // raster movement in a defined region
//  for (int j = 0; j < 100; j++) {
//    for (int i = 0; i < DMX_SIGNAL_REPEAT; i ++) {
//      writeDmx(PAN_X_AXIS_CHANNEL, j <= 50 ? j : 100 - j);
//      delay(DMX_SIGNAL_DELAY);
//    }
//  }
//  postProcessDmxState();
}