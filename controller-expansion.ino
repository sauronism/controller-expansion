
#include <Conceptinetics.h>

#define DMX_MASTER_CHANNELS      16 
#define RXEN_PIN                 2

#define LED_DELAY                100
#define DMX_SIGNAL_DELAY         5
#define DMX_SIGNAL_REPEAT        30
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

const boolean WRITE_ALL_DMX_STATE = true;               // writes all channels in one

DMX_Master dmx_master(DMX_MASTER_CHANNELS, RXEN_PIN);

int dmx_channels_state[DMX_MASTER_CHANNELS];


void writeDmx(int chan, int val){
  if (WRITE_ALL_DMX_STATE){
    dmx_channels_state[chan] = val;
  }
  else {
    dmx_master.setChannelValue(chan, val);
  }
}

void postProcessDmxState() {
  if (WRITE_ALL_DMX_STATE){
    // TODO: write all DMX values from tate
  }
}

void resetDmx() {
    for (int i = 0; i < DMX_SIGNAL_REPEAT; i ++) {
      dmx_master.setChannelRange(1, DMX_MASTER_CHANNELS, 0);
      delay(DMX_SIGNAL_DELAY);
    }
}

void initDmx() {
    for (int i = 0; i < DMX_SIGNAL_REPEAT; i++) {
      writeDmx(DIMMER_CHANNEL, DIMMER_MAX_BRIGHTNESS);
      writeDmx(STROBE_CHANNEL, STROBE_OPEN);
      writeDmx(COLOR_CHANNEL, COLOR_WHITE);
      delay(DMX_SIGNAL_DELAY);
    }
}


void setup() {             
  pinMode(DIGIT_1_LED_PIN, OUTPUT);
  pinMode(DIGIT_2_LED_PIN, OUTPUT);
  
  delay(500);
  dmx_master.enable();  
  delay(500);

  resetDmx();
  initDmx();
}

void blink(int val){
    for (int i = 1; i <= val / 10; i++) {
      digitalWrite(DIGIT_2_LED_PIN, HIGH);
      delay(LED_DELAY);
      digitalWrite(DIGIT_2_LED_PIN, LOW);
      delay(LED_DELAY);
    }
    for (int i = 1; i <= val % 10; i++) {
      digitalWrite(DIGIT_1_LED_PIN, HIGH);
      delay(LED_DELAY);
      digitalWrite(DIGIT_1_LED_PIN, LOW);
      delay(LED_DELAY);
    }
}

void loop() 
{
  // initDmx();
  // raster movement in a defined region
  for (int j = 0; j < 100; j++) {
    for (int i = 0; i < DMX_SIGNAL_REPEAT; i ++) {
      writeDmx(PAN_X_AXIS_CHANNEL, j <= 50 ? j : 100 - j);
      delay(DMX_SIGNAL_DELAY);
    }
  }
  postProcessDmxState();
}
