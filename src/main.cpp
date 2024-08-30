
#include <Arduino.h>
#include "avr/wdt.h"
#include <DmxSimple.h>
#include "ArduinoJson.hpp"
#include "FastLED.h"

/*
 * Arduino UNO + KeyeStudio DMX Shield
 * Jumper settings:

  EN
  DE
  TX-UART
  RX-IO - Actually don't care, but this allows flashing the device

 * */

/*
  Beam Channels:
  - 1: Color
  - 2: Strobe
  - 3: Dimmer
  - 4: Gobo
  - 5: Prism 1 Insertion
  - 6: Prism 2 Insertion
  - 7: Prism 3 Insertion
  - 8: Prism 4 Insertion
  - 9: Focus
  - 10: Pan (X axis)
  - 11: Pan Fine
  - 12: Tilt (Y axis)
  - 13: Tilt Fine
  - 14: Pan/Tilt Speed
  - 15: Frost & Rainbow Lens
  - 16: Lamp Control & Reset
  # In 16 Channel Mode - The following channels are not used
  - 17: Empty
  - 18: Color Speed
  - 19: Dimmer-Prism-Frost Speed
  - 20: Gobo Speed

 * */

#ifdef BEAM_MODE_16_CHANNELS
#undef BEAM_MODE_20_CHANNELS
#endif // BEAM_MODE_16_CHANNELS

#ifdef BEAM_MODE_20_CHANNELS
#undef BEAM_MODE_16_CHANNELS
#endif // BEAM_MODE_20_CHANNELS

#ifdef BEAM_MODE_16_CHANNELS
#define BEAM_MAX_CHANNELS        16
#else // BEAM_MODE_16_CHANNELS
#define BEAM_MODE_20_CHANNELS
#define BEAM_MAX_CHANNELS        20
#endif // BEAM_MODE_16_CHANNELS

#define RXEN_PIN                 2
#define DMX_TX_PIN               10

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


// Manual controls constants
// Joystick pins
#define CONTROL_JOYSTICK_LEFT_PIN 4
#define CONTROL_JOYSTICK_RIGHT_PIN 7
#define CONTROL_JOYSTICK_UP_PIN 8
#define CONTROL_JOYSTICK_DOWN_PIN 12
// Selector pins
#define CONTROL_BEAM_SELECTOR_PIN 3
#define CONTROL_MOTOR_SELECTOR_PIN 11
#define CONTROL_BRIGHTNESS_SELECTOR_PIN 13


// Beam constants
#define COLOR_WHITE             0
#define STROBE_OPEN             255
#define DIMMER_MAX_BRIGHTNESS   255
#define DIMMER_MIN_BRIGHTNESS   50
#define GOBO_DISABLED           0
#define GOBO_BOBBING_MOTION     80
#define GOBO_SUPER_FOCUSED      60
#define FOCUS_MAX               255
#define FOCUS_MIN               0
#define PAN_MIN_ANGLE           0
#define PAN_MAX_ANGLE           540
#define TILT_MIN_ANGLE          0
#define TILT_ANGLE_OFFSET       28
#define TILT_MAX_ANGLE          270
#define PAN_TILT_SPEED_DEFAULT  0
#define FROST_DISABLED          0
#define FROST_BLUR              130
#define LAMP_OFF                100
#define LAMP_ON                 200
#define LAMP_RESET              255


const boolean BEAM_FROST_ENABLED = true;                // enables an aesthetic fade effect in the edges of the dmx_buffer
const boolean BEAM_GOBO_ENABLED = true;                 // enables a light bobbing motion of the dmx_buffer which can be pretty

// Motor constants
// R_EN and L_EN connected to VCC (IBT-2 pins 3 & 4)
// L_IS and R_IS not connected    (IBT-2 pins 5 & 6)
const int MOTOR_RPWM_OUTPUT = 5;                        // Arduino PWM output pin D9; connect to IBT-2 pin 1 (RPWM)
const int MOTOR_LPWM_OUTPUT = 6;                        // Arduino PWM output pin D10; connect to IBT-2 pin 2 (LPWM)
const int MOTOR_MAX_PWM_VALUE = 255;                    // Adjust this delay to control the maximum speed of the motor
const int MOTOR_ACCELERATION_DELAY_MILLIS = 30;         // Adjust this delay to control the speed of acceleration/deceleration

// motor state
typedef struct MotorState {
  short current_pwm = 0;    // tracking the current PWM value to ensure smooth transitions in case of duplicate commands
  short target_pwm = 0;     // allowing small incremets with short delays in a non blocking manner until the desired state is reached
} MotorState;

typedef struct CommandState {
  int16_t pan = 0;
  int16_t tilt = 0;
  int16_t brightness = 0;
  int16_t velocity = 0;
  int16_t motor = 0;

  void update_from_json(const ArduinoJson::JsonDocument &command);

} CommandState;

typedef struct BeamState {
  byte color;
  byte strobe;
  byte dimmer;
  byte gobo;
  byte prism_1;
  byte prism_2;
  byte prism_3;
  byte prism_4;
  byte focus;
  byte pan;
  byte pan_fine;
  byte tilt;
  byte tilt_fine;
  byte pan_tilt_time;
  byte frost;
  byte lamp_ctrl_reset;
#ifdef BEAM_MODE_20_CHANNELS
  byte _empty;
  byte color_time;
  byte dimmer_prism_frost_time;
  byte gobo_time;
#endif  // BEAM_MODE_20_CHANNELS
} BeamState;


typedef union DmxBuffer {
  // Addressable properties by name
  struct {
    BeamState beam;
  };
  byte buffer[BEAM_MAX_CHANNELS];
} DmxBuffer;
static_assert(sizeof(BeamState) == BEAM_MAX_CHANNELS, "BeamState size mismatch");
static_assert(sizeof(DmxBuffer) == BEAM_MAX_CHANNELS, "DmxBuffer size mismatch");


static MotorState motor_state;
static CommandState command_state;
static DmxBuffer dmx_buffer;

/*
 * Translate the command beam to output units
 * */
void apply_command();

void dump_command_state();


namespace sauronism {
  namespace json_protocol {
    static const char *valid_commands[] = {
        "pan", "x",
        "tilt", "y",
        "brightness", "b",
        "velocity", "v",
        "motor", "m",
        "debug"
    };

  }
  namespace dmx_protocol {
    void dmx_update() {
      for (int i = 0; i < BEAM_MAX_CHANNELS; i++) {
        DmxSimple.write(i + 1, dmx_buffer.buffer[i]);
      }
    }

  namespace dmx_protocol {
    void dmx_update() {
      for (int i = 0; i < BEAM_MAX_CHANNELS; i++) {
        DmxSimple.write(i + 1, dmx_buffer.buffer[i]);
      }
    }

    void dmx_init() {
      DmxSimple.usePin(DMX_TX_PIN);
      pinMode(RXEN_PIN, OUTPUT);
      digitalWrite(RXEN_PIN, HIGH);
      DmxSimple.maxChannel(BEAM_MAX_CHANNELS);
    }
  }

  namespace motor {
    static void motor_init() {
      pinMode(MOTOR_RPWM_OUTPUT, OUTPUT);
      pinMode(MOTOR_LPWM_OUTPUT, OUTPUT);
      analogWrite(MOTOR_LPWM_OUTPUT, 0);
      motor_state.current_pwm = 0;
      motor_state.target_pwm = 0;
    }

    static void motor_update() {
      short delta = (motor_state.current_pwm > motor_state.target_pwm) ? -1 : 1;
      motor_state.current_pwm += delta;
      analogWrite(MOTOR_RPWM_OUTPUT, motor_state.current_pwm);
    }
  }

  namespace control_panel {

    static void control_panel_init() {
      pinMode(CONTROL_JOYSTICK_LEFT_PIN, INPUT_PULLUP);
      pinMode(CONTROL_JOYSTICK_RIGHT_PIN, INPUT_PULLUP);
      pinMode(CONTROL_JOYSTICK_UP_PIN, INPUT_PULLUP);
      pinMode(CONTROL_JOYSTICK_DOWN_PIN, INPUT_PULLUP);
      pinMode(CONTROL_BEAM_SELECTOR_PIN, INPUT_PULLUP);
      pinMode(CONTROL_MOTOR_SELECTOR_PIN, INPUT_PULLUP);
      pinMode(CONTROL_BRIGHTNESS_SELECTOR_PIN, INPUT_PULLUP);
    }

    void control_panel_log(
        bool select_beam,
        bool select_motor,
        bool select_brightness,
        bool control_up,
        bool control_down,
        bool control_left,
        bool control_right
    );


    static void control_panel_update_beam(
        const bool control_up,
        const bool control_down,
        const bool control_left,
        const bool control_right
    ) {
      if (control_left) {
        command_state.pan = constrain(command_state.pan - 1, 0, 180);
      } else if (control_right) {
        command_state.pan = constrain(command_state.pan + 1, 0, 180);
      } else if (control_up) {
        command_state.tilt = constrain(command_state.tilt + 1, -30, 30);
      } else if (control_down) {
        command_state.tilt = constrain(command_state.tilt - 1, -30, 30);
      }
    }

    static void control_panel_update_motor(const bool control_up, const bool control_down) {
      if (control_up) {
        command_state.motor = constrain(command_state.motor + 1, 0, 254);
      } else if (control_down) {
        command_state.motor = constrain(command_state.motor - 1, 0, 254);
      }
    }

    static void control_panel_update_brightness(const bool control_up, const bool control_down) {
      if (control_up) {
        command_state.brightness = constrain(command_state.brightness + 1, 0, 255);
      } else if (control_down) {
        command_state.brightness = constrain(command_state.brightness - 1, 0, 255);
      }
    }

    static void control_panel_update() {
      const auto select_beam = digitalRead(CONTROL_BEAM_SELECTOR_PIN) == LOW;
      const auto select_motor = digitalRead(CONTROL_MOTOR_SELECTOR_PIN) == LOW;
      const auto select_brightness = digitalRead(CONTROL_BRIGHTNESS_SELECTOR_PIN) == LOW;
      const auto control_up = digitalRead(CONTROL_JOYSTICK_UP_PIN) == LOW;
      const auto control_down = digitalRead(CONTROL_JOYSTICK_DOWN_PIN) == LOW;
      const auto control_left = digitalRead(CONTROL_JOYSTICK_LEFT_PIN) == LOW;
      const auto control_right = digitalRead(CONTROL_JOYSTICK_RIGHT_PIN) == LOW;
      const auto is_any_command_active = select_beam || select_motor || select_brightness;

      if (select_beam) {
        control_panel_update_beam(control_up, control_down, control_left, control_right);
      } else if (select_motor) {
        control_panel_update_motor(control_up, control_down);
      } else if (select_brightness) {
        control_panel_update_brightness(control_up, control_down);
      }
      if (is_any_command_active) {
        control_panel_log(
            select_beam,
            select_motor,
            select_brightness,
            control_up,
            control_down,
            control_left,
            control_right
        );
        apply_command();
      }
    }
  }
}

void CommandState::update_from_json(const ArduinoJson::JsonDocument &command) {
  if (command.containsKey("pan")) {
    pan = command["pan"];
  }
  if (command.containsKey("x")) {
    pan = command["x"];
  }
  if (command.containsKey("tilt")) {
    tilt = command["tilt"];
  }
  if (command.containsKey("y")) {
    tilt = command["y"];
  }
  if (command.containsKey("brightness")) {
    brightness = command["brightness"];
  }
  if (command.containsKey("b")) {
    brightness = command["b"];
  }
  if (command.containsKey("velocity")) {
    velocity = command["velocity"];
  }
  if (command.containsKey("v")) {
    velocity = command["v"];
  }
  if (command.containsKey("motor")) {
    motor = command["motor"];
  }
  if (command.containsKey("m")) {
    motor = command["m"];
  }
}


void init_dmx_buffer() {
  for (auto &_byte: dmx_buffer.buffer) {
    _byte = 0;
  }

  dmx_buffer.beam.color = COLOR_WHITE;
  dmx_buffer.beam.strobe = STROBE_OPEN;
  dmx_buffer.beam.dimmer = DIMMER_MIN_BRIGHTNESS + 20;
  dmx_buffer.beam.gobo = GOBO_DISABLED;
  dmx_buffer.beam.prism_1 = 0;
  dmx_buffer.beam.prism_2 = 0;
  dmx_buffer.beam.prism_3 = 0;
  dmx_buffer.beam.prism_4 = 0;
  dmx_buffer.beam.focus = FOCUS_MIN;
  dmx_buffer.beam.pan = 0;
  dmx_buffer.beam.pan_fine = 0;
  dmx_buffer.beam.tilt = 0;
  dmx_buffer.beam.tilt_fine = 0;
  dmx_buffer.beam.pan_tilt_time = PAN_TILT_SPEED_DEFAULT;
  dmx_buffer.beam.frost = FROST_DISABLED;
  dmx_buffer.beam.lamp_ctrl_reset = LAMP_ON;
#ifdef BEAM_MODE_20_CHANNELS
  dmx_buffer.beam._empty = 0;
  dmx_buffer.beam.color_time = 0;
  dmx_buffer.beam.dimmer_prism_frost_time = 0;
  dmx_buffer.beam.gobo_time = 0;
#endif  // BEAM_MODE_20_CHANNELS

}

void init_command_state() {
  command_state.pan = 0;
  command_state.tilt = 0;
  command_state.brightness = 0;
  command_state.velocity = 0;
  command_state.motor = 0;
}


void dump_debug_state() {
  // Beam
  Serial.print(F(R"({"module": "dump_debug_state", "color": )"));
  Serial.print(dmx_buffer.beam.color);
  Serial.print(F(R"(, "strobe": )"));
  Serial.print(dmx_buffer.beam.strobe);
  Serial.print(F(R"(, "dimmer": )"));
  Serial.print(dmx_buffer.beam.dimmer);
  Serial.print(F(R"(, "gobo": )"));
  Serial.print(dmx_buffer.beam.gobo);
  Serial.print(F(R"(, "focus": )"));
  Serial.print(dmx_buffer.beam.focus);
  Serial.print(F(R"(, "pan": )"));
  Serial.print(dmx_buffer.beam.pan);
  Serial.print(F(R"(, "tilt": )"));
  Serial.print(dmx_buffer.beam.tilt);
  Serial.print(F(R"(, "pan_tilt_time": )"));
  Serial.print(dmx_buffer.beam.pan_tilt_time);
  Serial.print(F(R"(, "frost": )"));
  Serial.print(dmx_buffer.beam.frost);
  Serial.print(F(R"(, "lamp_ctrl_reset": )"));
  Serial.print(dmx_buffer.beam.lamp_ctrl_reset);
#ifdef BEAM_MODE_20_CHANNELS
//  Serial.print(F(R"(, "_empty": )"));
//  Serial.print(dmx_buffer.beam._empty);
  Serial.print(F(R"(, "color_time": )"));
  Serial.print(dmx_buffer.beam.color_time);
  Serial.print(F(R"(, "dimmer_prism_frost_time": )"));
  Serial.print(dmx_buffer.beam.dimmer_prism_frost_time);
  Serial.print(F(R"(, "gobo_time": )"));
  Serial.print(dmx_buffer.beam.gobo_time);
#endif  // BEAM_MODE_20_CHANNELS


  // Motor State
  Serial.print(F(R"(, "motor_current_pwm": )"));
  Serial.print(motor_state.current_pwm);
  Serial.print(F(R"(, "motor_target_pwm": )"));
  Serial.print(motor_state.target_pwm);

  Serial.print(F("}\n"));
}

void apply_command() {
  dmx_buffer.beam.pan = map(command_state.pan, PAN_MIN_ANGLE, PAN_MAX_ANGLE, 0, 255);
  dmx_buffer.beam.tilt = constrain(
      map(command_state.tilt + TILT_ANGLE_OFFSET, TILT_MIN_ANGLE, TILT_MAX_ANGLE - TILT_ANGLE_OFFSET, 0, 255), 0,
      255);
  dmx_buffer.beam.dimmer = command_state.brightness;
  dmx_buffer.beam.pan_tilt_time = map(command_state.velocity, 0, 100, 0, 255);
  motor_state.target_pwm = command_state.motor;
}

void dump_command_state() {
  Serial.print(F("{"));
  Serial.print(F(R"("pan": )"));
  Serial.print(command_state.pan);
  Serial.print(F(R"(, "tilt": )"));
  Serial.print(command_state.tilt);
  Serial.print(F(R"(, "brightness": )"));
  Serial.print(command_state.brightness);
  Serial.print(F(R"(, "velocity": )"));
  Serial.print(command_state.velocity);
  Serial.print(F(R"(, "motor": )"));
  Serial.print(command_state.motor);
  Serial.print("}");
}

void sauronism::control_panel::control_panel_log(
    const bool select_beam,
    const bool select_motor,
    const bool select_brightness,
    const bool control_up,
    const bool control_down,
    const bool control_left,
    const bool control_right
) {
  Serial.print(F(R"({"module": "control_panel", "select_beam": )"));
  Serial.print(select_beam);
  Serial.print(F(R"(, "select_motor": )"));
  Serial.print(select_motor);
  Serial.print(F(R"(, "select_brightness": )"));
  Serial.print(select_brightness);
  Serial.print(F(R"(, "control_up": )"));
  Serial.print(control_up);
  Serial.print(F(R"(, "control_down": )"));
  Serial.print(control_down);
  Serial.print(F(R"(, "control_left": )"));
  Serial.print(control_left);
  Serial.print(F(R"(, "control_right": )"));
  Serial.print(control_right);
  Serial.print(R"(, "command_state": )");
  dump_command_state();
  Serial.print("}\n");
}


void command_parse_log(const ArduinoJson::DeserializationError &error) {
  Serial.print(F(R"({"module": "command_parse", "ok": )"));
  Serial.print(!error ? F("true") : F("false"));
  Serial.print(F(R"(, "details": ")"));
  Serial.print(error.f_str());
  Serial.print(F(R"(", "command_state": )"));
  dump_command_state();
  Serial.print("}\n");
}


auto command_parse() -> ArduinoJson::DeserializationError {
  static const ArduinoJson::JsonObjectConst filter = ([&]() {
    static ArduinoJson::JsonDocument _command_filter;
    for (const auto field: sauronism::json_protocol::valid_commands) {
      _command_filter[field] = true;
    }
    return _command_filter.as<ArduinoJson::JsonObject>();
  })();

  static ArduinoJson::JsonDocument command_json_buffer;
  const auto error = ArduinoJson::deserializeJson(
      command_json_buffer,
      Serial,
      ArduinoJson::DeserializationOption::Filter(filter),
      ArduinoJson::DeserializationOption::NestingLimit(1)
  );

  switch (error.code()) {
    case ArduinoJson::DeserializationError::Ok:
      command_state.update_from_json(command_json_buffer);
      if (command_json_buffer.containsKey("debug")) {
        dump_debug_state();
      }
      apply_command();
      break;
    default:
      break;
  }
  return
      error;
}

void setup() {
  delay(500);
  wdt_enable(WDTO_2S);
  Serial.begin(256000);
  Serial.println("Starting Beam Controller");
  init_command_state();
  init_dmx_buffer();
  Serial.println("Initializing Motor Controller");
  sauronism::motor::motor_init();
  Serial.println("Initializing Control Panel");
  sauronism::control_panel::control_panel_init();
  Serial.println("Initializing DMX Protocol");
  sauronism::dmx_protocol::dmx_init();
  apply_command();
  sauronism::dmx_protocol::dmx_update();
  Serial.println("Beam Controller Ready");
}


/*
 * This function is called whenever the serial port receives data
 * */
void serialEvent() {
  if (Serial.available()) {
    const auto error = command_parse();
    command_parse_log(error);
  }
}


void loop() {
  wdt_reset();
  EVERY_N_MILLISECONDS(10) {
    sauronism::dmx_protocol::dmx_update();
  }
  EVERY_N_MILLISECONDS(100) {
    sauronism::motor::motor_update();
  }
  EVERY_N_MILLISECONDS(100) {
    sauronism::control_panel::control_panel_update();
  }

  EVERY_N_SECONDS(10) {
    dump_debug_state();
  }
}
