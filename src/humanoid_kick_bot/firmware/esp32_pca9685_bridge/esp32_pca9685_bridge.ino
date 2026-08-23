/*
 * esp32_pca9685_bridge.ino
 * -------------------------
 * Firmware companion to servo_controller_node.py.
 *
 * Wiring:
 *   ESP32 GPIO21 -> PCA9685 SDA
 *   ESP32 GPIO22 -> PCA9685 SCL
 *   PCA9685 runs at 50 Hz (standard analog servo rate).
 *
 * Protocol (from ROS2 over USB serial, 115200 baud):
 *   ASCII line: "<channel>,<pwm_ticks>\n"
 *   e.g. "10,340\n"  -> channel 10 (left_thigh) set to 340 PWM ticks.
 *   pwm_ticks is already computed on the ROS side from the 0-180 deg ->
 *   120-520 tick calibration, so this firmware just forwards it.
 *
 * On boot, all 9 channels (7-15) are driven to the 90 deg neutral pulse
 * so the robot always powers up in a known, safe pose.
 *
 * Library: Adafruit PWM Servo Driver Library (install via Library Manager).
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SDA_PIN 21
#define SCL_PIN 22
#define PWM_FREQ_HZ 50

// Must match config/servo_config.yaml exactly.
#define PWM_MIN 120
#define PWM_MAX 520
#define NEUTRAL_TICKS ((PWM_MIN + PWM_MAX) / 2)  // ~320 ticks == 90 deg

const uint8_t ACTIVE_CHANNELS[] = {7, 8, 9, 10, 11, 12, 13, 14, 15};
const uint8_t NUM_CHANNELS = sizeof(ACTIVE_CHANNELS) / sizeof(ACTIVE_CHANNELS[0]);

String rxBuffer = "";

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(10);

  // Safe boot: all servos to neutral (90 deg == PWM_MIN..PWM_MAX midpoint)
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    pwm.setPWM(ACTIVE_CHANNELS[i], 0, NEUTRAL_TICKS);
  }

  Serial.println("esp32_pca9685_bridge ready");
}

void handleLine(const String &line) {
  int commaIdx = line.indexOf(',');
  if (commaIdx < 0) return;

  int channel = line.substring(0, commaIdx).toInt();
  int ticks = line.substring(commaIdx + 1).toInt();

  if (channel < 0 || channel > 15) return;                 // PCA9685 has 16 channels
  if (ticks < PWM_MIN - 20 || ticks > PWM_MAX + 20) return; // sanity guard vs. corrupt bytes

  ticks = constrain(ticks, PWM_MIN, PWM_MAX);
  pwm.setPWM(channel, 0, ticks);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleLine(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
      if (rxBuffer.length() > 32) rxBuffer = "";  // guard against garbage overflow
    }
  }
}
