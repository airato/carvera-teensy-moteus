#ifndef __IMXRT1062__
#error "This sketch should be compiled for Teensy 4.x"
#endif

#include <ACAN_T4.h>
// install "Moteus" library via Arduino IDE library manager, then replace Moteus folder in
// Arduino/libraries with https://github.com/kylevernyi/moteus-teensy
#include "Moteus.h"
#include "moteus_multiplex.h"

Moteus moteus1;
Moteus::PositionMode::Command position_cmd;
Moteus::PositionMode::Format position_fmt;

void printLastMoteusState()
{
  const auto &v = moteus1.last_result().values;
  Serial.print(F(" mode="));
  Serial.print(static_cast<int>(v.mode));
  Serial.print(F(" pos="));
  Serial.print(v.position);
  Serial.print(F(" vel="));
  Serial.print(v.velocity);
  Serial.print(F(" torque="));
  Serial.print(v.torque);
  Serial.println();
}

// PWM input pin. Carvera sends 5V, this needs to be level shifted to 3.3V
const int pwmInputPin = 6;
// Feedback output pin. Carvera expects 3.3V, no level shift needed
const int feedbackOutputPin = 7;

// PWM signal handling variables
volatile uint32_t riseTime = 0;
volatile uint32_t fallTime = 0;
volatile uint32_t lastRiseTime = 0;
volatile uint32_t pulseWidth = 0;
volatile uint32_t period = 0;
volatile float powerLevelRaw = 0;
volatile float oldPowerLevelRaw = 0;
volatile bool lastPinState = false;        // Track last pin state
volatile uint32_t lastStateChangeTime = 0; // Track last state change time

float targetMotorRPM = 0;
float targetSpindleRPM = 0;
float currentRPM = 0;
// this is set based on 15000rpm / 1.635 (gear ratio)
const float MAX_MOTOR_RPM = 9174.0f;

const unsigned long pwmTimeout = 1000000;
unsigned long lastValidPWMTime = 0;
unsigned long lastPinStateTime = 0;

float feedbackPulsesPerSecond = 0;
float oldFeedbackPulsesPerSecond = 0;
float pulseIntervalMicros = 0;
unsigned long lastPulseMicros = 0;
bool feedbackPinState = false;           // Track feedback pin state
unsigned long feedbackPinChangeTime = 0; // Track when feedback pin last changed state
float targetVelocity = 0;
float lastVelocity = 0;

void pwmISR()
{
  uint32_t now = micros();
  bool state = digitalRead(pwmInputPin);
  lastPinStateTime = now;
  if (state != lastPinState)
  { // Only process if state actually changed
    lastStateChangeTime = now;

    if (state)
    { // Rising edge
      period = now - lastRiseTime;
      lastRiseTime = now;
      riseTime = now;
    }
    else
    { // Falling edge
      fallTime = now;
      pulseWidth = fallTime - riseTime;
      powerLevelRaw = 100.0 * min(pulseWidth, 1000) / 1000.0;
      lastValidPWMTime = now;
    }

    lastPinState = state;
  }
}

// Constants
const float MIN_FREQUENCY = 10.0; // Minimum PWM frequency in Hz
const float PPR = 12.0;           // Pulses Per Revolution
// Function to update the pulse frequency based on RPM
void updatePulseFrequency()
{
  // Calculate frequency in Hz
  // For 12 pulses per revolution, at currentRpm
  float frequency = (targetMotorRPM * PPR) / 60.0;

  // If frequency is too low or zero, turn off the output
  if (frequency < MIN_FREQUENCY || targetMotorRPM <= 0)
  {
    analogWrite(feedbackOutputPin, 0); // 0% duty cycle (off)
  }
  else
  {
    // Set the PWM frequency and duty cycle
    analogWriteFrequency(feedbackOutputPin, frequency);
    analogWrite(feedbackOutputPin, 128); // 50% duty cycle (0-255)
  }
}

void setup()
{
  pinMode(pwmInputPin, INPUT);
  pinMode(feedbackOutputPin, OUTPUT);
  analogWrite(feedbackOutputPin, 0);

  // Initial update of pulse frequency
  updatePulseFrequency();

  // Check initial state of PWM input
  lastPinState = digitalRead(pwmInputPin);
  if (lastPinState)
  {
    // If pin is already HIGH, set power to 100%
    powerLevelRaw = 100.0f;
    lastValidPWMTime = micros();
  }

  attachInterrupt(digitalPinToInterrupt(pwmInputPin), pwmISR, CHANGE);

  /* Let the world know we have begun! */
  Serial.begin(115200);
  while (!Serial)
  {
  }
  Serial.println(F("started"));

  /* Configure and start CAN bus (needs to be CAN3 to work with CAN-FD) */
  ACAN_T4FD_Settings settings(1000000, DataBitRateFactor::x1);

  const uint32_t errorCode = ACAN_T4::can3.beginFD(settings);
  if (0 == errorCode)
  {
    Serial.println("can3 ok");
  }
  else
  {
    Serial.print("Error can3: 0x");
    Serial.println(errorCode, HEX);
  }

  /* Configure Moteus controller */
  moteus1.options_.id = 1;
  moteus1.options_.default_query = true;

  /* Configure fields that we want to receive from Moteus controller */
  moteus1.options_.query_format.velocity = Moteus::kFloat;
  // moteus1.options_.query_format.mode = mjbots::moteus::kIgnore;
  moteus1.options_.query_format.torque = mjbots::moteus::kIgnore;
  moteus1.options_.query_format.voltage = mjbots::moteus::kIgnore;
  moteus1.options_.query_format.temperature = mjbots::moteus::kIgnore;
  moteus1.options_.query_format.fault = mjbots::moteus::kIgnore;

  /* Configure fields that we want to send to Moteus controller */
  position_fmt.position = Moteus::kFloat;
  position_fmt.velocity = Moteus::kFloat;
  position_fmt.maximum_torque = Moteus::kFloat;
  // position_fmt.kp_scale = Moteus::kFloat;
  // position_fmt.kd_scale = Moteus::kFloat;
  // position_fmt.velocity_limit = Moteus::kFloat;
  // position_fmt.accel_limit = Moteus::kFloat;
  position_fmt.watchdog_timeout = Moteus::kFloat;
  // position_fmt.mode = Moteus::kIgnore;

  /* Custom limits to override default values */
  // position_cmd.velocity_limit = 4.0;
  // position_cmd.accel_limit = 0.0;

  /* First we'll clear faults and stop the motor */
  moteus1.SetStop(&moteus1.options_.query_format);
  Serial.println(F("stop motor"));
  printLastMoteusState();
}

static unsigned long lastPrint = 0;
static float lastRpm = 0;

void loop()
{
  /* Take care of signal loss and 100% pwm signal (no falling edge) */
  uint32_t currentMicros = micros();
  if (currentMicros - lastPinStateTime > 3000 && currentMicros > lastPinStateTime)
  {
    if (lastPinState)
    {
      if (powerLevelRaw != 100.0)
      {
        Serial.println("timeout set 100");
      }
      powerLevelRaw = 100.0;
    }
    else
    {
      if (powerLevelRaw != 0.0)
      {
        Serial.print("timeout set 0");
      }
      powerLevelRaw = 0.0;
    }
  }

  /* Update target RPM based on power level */
  targetMotorRPM = powerLevelRaw * MAX_MOTOR_RPM / 100.0;
  targetSpindleRPM = targetMotorRPM * 1.635;

  /* Update pulse frequency if RPM changed by more than 1 */
  if (abs(targetMotorRPM - lastRpm) > 1.0)
  {
    updatePulseFrequency();
    lastRpm = targetMotorRPM;
  }

  /* Send velocity command to Moteus controller */
  targetVelocity = targetMotorRPM / 60;
  if (targetVelocity == 0)
  {
    moteus1.SetStop(&moteus1.options_.query_format);
  }
  else if (targetVelocity != lastVelocity) // TODO: send this at least once before watchdog timer expires
  {
    lastVelocity = targetVelocity;
    
    // spin at given velocity
    position_cmd.position = NaN;
    position_cmd.velocity = targetVelocity;

    // scale P and D values that configured in the moteus controller
    // position_cmd.kp_scale = 0.5;
    // position_cmd.kd_scale = 0.5;

    // this is torque probably too high, but that should be ok. It will be limited by power supply
    // and servo.max_current_A value
    // TODO: check how high we can set servo.max_current_A value
    position_cmd.maximum_torque = 0.8;

    // TODO: might be better to have this lower
    position_cmd.watchdog_timeout = 100.0;
    moteus1.SetPosition(position_cmd, &position_fmt, &moteus1.options_.query_format);
    // Serial.print("sent velocity: ");
    // Serial.println(targetVelocity);
  }

  // TODO: check for Moteus faults and other errors, implement stall detection, and send fault signal to carvera (3.3v might work)

  /* Print debug information */
  if ((
          (powerLevelRaw != oldPowerLevelRaw || feedbackPulsesPerSecond != oldFeedbackPulsesPerSecond) &&
          millis() - lastPrint > 100) ||
      millis() - lastPrint > 1000)
  {
    const auto &moteusState = moteus1.last_result().values;
    // Serial.print("pulseWidth: ");
    // Serial.print(pulseWidth, 1);
    // Serial.print(", powerLevelRaw %: ");
    // Serial.print(powerLevelRaw, 1);
    Serial.print(", Target motor RPM: ");
    Serial.print(targetMotorRPM, 1);
    Serial.print(", Current motor RPM: ");
    Serial.print(moteusState.velocity * 60);
    Serial.print(", Target spindle RPM: ");
    Serial.print(targetSpindleRPM, 1);
    Serial.print(", Current spindle RPM: ");
    Serial.print(moteusState.velocity * 60 * 1.635, 1);
    Serial.print(", vel: ");
    Serial.print(moteusState.velocity);
    Serial.print(", Target vel: ");
    Serial.print(targetVelocity);
    Serial.print(", PPS: ");
    Serial.print(feedbackPulsesPerSecond, 1);
    Serial.print(", Moteus mode: ");
    Serial.print(static_cast<int>(moteusState.mode));
    Serial.println();
    lastPrint = millis();
  }
  oldPowerLevelRaw = powerLevelRaw;
  oldFeedbackPulsesPerSecond = feedbackPulsesPerSecond;
}
