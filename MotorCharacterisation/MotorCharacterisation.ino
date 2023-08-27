#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;

// Change next line to this if you are using the older 3pi+
// with a black and green LCD display:
OLED display;

// Configuration for step response test:
const int16_t step_resp_speed = 40;
const int16_t test_duration_ms = 1000;
const int16_t test_sample_period_ms = 10;

// Variables for performing the step test:
const int16_t log_length = test_duration_ms / test_sample_period_ms;
int16_t step_response_log[log_length] = {};

// Function for performing step test on left motor.
void performStepTestLeft(int16_t test_motor_speed ) {
  // Record start time
  unsigned long time_to_perform_next_sample_ms = millis();
  // Initialise iterator for log
  int i_log = 0;

  // Switch on the left motor
  motors.setSpeeds(test_motor_speed, 0);

  // Start recording left encoder values
  while (i_log < log_length) {
    // Wait for the correct time to take a sample
    while (millis() < time_to_perform_next_sample_ms) { }
    // Record encoder value
    step_response_log[i_log] = encoders.getCountsLeft();
    // Increment iterator
    i_log++;
    // Update this variable to schedule when we'll take the next recording
    time_to_perform_next_sample_ms += test_sample_period_ms;
  }

  // Switch off the motor
  motors.setSpeeds(0, 0);

  // Print header
  Serial.print("\n\n");
  for (int h = 0; h < 40; h++) {Serial.print("-");} Serial.print("\n");
  Serial.print("LEFT STEP TEST - MOTOR COMMAND is [");
  Serial.print(test_motor_speed); Serial.print("]\n");
  for (int h = 0; h < 40; h++) {Serial.print("-");} Serial.print("\n");
  printLogToSerialPort();
}

// Function for performing step test on right motor. Exactly the same as left except uses right function calls.
void performStepTestRight(int16_t test_motor_speed ) {
  // Record start time
  unsigned long time_to_perform_next_sample_ms = millis();
  // Initialise iterator for log
  int i_log = 0;

  // Switch on the left motor
  motors.setSpeeds(0, test_motor_speed);

  // Start recording left encoder values
  while (i_log < log_length) {
    // Wait for the correct time to take a sample
    while (millis() < time_to_perform_next_sample_ms) { }
    // Record encoder value
    step_response_log[i_log] = encoders.getCountsRight();
    // Increment iterator
    i_log++;
    // Update this variable to schedule when we'll take the next recording
    time_to_perform_next_sample_ms += test_sample_period_ms;
  }

  // Switch off the motor
  motors.setSpeeds(0, 0);

  // Print header
  Serial.print("\n\n");
  for (int h = 0; h < 40; h++) {Serial.print("-");} Serial.print("\n");
  Serial.print("RIGHT STEP TEST - MOTOR COMMAND is [");
  Serial.print(test_motor_speed); Serial.print("]\n");
  for (int h = 0; h < 40; h++) {Serial.print("-");} Serial.print("\n");
  printLogToSerialPort();
}

int16_t step_delta;
float rev_per_sec;
float mm_per_sec;
void printLogToSerialPort() {
  // Print data headers:
  Serial.print("t(ms)"); Serial.print("\t");
  Serial.print("steps"); Serial.print("\t");
  Serial.print("sDelta"); Serial.print("\t");
  Serial.print("rot/sec"); Serial.print("\t");
  Serial.print("mm/sec"); Serial.print("\n");

  // Iterate through log, printing each line
  for (int i = 0; i < log_length; i++) {
    // Generate and print timestamp
    Serial.print(i * test_sample_period_ms);
    Serial.print("\t");

    // Print raw data
    Serial.print(step_response_log[i]);
    Serial.print("\t");

    // Calculate and print step delta and rotation/sec data
    // Take count delta, divide by 358.3 (counts/rev) to get revolutions, then divide by time period for rev/sec
    if (i == 0) {step_delta = 0;} else {step_delta = step_response_log[i] - step_response_log[i-1];}
    rev_per_sec = (float)step_delta / (358.3 * ((float)test_sample_period_ms / 1000) );
    Serial.print(step_delta);
    Serial.print("\t");
    Serial.print(rev_per_sec);
    Serial.print("\t");

    // Calculate and print mm/sec data
    // Multiply by 100.5 (mm/rotation - i.e.: circumference of our wheel) to get mm/sec
    mm_per_sec = rev_per_sec * 100.5;
    Serial.print(mm_per_sec);
    Serial.print("\n");
  }
}

void setup()
{
  // Setup serial port for printing results at 9600 baud
  Serial.begin(9600);

  // Print welcome message
  display.setLayout11x4();
  display.gotoXY(0, 0);
  display.print(F("Motor test"));
  display.gotoXY(0, 1);
  display.print(F("   Open"));
  display.gotoXY(0, 2);
  display.print(F("Serial port"));
  display.gotoXY(0, 3);
  display.print(F("B -> Start"));

  // Wait for B button press to start the tests
  while(!buttonB.getSingleDebouncedPress()) {}
  delay(500);

  // Perform left step response forward direction & print results
  display.clear(); display.gotoXY(1, 0); display.print("Left step"); display.gotoXY(1, 1); display.print("Forward");
  performStepTestLeft(step_resp_speed);
  delay(250);

  // Perform left step response reverse direction & print results
  display.clear(); display.gotoXY(1, 0); display.print("Left step"); display.gotoXY(1, 1); display.print("Reverse");
  performStepTestLeft(-step_resp_speed);
  delay(250);

  // Perform right step response forward direction & print results
  display.clear(); display.gotoXY(1, 0); display.print("Right step"); display.gotoXY(1, 1); display.print("Forward");
  performStepTestRight(step_resp_speed);
  delay(250);

  // Perform right step response reverse direction & print results
  display.clear(); display.gotoXY(1, 0); display.print("Right step"); display.gotoXY(1, 1); display.print("Reverse");
  performStepTestRight(-step_resp_speed);
  delay(250);

  // Perform linearity test

}

void loop()
{
  // Do nothing!
  display.clear();
  display.gotoXY(1, 0);
  display.print("Complete!");
  delay(10000);
}
