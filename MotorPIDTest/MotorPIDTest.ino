/*
Requires PID library, RunningAverage Library and
*/

#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Motors motors;
ButtonB buttonB;
OLED display;

// **************************************************************************
// ********** Tester configurable values below ******************************
// **************************************************************************

// PID controller settings
#define MOTOR_CTRL_LOOP_PERIOD_SEC 0.02 // Set the loop time period, in seconds
#define P_GAIN 0.1
#define I_GAIN 0.1
#define D_GAIN 0.0
#define FEEDFWD_GAIN (400.0 / 1500.0) // Set to (400.0 / 1500.0) which approximates ideal value by assuming full motor command of 400 corresponds to full linear velocity output of 1500 mm/s
#define VEL_FILTER_ROLLING_AVG_BUFFER_SIZE 5
#define PID_LOWER_OUTPUT_LIMIT -150.0 // Clamp PID controller's output to these values:
#define PID_UPPER_OUTPUT_LIMIT 150.0

// Step response test:
const double low_velocity_mm_per_sec = 0;
const double high_velocity_mm_per_sec = 400;
const int16_t test_low_duration_ms = 50;
const int16_t test_high_duration_ms = 1000;
const int16_t test_sample_period_ms = 10;

// **************************************************************************
// ********** End tester configurable values ********************************
// **************************************************************************

// Variables for performing the step test:
const int16_t step_log_length = (test_low_duration_ms + test_high_duration_ms) / test_sample_period_ms;
int16_t step_response_log[step_log_length] = {};
double PID_logged_output[step_log_length] = {};

void performLeftStepInputTest(double low_test_velocity, double high_test_velocity) {
  // Record start time
  unsigned long time_to_perform_next_sample_ms = millis();
  unsigned long high_time_ms = millis() + test_low_duration_ms; // Calculate what time we need to change output value
  // Initialise iterator for log
  int i_log = 0;

  // Set left motor to start speed
  setWheelVelsMmPerSec(low_test_velocity, 0);

  // Start recording left encoder values for the low portion of the test
  while ((i_log < step_log_length) && (millis() < high_time_ms)) {
    // Wait for the correct time to take a sample
    while (millis() < time_to_perform_next_sample_ms) { }
    // Record encoder value and PID output
    step_response_log[i_log] = encoders.getCountsLeft();
    PID_logged_output[i_log] = getLeftPIDoutput();
    // Increment iterator
    i_log++;
    // Update this variable to schedule when we'll take the next recording
    time_to_perform_next_sample_ms += test_sample_period_ms;
  }

  // Set left motor to high speed (provide the step input)
  setWheelVelsMmPerSec(high_test_velocity, 0);

  // Start recording left encoder values for the high portion of the test
  while (i_log < step_log_length) {
    // Wait for the correct time to take a sample
    while (millis() < time_to_perform_next_sample_ms) { }
    // Record encoder value and PID output
    step_response_log[i_log] = encoders.getCountsLeft();
    PID_logged_output[i_log] = getLeftPIDoutput();
    // Increment iterator
    i_log++;
    // Update this variable to schedule when we'll take the next recording
    time_to_perform_next_sample_ms += test_sample_period_ms;
  }

  // Switch off the motor
  setWheelVelsMmPerSec(0, 0);

  // Print header
  Serial.print("\n\n");
  for (int h = 0; h < 48; h++) {Serial.print("-");} Serial.print("\n");
  Serial.println("LEFT MOTOR PID STEP RESPONSE TEST");
  Serial.print("P_GAIN"); Serial.print("\t");
  Serial.print("I_GAIN"); Serial.print("\t");
  Serial.print("D_GAIN"); Serial.print("\t");
  Serial.print("FDFWD"); Serial.print("\t");
  Serial.print("LOOP_PERIOD (SEC)"); Serial.print("\n");
  Serial.print(P_GAIN); Serial.print("\t");
  Serial.print(I_GAIN); Serial.print("\t");
  Serial.print(D_GAIN); Serial.print("\t");
  Serial.print(FEEDFWD_GAIN, 4); Serial.print("\t");
  Serial.print(MOTOR_CTRL_LOOP_PERIOD_SEC); Serial.print("\n");
  for (int h = 0; h < 48; h++) {Serial.print("-");} Serial.print("\n");
  printLogToSerialPort();
}

int16_t step_delta;
float rev_per_sec;
float mm_per_sec;
void printLogToSerialPort() {
  // Print data headers:
  Serial.print(" TIME"); Serial.print("\t");
  Serial.print("|"); Serial.print("\t");
  Serial.print("VEL"); Serial.print("\t");
  Serial.print(""); Serial.print("\t");
  Serial.print(""); Serial.print("\t");
  Serial.print("| PIDOUT |"); Serial.print("\n");
  for (int h = 0; h < 48; h++) {Serial.print("-");} Serial.print("\n");

  Serial.print("t(ms)"); Serial.print("\t");
  Serial.print("steps"); Serial.print("\t");
  Serial.print("sDelta"); Serial.print("\t");
  Serial.print("rot/sec"); Serial.print("\t");
  Serial.print("mm/sec"); Serial.print("\t");
  Serial.print("MtrCmd"); Serial.print("\n");

  // Iterate through log, printing each line
  for (int i = 0; i < step_log_length; i++) {
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
    Serial.print("\t");

    // Print PID output
    Serial.print(PID_logged_output[i]);
    Serial.print("\n");
  }
}

void setup()
{
  // Set up interrupts and PID controllers
  setupMotorPIDs();

  // Set up serial link for logging
  Serial.begin(9600);

  // Set up display and show welcome message
  display.setLayout11x4();
  display.gotoXY(0, 0);
  display.print(F(" MOTOR PID"));
  display.gotoXY(0, 1);
  display.print(F("   TEST"));
  display.gotoXY(0, 2);
  display.print(F("B -> START"));

  // Wait for B button to be pressed to begin
  while(!buttonB.getSingleDebouncedPress()) {}

  // Run step response test, print results
  performLeftStepInputTest(low_velocity_mm_per_sec, high_velocity_mm_per_sec);

  // Run ramp
}


void loop()
{
  // Do nothing
  delay(10000);
}
