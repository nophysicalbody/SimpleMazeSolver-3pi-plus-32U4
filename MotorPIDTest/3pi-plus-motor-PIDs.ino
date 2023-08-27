#include <PID_v1.h>
#include "RunningAverage.h"

// Timer3 interrupt service routine (this runs at the periodicity defined in MOTOR_CTRL_LOOP_PERIOD_SEC)
ISR (TIMER3_COMPA_vect) {
  // Toggle LED to indicate the ISR is running
  toggleBuiltInLED();

  // Manage the motor PID control
  motorControlLoop();
}

void initTimer3() {
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 10000;
  TCCR3B = bit(WGM32) | bit(CS32);  // WGM32 => CTC(Clear Timer on Compare Match), CS32 => prescaler 1/256
  TIMSK3 = bit(OCIE3A);             // OCIE3A => Timer3 compare match A interrupt
  sei();
}

void setTimer3(float _timePeriod) {
  long cnt = 16000000 / 256 * _timePeriod;  // cnt = clk / prescaler * time(s)
  if(cnt > 65535) {
    cnt = 65535;        // "timer3 16bit counter over."
  }
  OCR3A = cnt;          // Output Compare Register Timer3A
  TIMSK3 = bit(OCIE3A);
}

void stopTimer3(){
    TIMSK3 = 0;
}

void restartTimer3(){
  TIMSK3 = bit(OCIE3A); 
}

// Variable for keeping track of LED state
volatile boolean timer3_out = HIGH;

void toggleBuiltInLED(){
  // flicker LED to indicate the loop is running
  digitalWrite(LED_BUILTIN, timer3_out);
  timer3_out = !timer3_out;
}


// Variables for velocity calculation and PID control
volatile static int16_t L_previous_encoder_count = encoders.getCountsLeft();
volatile static int16_t R_previous_encoder_count = encoders.getCountsRight();
int16_t L_current_counts; int16_t R_current_counts;
int16_t L_count_delta; int16_t R_count_delta;
volatile double L_current_speed_mm_per_sec = 0; volatile double R_current_speed_mm_per_sec = 0;
volatile double L_desired_speed_mm_per_sec = 0; volatile double R_desired_speed_mm_per_sec = 0;

volatile double L_PID_controller_output = 0; volatile double R_PID_controller_output = 0;
volatile double L_motor_output = 0; volatile double R_motor_output = 0;

volatile PID LmotorPID(&L_current_speed_mm_per_sec, &L_PID_controller_output, &L_desired_speed_mm_per_sec, P_GAIN, I_GAIN, D_GAIN, DIRECT);
volatile PID RmotorPID(&R_current_speed_mm_per_sec, &R_PID_controller_output, &R_desired_speed_mm_per_sec, P_GAIN, I_GAIN, D_GAIN, DIRECT);
volatile int16_t L_feedforward = 0; volatile int16_t R_feedforward = 0;

// Running average object for filtering noisy speed data
RunningAverage LeftSpeedRunningAverage(VEL_FILTER_ROLLING_AVG_BUFFER_SIZE);
RunningAverage RightSpeedRunningAverage(VEL_FILTER_ROLLING_AVG_BUFFER_SIZE);

void setupMotorPIDs(){
  // Use timer 3, as timer 1 is used for something on the 3pi+.
  initTimer3();
  
  // Configure timer 3 to call ISR every x seconds:
  setTimer3(MOTOR_CTRL_LOOP_PERIOD_SEC);
  
  // setup motor PID objects
  LmotorPID.SetMode(AUTOMATIC); RmotorPID.SetMode(AUTOMATIC);
  LmotorPID.SetOutputLimits(PID_LOWER_OUTPUT_LIMIT, PID_UPPER_OUTPUT_LIMIT); // by default, this library won't output negative values.
  RmotorPID.SetOutputLimits(PID_LOWER_OUTPUT_LIMIT, PID_UPPER_OUTPUT_LIMIT);

  // Also set up the LED, will use this to blink indicating that the ISR is running
  pinMode(LED_BUILTIN, OUTPUT);

  // Explicitly clear the running average
  LeftSpeedRunningAverage.clear();
  RightSpeedRunningAverage.clear();
}

double getLeftPIDoutput() {
  return L_motor_output;
}

void motorControlLoop(){
  // Compute current speed
  L_current_counts = encoders.getCountsLeft(); R_current_counts = encoders.getCountsRight();
  L_count_delta = L_current_counts - L_previous_encoder_count; R_count_delta = R_current_counts - R_previous_encoder_count;
  L_previous_encoder_count = L_current_counts; R_previous_encoder_count = R_current_counts; // update for next time

  // Take count delta, divide by 358.3 (counts/rev) to get revolutions, then multiply by 100.5 (mm/rotation) to get mm, then divide by time period
  //   L_current_speed_mm_per_sec = (double)L_count_delta * 100.5 / (358.3 * MOTOR_CTRL_LOOP_PERIOD_SEC);
  //   R_current_speed_mm_per_sec = (double)R_count_delta * 100.5 / (358.3 * MOTOR_CTRL_LOOP_PERIOD_SEC);
  LeftSpeedRunningAverage.addValue((double)L_count_delta * 100.5 / (358.3 * MOTOR_CTRL_LOOP_PERIOD_SEC));
  RightSpeedRunningAverage.addValue((double)R_count_delta * 100.5 / (358.3 * MOTOR_CTRL_LOOP_PERIOD_SEC));
  L_current_speed_mm_per_sec = LeftSpeedRunningAverage.getAverage();
  R_current_speed_mm_per_sec = RightSpeedRunningAverage.getAverage();

  
  // PID looks at current_speed_mm_per_sec and L_desired_speed_mm_per_sec and sets PID_controller_output
  LmotorPID.Compute(); RmotorPID.Compute();
  
  // Add PID output to feedforward term, set motor command to this value
  L_motor_output = L_PID_controller_output + L_feedforward; R_motor_output = R_PID_controller_output + R_feedforward;
  motors.setSpeeds(L_motor_output, R_motor_output);
}

void setWheelVelsMmPerSec (double L_newSpeed, double R_newSpeed){
  // disable the timer
  stopTimer3();
  
  // Update desired speed
  L_desired_speed_mm_per_sec = L_newSpeed; R_desired_speed_mm_per_sec = R_newSpeed;
  
  // Update feedforward term
  L_feedforward = int16_t(L_newSpeed * FEEDFWD_GAIN); R_feedforward = int16_t(R_newSpeed * FEEDFWD_GAIN);
  
  // Re-enable the timer
  restartTimer3();
}

