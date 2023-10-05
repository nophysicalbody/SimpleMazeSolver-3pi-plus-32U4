/*
 * Simple3piMazeSolver - demo code for the Pololu 3pi Robot
 * 
 * Modified from example for original 3pi robot by Robin Sayers 2023 for ZEIT8034 Adv. T&E Techniques.
 * 
 * This code will solve a line maze constructed with a black line on a
 * white background, as long as there are no loops.  It has two
 * phases: first, it learns the maze, with a "left hand on the wall"
 * strategy, and computes the most efficient path to the finish.
 * Second, it follows its most efficient solution.
 *
 * http://www.pololu.com/docs/0J21
 * http://www.pololu.com
 * http://forum.pololu.com
 *
 */

// The following libraries will be needed by this demo
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

OLED display;
Buzzer buzzer;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
LineSensors lineSensors;
BumpSensors bumpSensors;
Motors motors;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

/**********************************
 * Configure robot settings here **
 **********************************/
// Factor A: must be a whole integer
uint16_t maxSpeed = 125; // This is the speed the robot will run while driving

// Factor B: must be a whole integer
uint16_t turnSpeed = 99; // This is the speed the motors will run while turning

/**********************************
 **** End configuration section ***
 **********************************/

// Optimised values from screening model
uint16_t angintSpeed = 61; // Drive straight a bit in case we entered the intersection at an angle.
uint16_t angintDelay = 38; // Drive straight a bit in case we entered the intersection at an angle.
uint16_t interSpeed = 40;
uint16_t interDelay = 90; // Intersection Motor Creep Run Delay
uint16_t turnDelay = 202;
uint16_t llbrakeoneSpeed = 72; // Learned Lap Braking Speed 1
uint16_t llbrakeoneDelay = 38; // Duration at braked speed one
uint16_t llbraketwoSpeed = 50; // Learned Lap Braking Speed 2
uint16_t llbraketwoDelay = 55; // Duration at braked speed two

// Other tuning values
int16_t minSpeed = 0;
// This is the speed the motors will run when centered on the line.
uint16_t baseSpeed = maxSpeed;
// This is the speed the motors will run while doing line sensor calibration
uint16_t calibrationSpeed = 60;

// PID configuration: This example is configured for a default
// proportional constant of 1/4 and a derivative constant of 1, which
// seems to work well at low speeds for all of our 3pi+ editions.  You
// will probably want to use trial and error to tune these constants
// for your particular 3pi+ and line course, especially if you
// increase the speed.

uint16_t proportional = 64; // coefficient of the P term * 256
uint16_t derivative = 256; // coefficient of the D term * 256

//// A couple of simple tunes, stored in program space.
const char welcome[] PROGMEM = ">g32>>c32";

// Sets up special characters in the LCD so that we can display
// bar graphs.
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  display.loadCustomCharacter(levels + 0, 0);  // 1 bar
  display.loadCustomCharacter(levels + 1, 1);  // 2 bars
  display.loadCustomCharacter(levels + 2, 2);  // 3 bars
  display.loadCustomCharacter(levels + 3, 3);  // 4 bars
  display.loadCustomCharacter(levels + 4, 4);  // 5 bars
  display.loadCustomCharacter(levels + 5, 5);  // 6 bars
  display.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

void printBar(uint8_t height)
{
  if (height > 8) { height = 8; }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, (char)255};
  display.print(barChars[height]);
}

void calibrateSensors()
{
  display.clear();
  display.gotoXY(0, 1);
  display.print(F("Calibrate"));
  display.gotoXY(0, 2);
  display.print(F("Sensors"));

  // Wait 1 second and then begin automatic sensor calibration
  // by rotating in place to sweep the sensors over the line
  delay(3000);
  uint16_t calibrationSpeed = 40;
  for(uint16_t i = 0; i < 80; i++)
  {
    if (i > 20 && i <= 60)
    {
      motors.setSpeeds(-(int16_t)calibrationSpeed, calibrationSpeed);
    }
    else
    {
      motors.setSpeeds(calibrationSpeed, -(int16_t)calibrationSpeed);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
}

// Displays the estimated line position and a bar graph of sensor
// readings on the LCD. Returns after the user presses B.
void showReadings()
{
  display.clear();

  while(!buttonB.getSingleDebouncedPress())
  {
    uint16_t position = lineSensors.readLineBlack(lineSensorValues);

    display.gotoXY(0, 0);
    display.print("Pos: ");
    display.print(position);
    display.print("    ");
    display.gotoXY(3, 1);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
    display.gotoXY(3, 2);
    display.print("L C R");
    display.gotoXY(0, 3);
    display.print("B: Start");
    delay(50);
  }
}

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
void setup()
{
  // Load custom characters so the display works properly
  loadCustomCharacters();
  
  // Increase resolution of OLED display
  display.setLayout11x4();
  display.clear();
  display.gotoXY(0, 0);
  display.print(F("Pololu"));
  display.gotoXY(0, 1);
  display.print(F("3\xf7 Robot"));
  display.gotoXY(0, 2);
  display.print(F("Maze solver"));
  // Play a little welcome song
  buzzer.play(">g32>>c32");
  delay(1500);

  // Display selected factors:
  display.clear();
  display.gotoXY(0, 0);
  display.print(F("   Robot"));
  display.gotoXY(0, 1);
  display.print(F(" Settings"));
  display.gotoXY(0, 2);
  display.print(F("A: "));
  display.print(baseSpeed);
  display.gotoXY(0, 3);
  display.print(F("B: "));
  display.print(turnSpeed);
  delay(2500);
  
  // Display battery voltage and wait for button press
  int bat = readBatteryMillivolts();
  display.clear();
  display.gotoXY(0, 0);
  display.print(F("Batt volts:"));
  display.gotoXY(0, 1);
  display.print(bat);
  display.print("mV");
  display.gotoXY(0, 2);
  display.print(F("Press B to"));
  display.gotoXY(0, 3);
  display.print(F("calibrate"));
  while(!buttonB.getSingleDebouncedPress()) {}
  
  // Play a note so we know we've hit the button
  buzzer.play(">>a32");
  
  // Always wait for the button to be released so that 3pi doesn't
  // start moving until your hand is away from it.
  // In this case I'll just wait
  delay(500);
  
  // Auto-calibration: turn right and left while calibrating the
  // sensors.
  display.clear();
  calibrateSensors();
  showReadings();

  // showreadings function exits when user presses B, let's start!


//  // Test for line follow function - run robot on single strip of tape and it will pace
//  while (1)
//  {
//    // Pace
//    follow_segment();
//    turn('B');
//  }
//  motors.setSpeeds(0, 0);
}

int16_t lastError = 0;
// This function, causes the 3pi to follow a segment of the maze until
// it detects an intersection, a dead end, or the finish.
void follow_segment()
{
  lastError = 0;
  while (1)
  {
    // Get the position of the line.  Note that we *must* provide
    // the "lineSensorValues" argument to readLineBlack() here, even
    // though we are not interested in the individual sensor
    // readings. *Edit: WE ARE
    int16_t position = lineSensors.readLineBlack(lineSensorValues);
  
    // Our "error" is how far we are away from the center of the
    // line, which corresponds to position 2000.
    int16_t error = position - 2000;
  
    // Get motor speed difference using proportional and derivative
    // PID terms (the integral term is generally not very useful
    // for line following).
    int16_t speedDifference = error * (int32_t)proportional / 256  + (error - lastError) * (int32_t)derivative / 256;
  
    lastError = error;
  
    // Get individual motor speeds.  The sign of speedDifference
    // determines if the robot turns left or right.
    int16_t leftSpeed = (int16_t)baseSpeed + speedDifference;
    int16_t rightSpeed = (int16_t)baseSpeed - speedDifference;
  
    // Constrain our motor speeds to be between 0 and maxSpeed.
    // One motor will always be turning at maxSpeed, and the other
    // will be at maxSpeed-|speedDifference| if that is positive,
    // else it will be stationary.  For some applications, you
    // might want to allow the motor speed to go negative so that
    // it can spin in reverse.
    leftSpeed = constrain(leftSpeed, minSpeed, (int16_t)maxSpeed);
    rightSpeed = constrain(rightSpeed, minSpeed, (int16_t)maxSpeed);
  
    motors.setSpeeds(leftSpeed, rightSpeed);
    // We use the inner three sensors (1, 2, and 3) for
    // determining whether there is a line straight ahead, and the
    // sensors 0 and 4 for detecting lines going to the left and
    // right.
    if (lineSensorValues[1] < 100 && lineSensorValues[2] < 100 && lineSensorValues[3] < 100)
    {
      // There is no line visible ahead, and we didn't see any
      // intersection.  Must be a dead end.
      return;
    }
    else if (lineSensorValues[0] > 200 || lineSensorValues[4] > 200)
    {
      // Found an intersection.
      return;
    }
  }
}


// Code to perform various types of turns according to the parameter dir,
// which should be 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
// The delays here had to be calibrated for the 3pi's motors.
void turn(unsigned char dir)
{
  switch(dir)
  {
  case 'L':
    // Turn left.
    motors.setSpeeds(-turnSpeed, turnSpeed);
    delay(turnDelay);
    break;
  case 'R':
    // Turn right.
    motors.setSpeeds(turnSpeed, -turnSpeed);
    delay(turnDelay);
    break;
  case 'B':
    // Turn around.
    motors.setSpeeds(turnSpeed, -turnSpeed);
    delay(2 * turnDelay);
    break;
  case 'S':
    // Don't do anything!
    break;
  }
}

// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
//
// Whenever the robot makes a U-turn, the path can be simplified by
// removing the dead end.  The follow_next_turn() function checks for
// this case every time it makes a turn, and it simplifies the path
// appropriately.
char path[100] = "";
unsigned char path_length = 0; // the length of the path

// Displays the current path on the LCD, using two rows if necessary.
void display_path()
{
  // Set the last character of the path to a 0 so that the print()
  // function can find the end of the string.  This is how strings
  // are normally terminated in C.
  path[path_length] = 0;
  
  // Set to a layout that can hold more characters!
  display.setLayout21x8();
  display.clear();
  display.gotoXY(0, 0);
  display.print("Path found:");
  display.gotoXY(0, 1);
  display.print(path);

  if (path_length > 21)
  {
    display.gotoXY(0, 2);
    display.print(path + 21);
  }
  if (path_length > 42)
  {
    display.gotoXY(0, 3);
    display.print(path + 42);
  }
  if (path_length > 63)
  {
    display.gotoXY(0, 4);
    display.print(path + 63);
  }
  if (path_length > 84)
  {
    display.gotoXY(0, 5);
    display.print(path + 84);
  }
  // Reset to the display layout used by the rest of the display functions
  display.setLayout11x4();
}

void display_lap_time()
{
    display.clear();
    display.print(F("A:"));
    display.print(baseSpeed);
    display.print(F(" B:"));
    display.print(turnSpeed);
    display.gotoXY(0, 1);
    display.print(get_timer_value_millis());
    display.print(" ms");
    display.gotoXY(0, 3);
    display.print("B-continue");
}

// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
unsigned char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{
  // Make a decision about how to turn.  The following code
  // implements a left-hand-on-the-wall strategy, where we always
  // turn as far to the left as possible.
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
}

// Path simplification.  The strategy is that whenever we encounter a
// sequence xBx, we can simplify it by cutting out the dead end.  For
// example, LBL -> S, because a single S bypasses the dead end
// represented by LBL.
void simplify_path()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if (path_length < 3 || path[path_length-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (path[path_length - i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;

  // Replace all of those turns with a single one.
  switch (total_angle)
  {
  case 0:
    path[path_length - 3] = 'S';
    break;
  case 90:
    path[path_length - 3] = 'R';
    break;
  case 180:
    path[path_length - 3] = 'B';
    break;
  case 270:
    path[path_length - 3] = 'L';
    break;
  }

  // The path is now two steps shorter.
  path_length -= 2;
}

// Create a timer to use for measuring run times
unsigned long start_time_millis = 0;
unsigned long stop_time_millis = 0;
void start_timer()
{
  start_time_millis = millis();
}

void stop_timer()
{
  stop_time_millis = millis();
}

unsigned long get_timer_value_millis()
{
  // Make sure both variables are going to give a sensible result
  if (stop_time_millis > start_time_millis)
  {
    return stop_time_millis - start_time_millis;
  }
  else
  {
    return 0;
  }
}

bool pause_wait_for_button_press(unsigned long delay_length)
// This function waits for the requested number of milliseconds, returning true if button
// is pressed and false if timer expires
{
  bool button_pressed = false;
  for (int j = 0; j < delay_length/100; j++) {
      if (buttonB.getSingleDebouncedPress())
      {
        button_pressed = true;
        break;
      } else {
        delay(100);
      }
    }
  return button_pressed;
}

// This function comprises the body of the maze-solving program.  It is called
// repeatedly by the Arduino framework.
void loop()
{
  /*
   * For testing only
   */
//  for(int i = 0; i < 57; i++){
//    path[i]=char(i+65);
//    path_length++;
//  }
//  // Display the path on the LCD.
//  display_path();
//  delay(10000);
  /*
   * End testing
   */
  // Start "learning lap"
  // Play music and wait for it to finish before we start driving.
  buzzer.play("L16 cdegreg4");
  display.clear();
  display.print(F("Get ready"));
  while(buzzer.isPlaying());
  delay(1000);

  display.clear();
  display.gotoXY(4, 1);
  display.print(F("GO!"));
  buzzer.play(">>a32");
  start_timer();
  while (1)
  {
    follow_segment();

    // Drive straight a bit.  This helps us in case we entered the
    // intersection at an angle.
    // Note that we are slowing down - this prevents the robot
    // from tipping forward too much.
    motors.setSpeeds(angintSpeed, angintSpeed);
    delay(angintDelay);

    // These variables record whether the robot has seen a line to the
    // left, straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    // Now read the sensors and check the intersection type.
    unsigned int sensors[5];
    //robot.readLine(sensors, IR_EMITTERS_ON);
    lineSensors.readLineBlack(sensors);

    // Check for left and right exits.
    if (sensors[0] > 200)
      found_left = 1;
    if (sensors[4] > 200)
      found_right = 1;

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.
    motors.setSpeeds(interSpeed, interSpeed);
    delay(interDelay);

    // Check for a straight exit.
    //robot.readLine(sensors, IR_EMITTERS_ON);
    lineSensors.readLineBlack(sensors);
    if (sensors[1] > 300 || sensors[2] > 300 || sensors[3] > 300)
      found_straight = 1;

    // Check for the ending spot.
    // If all three middle sensors are on dark black, we have
    // solved the maze.
    if (sensors[1] > 600 && sensors[2] > 600 && sensors[3] > 600)
      break;

    // Intersection identification is complete.
    // If the maze has been solved, we can follow the existing
    // path.  Otherwise, we need to learn the solution.
    unsigned char dir = select_turn(found_left, found_straight, found_right);

    // Make the turn indicated by the path.
    turn(dir);

    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length++;

    // You should check to make sure that the path_length does not
    // exceed the bounds of the array.  We'll ignore that in this
    // example.

    // Simplify the learned path.
    simplify_path();

    // Display the path on the LCD.
    display_path();
  }

  // Solved the maze!
  stop_timer();

  // Now enter an infinite loop - we can re-run the maze as many
  // times as we want to.
  while (1)
  {
    // Beep to show that we solved the maze.
    motors.setSpeeds(0, 0);
    buzzer.play(">>a32");
    
    while(1)
    {
      // Show lap time and prompt to re-run for 4 seconds
      display_lap_time();
      if (pause_wait_for_button_press(5000)) break;
      // Show the path for 3 seconds
      display_path();
      if (pause_wait_for_button_press(2000)) break;
    }
    buzzer.play("L16 cdegreg4");
    display.clear();
    display.print(F("Get ready"));
    while(buzzer.isPlaying());
    delay(1000);
  
    display.clear();
    display.gotoXY(0, 1);
    display.print("Re-running");
    display.gotoXY(0, 2);
    display.print("learned lap");
    buzzer.play(">>a32");

    // Re-run the maze.  It's not necessary to identify the
    // intersections, so this loop is really simple.
    buzzer.play(">>a32");
    start_timer();
    int i;
    for (i = 0; i < path_length; i++)
    {
      follow_segment();

      // Drive straight while slowing down, as before.
      motors.setSpeeds(llbrakeoneSpeed, llbrakeoneSpeed);
      delay(llbrakeoneDelay);
      motors.setSpeeds(llbraketwoSpeed, llbraketwoSpeed);
      delay(llbraketwoDelay);

      // Make a turn according to the instruction stored in
      // path[i].
      turn(path[i]);
    }

    // Follow the last segment up to the finish.
    follow_segment();

    // Now we should be at the finish!  Restart the loop.
    stop_timer();
  }
}
