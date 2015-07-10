/* B3_05_DriveTest.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   in a series of timed intervals, stopping at end of each.
   Begin running on push of Right Bump switch.
   Start a new iteration for each speed on each left bump switch, up to
   5 iterations.  Then wait for a right switch to move to next speed.

   TLDR: Right switch to start each speed series.  Left switch to start
         each run within a series.


   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; June 7, 2015
   Major revisions:
         7/3/2015 DLE: renamed, changed motorshield pointer passing)
         7/6/2015 DLE: Changed to use bump switches for running
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <breadboardbot.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Pin Assignments
const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors
// Define Constants
#define TESTTIME 1000
#define WAITTIME 1000

// Define 'ports' for motors.
const byte LEFTMOTORPORT = 3;
const byte RIGHTMOTORPORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFTMOTORPORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHTMOTORPORT);

void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);

  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit,
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);
}

void loop(void) {

  // delay(2000); // Delay no longer needed!
  while (digitalRead(RIGHT_BUMP_PIN) == HIGH) {} // PAUSE until Right bumper hit

  /* Autonomous loop for determining power and distance, will
     run through each power 5 times, for replicates, then advance power
     by 50.  Therefore it tests: 50, 100, 150, 200 and 250.
  */

  for ( byte leg = 0; leg <= 4; leg++ ) {
    byte speed = 50 + 50 * leg;
    for (byte trial = 1; trial <= 5; trial++ ) {
      delay(WAITTIME);  // Wait for hands to get out of the way!
      motorLeft->setSpeed(speed);  // Set both speeds
      motorRight->setSpeed(speed);
      motorLeft->run(FORWARD);
      motorRight->run(FORWARD);
      delay(TESTTIME);  // Run for test time
      allStop(FORWARD, motorLeft, motorRight);
      while (digitalRead(LEFT_BUMP_PIN) == HIGH) {} // PAUSE until Left bumper hit
    }
    while (digitalRead(RIGHT_BUMP_PIN) == HIGH) {} // PAUSE until Right bumper hit

  }
  // One last loop for full speed, 255, just for fun!
  for (byte trial = 1; trial <= 5; trial++ ) {
    delay(WAITTIME);  // Wait for hands to get out of the way!
    motorLeft->setSpeed(255);  // Set both speeds
    motorRight->setSpeed(255);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    delay(TESTTIME);  // Run for 1 second
    allStop(FORWARD, motorLeft, motorRight);
    while (digitalRead(LEFT_BUMP_PIN) == HIGH) {} // PAUSE until Left bumper hit
  }
  while (1) {}  // Infinite loop to stop the loop function
}
