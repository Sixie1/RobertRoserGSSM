/* B3_09_LineFollow_Left.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   forward, following a black line, but pretty inefficiently.
   When a whisker bump sensor on either side hits something,
   back up and turn slightly away from that direction and resume
   forward path.
   Stop at a specified distance from an object directly ahead.
   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438
   Programmer: Dave Eslinger; June 13, 2015
   Revisions:
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <breadboardbot.h>
#include <NewPing.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

/* Define Constants */

// IO Pins used
const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors
const byte SONIC_TRIGGER_PIN = 50;
const byte SONIC_ECHO_PIN = 51;
const byte LEFT_IR_LED_PIN = 43;
const byte LEFT_IR_PIN = A11;
const byte RIGHT_IR_LED_PIN = 42;
const byte RIGHT_IR_PIN = A10;
int BACK= 60;
// Parameters controlling program behavior
// Bump behavior
const byte FORWARD_SPEED = 90;   // Define normal speeds
const byte BACKWARD_SPEED = 100;  // and backup/turn speed.
const int  TURN_DURATION = 600;   // Turn length in milliseconds

// Sonic sensor
const float TARGET_DISTANCE_INCHES = 10;
const int MAX_SONIC_DISTANCE = 500;      // cm, optional, 500 cm is default

// Line Following
const byte FOLLOW_LEFT_SIDE = 1;
const float HI_LIGHT = 0.80;     // Keep the BRIGHT side of the line ABOVE this
// fraction of the Maximum read by that sensor
const float LO_LIGHT = 0.20;     // Keep the DARK side of the line BELOW this
// fraction of the Maximum read by that sensor

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

/* Define new untrasonic sensor with trigger & echo pins and
   the maximum distance to be sensed. */
NewPing sonic(SONIC_TRIGGER_PIN, SONIC_ECHO_PIN, MAX_SONIC_DISTANCE);

/* Global Variables */
int leftIR;
int minLeftIR = 1026;
int maxLeftIR = -1024;
int rightIR;
int minRightIR = 1026;
int maxRightIR = -1024;
float brightLeft;
float darkRight;
float leftRange;
float rightRange;

void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  Serial.begin(9600);  //Begin serial communcation

  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit,
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN, INPUT_PULLUP);

  /* Define IR LED source pins as OUTPUTs and turn on the IR LEDs */
  pinMode(LEFT_IR_LED_PIN, OUTPUT);
  pinMode(RIGHT_IR_LED_PIN, OUTPUT);
  digitalWrite(LEFT_IR_LED_PIN, HIGH);
  digitalWrite(RIGHT_IR_LED_PIN, HIGH);

  /* Calibrate the IR sensors
     1) Place over the brightest target and hold down right bumper switch.
     2) Move back and forth to read various bright values, release bumper.
     3) Place over the darkest target and hold down right bumper again.
     4) All done!  The program
        calculates the 80% and 20% (or as set by HI_LIGHT% and LO_LIGHT% values)
        target values to keep the sensors at.
  */
  while (digitalRead(RIGHT_BUMP_PIN) == HIGH) {} // Pause until the pin is grounded (our switch)
  while (digitalRead(RIGHT_BUMP_PIN) == LOW) { // Get high light values until the pin
    // is no longer grounded
    leftIR = analogRead(LEFT_IR_PIN);
    rightIR = analogRead(RIGHT_IR_PIN);
    Serial.print(leftIR);
    Serial.print("  = leftIR, Calibrate LIGHT rightIR = ");
    Serial.println(rightIR);
    if ( leftIR > maxLeftIR ) {
      maxLeftIR = leftIR;
    }
    if ( rightIR > maxRightIR ) {
      maxRightIR = rightIR;
    }
  }
  /*Do the same, but over a dark target */
  while (digitalRead(RIGHT_BUMP_PIN) == HIGH) {} // Pause until the pin is grounded (our switch)
  while (digitalRead(RIGHT_BUMP_PIN) == LOW) {   // Get dark light values until the pin
    // is no longer grounded
    leftIR = analogRead(LEFT_IR_PIN);
    rightIR = analogRead(RIGHT_IR_PIN);
    Serial.print(leftIR);
    Serial.print("  = leftIR, Calibrate DARK  rightIR = ");
    Serial.println(rightIR);
    if ( leftIR < minLeftIR ) {
      minLeftIR = leftIR;
    }
    if ( rightIR < minRightIR ) {
      minRightIR = rightIR;
    }
  }
  leftRange = maxLeftIR - minLeftIR;
  rightRange = maxRightIR - minRightIR;
  brightLeft = maxLeftIR - ((1.0 - HI_LIGHT) * leftRange);
  darkRight = minRightIR + (LO_LIGHT * rightRange);

  while ( digitalRead(LEFT_BUMP_PIN) == HIGH) {} // Pause until you trigger the left bump switch
  delay(1000);                      // Then wait 1 second and start line following.
}

void loop() {

  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
  delay(20);
  leftIR = analogRead(LEFT_IR_PIN);
  rightIR = analogRead(RIGHT_IR_PIN);
  Serial.print(leftIR);
  Serial.print("  = leftIR,brightLeft = ");
  Serial.print(brightLeft);
  Serial.print(", darkRight = ");
  Serial.print(darkRight);
  Serial.print(", rightIR = ");
  Serial.println(rightIR);

  motorLeft->setSpeed(FORWARD_SPEED);
  motorRight->setSpeed(FORWARD_SPEED);

  if (leftIR < brightLeft ) {          // Too far right, stop left motor
    motorLeft->setSpeed(BACK);
    motorLeft->run(BACKWARD);
    delay(30);
  }
  else if (rightIR > darkRight) {
    motorRight->setSpeed(BACK);
    motorRight->run(BACKWARD);
    delay(30);
    // Too far right, stop RIGHT motor
  }

}
