/* B3_07_Light.ino

   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   either towards the light or away from the light, based on readings
   from two photresistors.  Each resistor is set up in a simple voltage
   divider circuit:
   ----------------------------------------------------

                       PhotoR   ~7.5-10K
               +5  o---/\/\/--.--/\/\/---o GND
                              |
       Sensor Pins o-----------

   ----------------------------------------------------
   Modified from: http://playground.arduino.cc/Learning/PhotoResistor

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; June 7, 2015
   Revisions:
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <breadboardbotV2.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Constants

const byte LEFT_PR_PIN = A15;    // Define light sensor pins for left
const byte RIGHT_PR_PIN = A14;   // and right sides
const bool PHOTOTAXIS = false;    // GOes towards light (true) or away (false)
const byte BASESPEED = 80;       // Base speed.  Light values will reduce this to turn
const bool SIMPLE_LOGIC = false; // Use a simple or more complex approach
const byte BACKWARD_SPEED = 100;  // and backup/turn speed
const int  TURN_DURATION = 1000;   // Turn time in milliseconds
const byte MAX = 255;

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;

const byte LEFT_BUMP_PIN = 47;    // Define DIGITAL Pins for left
const byte RIGHT_BUMP_PIN = 46;   // and right bump sensors
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  Serial.begin(9600);  //Begin serial communcation
  pinMode(LEFT_BUMP_PIN,INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN,INPUT_PULLUP);

  delay(2000);  // Two second delay to get the robot ready
}

void loop() {
  /*  First determine the relative light levels and print the
      results out to the serial monitor */
  int leftLight = analogRead(LEFT_PR_PIN);
  int rightLight = analogRead(RIGHT_PR_PIN);
  Serial.print(leftLight);           // We can only write ONE thing
  Serial.print(" = left, right = "); // at a time, therefore it takes
  Serial.println(rightLight);        // three statemens to get our output.
  byte DIRECT = FORWARD;
  /* Now that we have some light levels, make each motor's speed
     a function of light level and drive either towards or
     away from the light depending on the value of PHOTOAXIS  */
  if (SIMPLE_LOGIC) {
    motorLeft->setSpeed(map(rightLight, 0, 500, 90, 255));
    motorRight->setSpeed(map(leftLight, 0, 500, 90, 255));
    byte direction;
    if (! PHOTOTAXIS) {
      direction = FORWARD;
    }
    else {
      direction = BACKWARD;
    }
    motorLeft->run(direction);
    motorRight->run(direction);
  }
  else {
    /* Find the direction of the strongest light by taking the difference
        between the readings */
    int deltaLight = abs(leftLight - rightLight);
    deltaLight = constrain(deltaLight, 0, 50);  // Keep that limit between 0 and 50

    byte leftSpeed = BASESPEED;
    byte rightSpeed = BASESPEED;

    if (! PHOTOTAXIS) {
      if (rightLight - leftLight > 0) {       // The LEFT side is the brightest,

    motorLeft->setSpeed(leftSpeed);
    motorRight->setSpeed(rightSpeed);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    // slow down the LEFT side to turn towards it
      }
      else if (leftLight - rightLight > 0 ) { // The RIGHT side is the brightest,
        if (rightSpeed > 0 && leftSpeed > 0) {
          rightSpeed = rightSpeed - 1;
          leftSpeed = leftSpeed - 1;
        }
        if (rightSpeed <= 0 && leftSpeed <= 0) {
          DIRECT = BACKWARD;
          rightSpeed++;
          leftSpeed++;
        }
        motorLeft->setSpeed(leftSpeed);
        motorRight->setSpeed(rightSpeed);
        motorLeft->run(DIRECT);
        motorRight->run(DIRECT);

        // slow down the RIGHT side to turn towards it
      }
    }
    else {
      if (deltaLight<490 && rightLight > 400 && leftLight > 400) {       // The LEFT side is the brightest,
        rightSpeed++;
        leftSpeed++;
        motorLeft->setSpeed(leftSpeed);
        motorRight->setSpeed(rightSpeed);
        motorLeft->run(FORWARD);
        motorRight->run(FORWARD);
        // slow down the RIGHT side to turn away
      }
  
  /* If you got here, one of the bump switches was closed */

  /* First check the LEFT sensor: */
  /*if(digitalRead(LEFT_BUMP_PIN) != HIGH) { // the LEFT side switch was bumped
    motorLeft->setSpeed(BACKWARD_SPEED/2); // Slowly back up and turn to right
    motorRight->setSpeed(BACKWARD_SPEED);  
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    delay(TURN_DURATION);                  // for specified duration
    motorLeft->run(RELEASE);               // Then stop power to the motors
    motorRight->run(RELEASE);              // and move to next section of code
  }*/

  /* Then check the right sensor: */
   /*if(digitalRead(RIGHT_BUMP_PIN) != HIGH) { // the RIGHT side switch was bumped
    motorLeft->setSpeed(BACKWARD_SPEED); // Slowly back up and turn to left
    motorRight->setSpeed(BACKWARD_SPEED/2);  
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    delay(TURN_DURATION);                 // for specified duration
    motorLeft->run(RELEASE);               // Then stop power to the motors
    motorRight->run(RELEASE);              // and move to next section of code
  }*/
  }
  delay(10); //Use a short delay.
}
}
//black is the leftlight
//blue is the rightlight

