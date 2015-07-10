/* B3_06_SpinSquare.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   in a square, stopping at (or near!) end point.

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
const byte TESTSPEED = 60;
const byte DIRECTION = TURNLEFT;

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

void setup(void){
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void loop(void){
  
  delay(1000); // Delay for a second to give time to place the robot.

  // Autonomous loop for driving in a square
  for ( byte leg = 1; leg <= 4; leg++ ) {
    drive(10., TESTSPEED, motorLeft, motorRight);// Forward for 10 inches
    spin(-90., TESTSPEED, motorLeft, motorRight); // 90 deg. to left
    allStop(FORWARD, motorLeft, motorRight);
    delay(500);  // Pause for 1/2 a second after turning
  }
  delay(3000);
  for ( byte leg = 1; leg <= 4; leg++ ) {
    drive(10., TESTSPEED, motorLeft, motorRight);// Forward for 10 inches
    spin(90., TESTSPEED, motorLeft, motorRight); // 90 deg. to left
    allStop(FORWARD, motorLeft, motorRight);
    delay(500);  // Pause for 1/2 a second after turning
  }
  while(1){}
}
