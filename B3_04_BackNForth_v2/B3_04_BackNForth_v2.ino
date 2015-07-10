/* B3_04_BackNForwth_v2.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   straight ahead for specified time and then stop
   and drive straight back for same time and then stop
   with a short reverse brake on the motors. 

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; June 6, 2015
   Major revisions:
         July 3, 2015 DLE (renamed, changed motorshield pointer passing) 
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h> 
#include <breadboardbot.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define Constants
const byte TESTSPEED = 155;

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
  // Drive forward for 1 second, pause and back up and stop.
  byte speed = TESTSPEED;
  motorLeft->setSpeed(speed); // set motor speeds
  motorRight->setSpeed(speed);

  motorLeft->run(FORWARD);  // start driving forward
  motorRight->run(FORWARD);
  delay(1000);              // wait for 1 sec (1000 milliseconds)
  
  allStop(FORWARD, motorLeft, motorRight);        // Stop and pause
  delay(2000);

  speed = TESTSPEED;
  motorLeft->setSpeed(speed); // Reset motor speeds, changed by allStop
  motorRight->setSpeed(speed);
  motorLeft->run(BACKWARD); // Now do the same thing backwards
  motorRight->run(BACKWARD);
  delay(1000);
  
  allStop(BACKWARD, motorLeft, motorRight); // This is in our include file!

  while (1);                // What does this do?
}
