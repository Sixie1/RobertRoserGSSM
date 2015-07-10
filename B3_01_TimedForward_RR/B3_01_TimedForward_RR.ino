/* B3_TimedForward.ino
   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   straight ahead for specified time and then stop
   
   Intially commented out code will impliment a a short 
   reverse brake on the motors. 

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger (DLE); June 3, 2015
   Major revisions:  
         6/7/2015, DLE: Cleaned it up for GoSciTech
         7/4/2015 DLE renamed

*/
#include <Wire.h>
#include <Adafruit_MotorShield.h> 
#include <math.h>

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
  // Turn off all motors
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

void loop(void){
  // Drive forward for 1 second at X/255 (~60% power)
  int duration = 1000; // in milliseconds
  byte speed = 255;
  motorLeft->setSpeed(150);
  motorRight->setSpeed(150);
  
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
  
 delay(1000);
  
  motorLeft->setSpeed(150);
  motorRight->setSpeed(150);
  
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
  
  delay(1000);
  
}
  
 // motorLeft->run(RELEASE);   // Now stop the motors
  //motorRight->run(RELEASE);  // However, this just coasts

  /*motorLeft->run(FORWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(250);
  
  motorLeft->run(BACKWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(duration);
  
  
  motorLeft->run(FORWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(250);
  
  motorLeft->run(BACKWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(duration);
  
  motorLeft->run(FORWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(250);
  
  motorLeft->run(BACKWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(duration);
  
  motorLeft->run(FORWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(250);
  
  motorLeft->run(BACKWARD);  // Run backwards for a brief period
  motorRight->run(BACKWARD);
  delay(duration);
  

  motorLeft->run(RELEASE);     // and then stop the motors
  motorRight->run(RELEASE);   

  delay(2000);       // Pause for 2 seconds
//while (1) {};      // What does this do?
}

/*void allStop(int direction) {
  motorLeft->setSpeed(100); // 100 on a 0-255 scale.
  motorRight->setSpeed(100);
  if (direction == FORWARD) {
    motorLeft->run(FORWARD);
    motorRight->run(BACKWARD);
  }
  else {
    
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
  //delay(50);
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  return;
}*/

