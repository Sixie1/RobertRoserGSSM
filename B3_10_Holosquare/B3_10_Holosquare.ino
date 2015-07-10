/* B3_10_HoloSquare.ino
   This is a test sketch for autonomous driving mode of
   the three-wheeled drive Bread Board Bot (BBbot, B^3)

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer: Dave Eslinger; December 2, 2014
   Revisions: 2015, May 25:   Changed for new motor configuration. DLE
              2015, June 12:  Changed into B3_ code style for GoSciTech 2015. DLE
              2015, July 9: Name change, cleaned up and additional comments added. DLE
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <math.h>
#include <breadboardbot.h>
#include <NewPing.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Define Constants
const float cos30sin60 = sqrt(3.0) / 2.0; // cos(30 deg) = sin(60 deg), need for wheel
// vector calcs.

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
const byte BACK_MOTOR_PORT = 4;
const byte SONIC_TRIGGER_PIN = 50;
const byte SONIC_ECHO_PIN = 51;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);
Adafruit_DCMotor *motorBack = AFMS.getMotor(BACK_MOTOR_PORT);

// Define global variables
float direction;       // Velocity Vector Angle (DEGREES) from forward to drive
float magnitude = 50;       // Magnitude (0-100) of movement vector in given direction
float duration = 2;        // Duration to drive at given velocity vector

byte motorLeftdir;     // Clockwise or Counter clockwise for the 3 wheels
byte motorBackdir;
byte motorRightdir;
const float TARGET_DISTANCE_INCHES = 10;
const int MAX_SONIC_DISTANCE = 500;
NewPing sonic(SONIC_TRIGGER_PIN, SONIC_ECHO_PIN, MAX_SONIC_DISTANCE);
byte right = 0;
byte left = 0;

void setup(void) {
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors
  motorLeft->run(RELEASE);
  motorBack->run(RELEASE);
  motorRight->run(RELEASE);
}
bool brake = false;
void loop(void) {
  int oldMillis = millis + 2000;
  // Section for taking commands from Serial Input
  // N.B.  Need to comment out one bracket at end for the autonomous loop below
  /* if (Serial.available() > 0) { */
  /*   direction = Serial.parseInt(); */
  /*   magnitude = Serial.parseInt(); */
  /*   duration = Serial.parseInt(); */
  /* }  */

  /* Autonomous loop for driving in a square */
while(millis < oldMillis &&  Distance_inches(sonic.ping()) > TARGET_DISTANCE_INCHES)
  if (Distance_inches(sonic.ping()) > TARGET_DISTANCE_INCHES && left >= right) {

//  if (Distance_inches(sonic.ping()) < TARGET_DISTANCE_INCHES) {
//    motorLeft->run(RELEASE);
//    motorBack->run(RELEASE);
//    motorRight->run(RELEASE);
//    delay(2000);
//    direction = 0.;
//    holodrive(90, direction, 0.5);
//    direction = 180.;
//    holodrive(90, direction, 0.5);
//  }
    direction = 90.;
    holodrive(90., direction, 1);
    right++;
    left = 0;  
}
  
  if (Distance_inches(sonic.ping()) > TARGET_DISTANCE_INCHES && right > left) {

//   if (Distance_inches(sonic.ping()) < TARGET_DISTANCE_INCHES) {
//    motorRight->run(RELEASE);
//    motorLeft->run(RELEASE);
//    motorBack->run(RELEASE);
//    delay(2000);
//    direction = 0.;
//    holodrive(90, direction, 0.5);
//    direction = 180.;
//    holodrive(90, direction, 0.5);
//}
   direction = -90.;
   holodrive(90., direction, 1);
   left++;
   right = 0;
  }
  



  magnitude = 0;
  duration = 1;
  direction = 0.;
  magnitude = 50;
  duration = 0.5;
  direction = 0.;
  direction = 180.;
}



void holodrive(int magnitude, int direction, int duarion) {
  //  if ( duration > 0 ) {
  //    Serial.print("direction = ");
  //    Serial.print(direction);
  //    Serial.print(", magnitude = ");
  //    Serial.print(magnitude);
  //    Serial.print(" and duration = ");
  //    Serial.println(duration);

  float xVector = magnitude * sin((M_PI * direction) / 180.);
  float yVector = magnitude * cos((M_PI * direction) / 180.);
  Serial.print("xVector, yVector = ");
  Serial.print(xVector);
  Serial.print(", ");
  Serial.println(yVector);

  // Find relative power needed for each wheel based on the target velocity vector
  float backPower = -xVector * 0.9;  // Multiply by fudge factor to prevent rotation if needed
  float leftPower = 0.5 * xVector - cos30sin60 * yVector;
  float rightPower = 0.5 * xVector + cos30sin60 * yVector;

  // Find the actual motor speeds, 0-255, needed.  N.B. still need motor direction!
  byte backSpeed  = map(abs(backPower),  0, 100, 0, 255);
  byte leftSpeed  = map(abs(leftPower),  0, 100, 0, 255);
  byte rightSpeed = map(abs(rightPower), 0, 100, 0, 255);

  // Set the speeds
  motorBack-> setSpeed(backSpeed);
  motorLeft-> setSpeed(leftSpeed);
  motorRight->setSpeed(rightSpeed);

  /* Set Motor directions.  For Adafruit V2 Motorshield:
       1 is Clockwise (Positive motor direction, FORWARD)
       2 is Counterclockwise (Negative vector direction, BACKWARD)
       3 is Brake (Doesn't work at present)
       4 is Release = stop power, not driving, but not brake

     We can use a trinary operator to set direction within run call
  */
  motorBack-> run((backPower  > 0) ? FORWARD : BACKWARD );
  motorLeft-> run((leftPower  > 0) ? BACKWARD : FORWARD );
  motorRight->run((rightPower > 0) ? FORWARD : BACKWARD );

  //Print out motor control details
  Serial.print("Speeds Back,Left,Right = ");
  Serial.print(copysign(backPower, backSpeed));
  Serial.print(", ");
  Serial.print(copysign(leftPower, leftSpeed));
  Serial.print(", ");
  Serial.println(copysign(rightPower, rightSpeed));

  // Run motors for the duration needed, converting from seconds to milliseconds
  delay(1000 * duration);
  if (brake) {            // Not a real brake, but set power = 0, stop driving motors
    motorBack->setSpeed(0);
    motorLeft->setSpeed(0);
    motorRight->setSpeed(0);
    motorBack-> run(RELEASE);
    motorLeft-> run(RELEASE);
    motorRight->run(RELEASE);
  }
}
//  else {                    // no duration entered, so stop all motors
//    motorBack->setSpeed(0);
//    motorLeft->setSpeed(0);
//    motorRight->setSpeed(0);
//  }
//}
//}
/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
/* void serialEvent() { */
/*   while (Serial.available()) { */
/*     // get the new byte: */
/*     char inChar = (char)Serial.read();  */
/*     // add it to the inputString: */
/*     inputString += inChar; */
/*     // if the incoming character is a neleftPowerine, set a flag */
/*     // so the main loop can do something about it: */
/*     if (inChar == '\n') { */
/*       stringComplete = true; */
/*     }  */
/*   } */
/* } */
