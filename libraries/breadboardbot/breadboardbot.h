/* Common Funtions for Bread Board Bot

   Arduino Robotics summer camp at GoSciTech 2015nn
   SC Governor's School of Science and Math

   Arduino: Arduino Mega 256 v3 Clone
   Motor Shield: Adafruit assembled Motor Shield for Arduino v2
   ---->  http://www.adafruit.com/products/1438

   Programmer & Instructor: Dave Eslinger; June 6, 2015
   Major revisions: 
         July 3, 2015 DLE (changed motorshield pointer passing) 
*/


#include <Adafruit_MotorShield.h> 

#define TURNLEFT 0
#define TURNRIGHT 1

const float tireDiam = 2.75; // inches.  N.B. The units here determine units
const float trackWidth = 10.0; // inches      used everywhere for distance

void allStop(int oldDirection, 
	     Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
  mLeft->setSpeed(100);  // Note that we reset the speeds here; therefore, 
  mRight->setSpeed(100); // we need to reset them in other routine.
  if (oldDirection == FORWARD) {
    mLeft->run(BACKWARD);
    mRight->run(BACKWARD);
  }
  else {
    mLeft->run(BACKWARD);
    mRight->run(BACKWARD);
  }
  delay(50);
  mLeft->run(RELEASE);
  mRight->run(RELEASE);
  return;
}

float duration_per_distance( float distance, byte speed) {
  /* Find drive time in milliseconds from relationship developed from
     observations of distance/time for a speed.  */
  float dist_per_sec = (0.117 * float(speed)) - 1.5;// in inches, from data
  float duration = abs(distance) / dist_per_sec;    // needed time in sec
  return duration * 1000.0 ;                        // Return in milliseconds
}


void drive(float distance, byte speed,
	   Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
  byte direction;
  mLeft->setSpeed(speed);  // Set both speeds
  mRight->setSpeed(speed);

  if (distance > 0) {   // Postive distance is forward
    direction = FORWARD;
  }
  else {
    direction = BACKWARD;
  }
  float duration = duration_per_distance(distance, speed);
  /* Now move in the desired directions for that duration */
  mLeft->run(direction);
  mRight->run(direction);
  delay(duration);
  allStop(direction, mLeft, mRight);
  return;
}

void spin(float degrees, byte speed,
	    Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
  /* A spin turns moves the wheels in opposite directions.  Each
     needs to go the same distance, which is determined by the
     degrees parameter, around a circle with a diameter of the
     robot track width.

    The actual distance will be the Circumference * the fraction of the
    circle needed, given by the ratio degrees/360. */
  // float distance = PI * trackWidth * degrees / 360.;
  float distance = PI * trackWidth * degrees / 360.;
  float duration = duration_per_distance(distance, speed);
  mLeft->setSpeed(speed);  // Set both speeds
  mRight->setSpeed(speed);
  if (degrees > 0) {
    /* Positive angle is spin turn to the right; therefore right motor goes
       backwards and left side goes forward */
    mRight->run(BACKWARD);
    mLeft->run(FORWARD);
  }
  else { // Negative or 0 angle, turn to the left
    mRight->run(FORWARD);
    mLeft->run(BACKWARD);
  }
  delay(duration);
  return;
}

void pivot(float degrees, byte speed,
	   Adafruit_DCMotor *mLeft, Adafruit_DCMotor *mRight) {
  /* A pivot turns moves only one wheel, the one opposite the turn
     directions.  The needed distance is determined by the
     degrees parameter, around a circle with a RADIUS of the
     robot track width.
     
     The actual distance will be the Circumference * the fraction of the
     circle needed, given by the ratio degrees/360. */
  float distance = 2.0 * PI * trackWidth * degrees / 360.;
  float duration = duration_per_distance(distance, speed);
  mLeft->setSpeed(speed);  // Set both speeds
  mRight->setSpeed(speed);
  if (degrees > 0) {
    /* Positive angle is spin turn to the right; therefore right motor
       does nothing and left side goes forward */
    mRight->run(RELEASE);
    mLeft->run(FORWARD);
  }
  else { // Negative or 0 angle, turn to the left
    mRight->run(FORWARD);
    mLeft->run(RELEASE);
  }
  delay(duration);
  return;
}

float Distance_inches(int ping) {
  float inches_per_sec = 13582.67;  // Equivilent to 345 m/s
  return ping * inches_per_sec * 1e-6 *0.5;
}

float Distance_cm(int ping) {
  float cm_per_sec = 34500.;        // Equivilent to 345 m/s
  return ping * cm_per_sec * 1e-6 *0.5;
}
