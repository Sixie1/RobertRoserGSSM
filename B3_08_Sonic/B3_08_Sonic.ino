/* B3_08_Sonic.ino

   Drive the TWO-WHEELED Bread Board Bot (BBbot, B^3)
   forward.   When a whisker bump sensor on either side hits something,
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

// Parameters controlling program behavior
// Bump behavior
const byte FORWARD_SPEED = 150;   // Define normal speeds
const byte BACKWARD_SPEED = 100;  // and backup/turn speed
const int  TURN_DURATION = 600;   // Turn length in milliseconds

// Sonic sensor
const float TARGET_DISTANCE_INCHES = 10; 
const int MAX_SONIC_DISTANCE = 500;      // cm, optional, 500 cm is default

// Define 'ports' for motors.
const byte LEFT_MOTOR_PORT = 3;
const byte RIGHT_MOTOR_PORT = 1;
// Create pointers to motor control objects
Adafruit_DCMotor *motorLeft = AFMS.getMotor(LEFT_MOTOR_PORT);
Adafruit_DCMotor *motorRight = AFMS.getMotor(RIGHT_MOTOR_PORT);

/* Define new untrasonic sensor with trigger & echo pins and 
   the maximum distance to be sensed. */
NewPing sonic(SONIC_TRIGGER_PIN, SONIC_ECHO_PIN, MAX_SONIC_DISTANCE); 

void setup(void){
  AFMS.begin();  // create with the default frequency 1.6KHz
  // Turn off all motors to start, just a good habit
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  Serial.begin(9600);  //Begin serial communcation
  
  /*Set up Bump Pins with Arduino internal pullup resistors
    This will make them always high unless a bump switch is hit, 
    which will make a connection to ground and they will read low. */
  pinMode(LEFT_BUMP_PIN,INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN,INPUT_PULLUP);

  //  pinMode(SONIC_TRIGGER_PIN,OUTPUT);
  //  pinMode(SONIC_ECHO_PIN,INPUT);
  delay(2000);  // Two second delay to get the robot placed
}

void loop(){
  // Test some of the sonic library functions:
  int inches = sonic.ping_in();
  Serial.print(inches);
  Serial.print(" inches, cm = ");
  Serial.print(sonic.ping_cm());
  Serial.print(", actual ping time (ms) = ");
  int ping_milli = sonic.ping();
  Serial.print(ping_milli);
  Serial.print(", real inches = ");
  Serial.print(Distance_inches(ping_milli));
  Serial.print(", real cm = ");
  Serial.println(Distance_cm(ping_milli));
  // delay(100); // Just to slow things down
  int l = 0;
  int r = 0;
  /*  Assuming no switches closed initially.  Drive forward: */
  motorLeft->setSpeed(FORWARD_SPEED);
  motorRight->setSpeed(FORWARD_SPEED);
  while(digitalRead(LEFT_BUMP_PIN) && digitalRead(RIGHT_BUMP_PIN)
	&& Distance_inches(sonic.ping()) > TARGET_DISTANCE_INCHES) {
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
  
  /* If you got here, one of the bump switches was closed or B^3 is too
     close to something straight ahead */

  /* First check the LEFT sensor: */
  if(! digitalRead(LEFT_BUMP_PIN) || (Distance_inches(sonic.ping()) && l>=r)) { // the LEFT side switch was bumped
    motorLeft->setSpeed(BACKWARD_SPEED/2); // Slowly back up and turn to right
    motorRight->setSpeed(BACKWARD_SPEED);  
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    delay(TURN_DURATION);                  // for specified duration
    motorLeft->run(RELEASE);               // Then stop power to the motors
    motorRight->run(RELEASE);    // and move to next section of code
    l++;
  }
  /* Then check the right sensor: */
  else if(! digitalRead(RIGHT_BUMP_PIN) || (Distance_inches(sonic.ping()) && r>l)) { // the RIGHT side switch was bumped
    motorLeft->setSpeed(BACKWARD_SPEED); // Slowly back up and turn to left
    motorRight->setSpeed(BACKWARD_SPEED/2);  
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    delay(TURN_DURATION);                 // for specified duration
    motorLeft->run(RELEASE);               // Then stop power to the motors
    motorRight->run(RELEASE);              // and move to next section of code
    r++;
  }
  /* It must have been the sonar sensor */
  else {              
    motorLeft->run(RELEASE);   // So stop power to the motors
    motorRight->run(RELEASE);  // and move to next section of code
  }

  /*That is all!  Now go back to the beginning of the loop and 
     drive straight ahead until somehting is bumped. */
}

/* float Distance_inches(int ping) { */
/*   float inches_per_sec = 13582.67;  // Equivilent to 345 m/s */
/*   return ping * inches_per_sec * 1e-6 *0.5; */
/* } */

/* float Distance_cm(int ping) { */
/*   float cm_per_sec = 34500.;        // Equivilent to 345 m/s */
/*   return ping * cm_per_sec * 1e-6 *0.5; */
/* } */
