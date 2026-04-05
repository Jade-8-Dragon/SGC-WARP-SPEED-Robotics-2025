// ============ LIBRARY INCLUDES ============
#include <Arduino.h>          // Arduino core functionality (Serial, digitalWrite, etc.)
#include <Servo.h>            // Standard servo control library (NOT USED - PCA9685 PWM driver used instead)
#include <math.h>             // Math functions (sin, cos, sqrt, atan2, acos, PI)
#include <EEPROM.h>           // EEPROM memory for persistent storage of servo offsets
#include <Wire.h>             // I2C communication for PCA9685 PWM driver

// ============ CUSTOM HEADERS ============
#include "vectors.h"          // Vector3 and Vector2 classes for 3D math
#include "Helpers.h"          // Helper functions for printing and debugging
#include "Initializations.h"  // All servo pins, leg structures, and global variables
#include <Adafruit_PWMServoDriver.h>  // PCA9685 I2C PWM servo driver library

// ============ BUTTON STATE DEFINITIONS ============
#define UNPRESSED 0x1  // Button not pressed (active LOW, pin reads HIGH)
#define PRESSED 0x0    // Button pressed (active LOW, pin reads LOW)


//============ SERIAL COMMUNICATION BUFFERS AND TERMINATORS========
const byte numBytes = 16;
char receivedBytes[numBytes];
byte numReceived = 0;
bool newData = false; 

//============ LEG 5 SERVO DECLERATIONS=====
Servo leg5Shoulder;
Servo leg5Elbow;
Servo leg5Wrist; 

// ============ PWM SERVO DRIVER OBJECT ============
// PCA9685 I2C device for controlling 16 PWM servo channels (using 18 for 6 legs × 3 servos each)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// ============ SETUP - RUNS ONCE AT STARTUP ============
void setup(){ 
  // Initialize serial communication at 9600 baud for debugging output
  Serial.begin(9600);
  
  // Initialize the PCA9685 PWM driver on the default I2C address (0x40)
  // This must be done before using any servo control functions
  pwm.begin();
  pwm.setPWMFreq(50);//set to 50 hz for servos as stated in its datasheet

  attach_servos();

  // ============ WALKING STATE VARIABLES ============
  // Movement control values (set by autonomous mode or user input)
  forwardAmount = 0;
  stepLength = 50;  // Desired forward/backward speed
  turnAmount = 0;
  liftHeight = 80;     // Desired rotation amount
  //AssemblySetup();
  stateInitialize();
}

// ============ MAIN LOOP - RUNS CONTINUOUSLY ============
void loop() {
  // Update elapsed time counter (milliseconds since startup)
  elapsedTime = millis();
  readSerial();
  if(newData){
    forwardAmount = receivedBytes[0];
    stepLength    = receivedBytes[1];
    turnAmount    = receivedBytes[2];
    liftHeight    = receivedBytes[3];

    Serial.print("Forward Amount = ");
    Serial.println(forwardAmount, HEX);

    Serial.print("Step Length = ");
    Serial.println(forwardAmount, HEX);

    Serial.print("Turn Amount = ");
    Serial.println(turnAmount, HEX);

    Serial.print("Lift Height = ");
    Serial.println(liftHeight, HEX);

    newData = false;
  }
  printLegPositions();
  WalkState();
  //simpleWalkState();

 //State machine - call appropriate state function based on current state
  // switch(currentState) {
  //   case Initialize:
  //     stateInitialize();
  //     break;
  //   case Stand:
  //     //StandState();  // If this function exists
  //     break;
  //   case Walk:
  //     WalkState();   // Walking function
  //     break;
  // }

}


//============= READ FROM SERIAL ==========
void readSerial() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    byte startMarker = 0x3C;
    byte endMarker = 0x3E;
    byte rb;
   

    while (Serial.available() > 0 && newData == false) {
        rb = Serial.read();

        if (recvInProgress == true) {
            if (rb != endMarker) {
                receivedBytes[ndx] = rb;
                ndx++;
                if (ndx >= numBytes) {
                    ndx = numBytes - 1;
                }
            }
            else {
                receivedBytes[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                numReceived = ndx;  // save the number for use when printing
                ndx = 0;
                newData = true;
            }
        }

        else if (rb == startMarker) {
            recvInProgress = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("This just in (HEX values)... ");
        for (byte n = 0; n < numReceived; n++) {
            Serial.print(receivedBytes[n], HEX);
            Serial.print(' ');
        }
        Serial.println();
        newData = false;
    }
}


// ============ SERVO ATTACHMENT FOR LEG 6 ================
void attach_servos() {
  leg5Shoulder.write(90);
  leg5Elbow.write(180);
  leg5Wrist.write(90);

  leg5Shoulder.attach(leg5.shoulder);
  leg5Elbow.attach(leg5.elbow);
  leg5Wrist.attach(leg5.wrist);
}

// ============ SERVO CONTROL FUNCTIONS ============

// Convert angle (degrees) to PWM pulse width (microseconds)
// Standard servo range: 500µs = 0°, 2500µs = 180°
// Formula: microseconds = 500 + (angle / 180) × (2500 - 500)
int angleToMicroseconds(double angle) {
  double val = 450.0 + (((2550.0 - 450.0) / 180.0) * angle);
  return (int)val;
}

// Send target angles directly to a leg's servos (direct angle mode without IK)
// This bypasses inverse kinematics - useful for testing individual servo ranges
// Parameters:
//   leg: which leg (0-5)
//   targetRot: Vector3 with (shoulder_angle, elbow_angle, wrist_angle) in degrees
void rotateToAngle(int leg, Vector3 targetRot) {
  // Convert target angles to PWM pulse widths
  int shoulderMicroseconds = angleToMicroseconds(targetRot.x);
  int elbowMicroseconds = angleToMicroseconds(targetRot.y);
  int wristMicroseconds = angleToMicroseconds(targetRot.z);

  // Route to the correct leg and send PWM signals via PCA9685
  switch (leg) {
    case 0:
      pwm.writeMicroseconds(leg0.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg0.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg0.wrist,wristMicroseconds);
      break;

    case 1:
      pwm.writeMicroseconds(leg1.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg1.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg1.wrist,wristMicroseconds);
      break;

    case 2:
      pwm.writeMicroseconds(leg2.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg2.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg2.wrist,wristMicroseconds);
      break;

    case 3:
      pwm.writeMicroseconds(leg3.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg3.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg3.wrist,wristMicroseconds);
      break;

    case 4:
      pwm.writeMicroseconds(leg4.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg4.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg4.wrist,wristMicroseconds);
      break;

    case 5:
      leg5Shoulder.writeMicroseconds(shoulderMicroseconds);
      leg5Elbow.writeMicroseconds(elbowMicroseconds);
      leg5Wrist.writeMicroseconds(wristMicroseconds);
      break;

    default:
      break;
  }
  return;
}

// ============ INVERSE KINEMATICS (IK) ============

// Move a leg's foot to a target 3D position using inverse kinematics
// Parameters:
//   leg: which leg to move (0-5)
//   pos: target position (x, y, z) in 3D space relative to body center
// 
// This function:
// 1. Checks if position is reachable (distance <= total leg length)
// 2. Calculates the three servo angles needed to reach that position
// 3. Applies servo offset calibrations
// 4. Sends angles to the PCA9685 PWM driver
void moveToPos(int leg, Vector3 pos) {
  // Update the current position record for this leg
  currentPoints[leg] = pos;

  // Check if target position is physically reachable
  float dis = Vector3(0, 0, 0).distanceTo(pos);
  if (dis > legLength) {
    // Position exceeds maximum leg extension - skip this movement
    print_value("Point impossible to reach", pos, false);
    print_value("Distance", dis, true);
    return;
  }


  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  // float o1 = offsets[leg].x;
  // float o2 = offsets[leg].y;
  // float o3 = offsets[leg].z;

  float d = sqrt(x * x + y * y);
  float r = d - shoulderLength;
  float c = sqrt(z * z + r * r);

  float theta1 = atan2(y,x) * 180.0 / PI;
  //Serial.print("atan2(y, x): "); Serial.println(atan2(y,x));
  float theta2 = atan2(r,-z) * 180.0 / PI + acos(( elbowLength * elbowLength + c * c - wristLength * wristLength) / (2 * elbowLength * c)) * 180.0 / PI;
  float theta3 = 180.0 - acos((elbowLength * elbowLength + wristLength * wristLength - c * c) / (2 * elbowLength * wristLength)) * 180.0 / PI;

  // Serial.print("Theta 1 (shoulder): "); Serial.println(theta1);
  // Serial.print("Theta 2 (elbow): "); Serial.println(theta2);
  // Serial.print("Theta 3 (wrist): "); Serial.println(theta3);
  // Serial.println("====================");
  
  // Store calculated angles as target
  Vector3 targetRot = Vector3(constrain(theta1,0,180), constrain(theta2,0,180), constrain(theta3, 0, 180));

  // Convert angles to PWM pulse widths
  int shoulderMicroseconds = angleToMicroseconds(targetRot.x);
  int elbowMicroseconds = angleToMicroseconds(targetRot.y);
  int wristMicroseconds = angleToMicroseconds(targetRot.z);

  // Send PWM signals to the correct leg
  switch (leg) {
    case 0:
      pwm.writeMicroseconds(leg0.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg0.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg0.wrist,wristMicroseconds);
      break;

    case 1:
      pwm.writeMicroseconds(leg1.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg1.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg1.wrist,wristMicroseconds);
      break;

    case 2:
      pwm.writeMicroseconds(leg2.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg2.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg2.wrist,wristMicroseconds);
      break;

    case 3:
      pwm.writeMicroseconds(leg3.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg3.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg3.wrist,wristMicroseconds);
      break;

    case 4:
      pwm.writeMicroseconds(leg4.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg4.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg4.wrist,wristMicroseconds);
      break;

    case 5:

      leg5Shoulder.writeMicroseconds(shoulderMicroseconds);
      leg5Elbow.writeMicroseconds(elbowMicroseconds);
      leg5Wrist.writeMicroseconds(wristMicroseconds);
      
      // pwm.writeMicroseconds(leg5.shoulder,shoulderMicroseconds);
      // pwm.writeMicroseconds(leg5.elbow,elbowMicroseconds);
      // pwm.writeMicroseconds(leg5.wrist,wristMicroseconds);
      break;

    default:
      break;
  }
  return;
}

void printLegPositions() {
  Serial.println("------------------------");
  Serial.print("FWDAMNT: ");
  Serial.print(forwardAmount);
  Serial.print(" STPLNGTH: ");
  Serial.print(stepLength);
  Serial.print(" TURNAMNT: ");
  Serial.print(turnAmount);
  Serial.print(" LFTHEIGHT: ");
  Serial.print(liftHeight);
  Serial.println();
    Serial.println("LEG  |   X   |   Y   |  Z  | STATE");
    for(int i = 0; i < 6; i++) {
        Serial.print("  ");
        Serial.print(i);
        Serial.print("  | ");
        Serial.print(currentPoints[i].x, 1);
        Serial.print(" | ");
        Serial.print(currentPoints[i].y, 1);
        Serial.print(" | ");
        Serial.print(currentPoints[i].z, 1);
        Serial.print(" | ");
        switch(legStates[i]) {
            case Propelling: Serial.println("PROP"); break;
            case Up:         Serial.println("UP  "); break;
            case Lifting:    Serial.println("LIFT"); break;
            case Down:       Serial.println("DOWN"); break;
            default:         Serial.println("----"); break;
        }
    }
}
