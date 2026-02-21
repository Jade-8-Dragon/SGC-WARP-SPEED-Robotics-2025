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

// ============ PWM SERVO DRIVER OBJECT ============
// PCA9685 I2C device for controlling 16 PWM servo channels (using 18 for 6 legs × 3 servos each)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


// ============ SETUP - RUNS ONCE AT STARTUP ============
void setup() {
  // Initialize serial communication at 9600 baud for debugging output
  Serial.begin(9600);
  
  // Initialize the PCA9685 PWM driver on the default I2C address (0x40)
  // This must be done before using any servo control functions
  pwm.begin();
  
  // Load servo offset calibrations from EEPROM memory
  // These offsets fine-tune each servo's home position
  loadRawOffsetsFromEEPROM();

  //These amounts are only here for now for foward walking testing, change at leasure
  forwardAmount = 20;  // Desired forward/backward speed (0-100)
  turnAmount = 0;     // Desired rotation amount (negative=left, positive=right)
}

// ============ MAIN LOOP - RUNS CONTINUOUSLY ============
void loop() {
  // Update elapsed time counter (milliseconds since startup)
  elapsedTime = millis();

  //When Serial communication is introduced, the foward and turn amount will be parsed before this walk state
 // State machine - call appropriate state function based on current state
  switch(currentState) {
    case Initialize:
      stateInitialize();
      break;
    case Stand:
      //StandState();  // If this function exists
      break;
    case Walk:
      WalkState();   // Walking function
      break;
  }

}


// ============ GAIT CYCLE MANAGEMENT ============

// Save the current foot position as the starting point for this leg's gait cycle
// Used when a leg transitions from lifting to propelling phase
void setCycleStartPoints(int leg) {
  cycleStartPoints[leg] = currentPoints[leg];
}

// Save starting positions for ALL legs at once
// Bulk operation for synchronizing cycle start points across entire hexapod
void setCycleStartPoints() {
  for (int i = 0; i < 6; i++) {
    cycleStartPoints[i] = currentPoints[i];
  }
}

// ============ SERVO CONTROL FUNCTIONS ============

// Convert angle (degrees) to PWM pulse width (microseconds)
// Standard servo range: 500µs = 0°, 2500µs = 180°
// Formula: microseconds = 500 + (angle / 180) × (2500 - 500)
int angleToMicroseconds(double angle) {
  double val = 500.0 + (((2500.0 - 500.0) / 180.0) * angle);
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
      pwm.writeMicroseconds(leg5.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg5.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg5.wrist,wristMicroseconds);
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

  // Extract position coordinates
  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  // Get servo offset calibrations for this leg
  float o1 = offsets[leg].x;  // Shoulder/coxa offset
  float o2 = offsets[leg].y;  // Elbow/femur offset
  float o3 = offsets[leg].z;  // Wrist/tibia offset

  // ===== SHOULDER ANGLE (theta1) =====
  // Rotation around vertical axis - determines direction to foot in XY plane
  float theta1 = atan2(y, x) * (180 / PI) + o1;  // base angle + offset

  // ===== ELBOW/WRIST ANGLES (theta2, theta3) - 2-link IK in XZ plane =====
  // Calculate horizontal distance from shoulder to foot (in XY plane)
  float l = sqrt(x * x + y * y);
  
  // Subtract shoulder segment length to get remaining distance for elbow/wrist to cover
  float l1 = l - shoulderLength;
  
  // Calculate total straight-line distance from elbow joint to foot
  float h = sqrt(l1 * l1 + z * z);

  // ===== Using law of cosines for 2-link arm IK =====
  // Calculate angle at elbow joint using law of cosines
  // h² = femur² + tibia² - 2·femur·tibia·cos(angle_at_elbow)
  float phi1 = acos(constrain((pow(h, 2) + pow(elbowLength, 2) - pow(wristLength, 2)) / (2 * h * elbowLength), -1, 1));
  
  // Calculate angle from horizontal to line connecting elbow to foot
  float phi2 = atan2(z, l1);
  
  // Combine angles: femur angle = phi1 + phi2 + offset
  float theta2 = (phi1 + phi2) * 180 / PI + o2;
  
  // Calculate tibia angle using law of cosines (angle at wrist)
  float phi3 = acos(constrain((pow(elbowLength, 2) + pow(wristLength, 2) - pow(h, 2)) / (2 * elbowLength * wristLength), -1, 1));
  
  // Tibia angle = 180° - angle_at_wrist + offset (180° for straight reference)
  float theta3 = 180 - (phi3 * 180 / PI) + o3;

  // Store calculated angles as target
  targetRot = Vector3(theta1, theta2, theta3);

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
      pwm.writeMicroseconds(leg5.shoulder,shoulderMicroseconds);
      pwm.writeMicroseconds(leg5.elbow,elbowMicroseconds);
      pwm.writeMicroseconds(leg5.wrist,wristMicroseconds);
      break;

    default:
      break;
  }
  return;
}

// ============ SERVO OFFSET CALIBRATION SYSTEM ============

// EEPROM memory address where servo offsets are stored (18 bytes for 6 legs × 3 servos)
#define EEPROM_OFFSETS_ADDR 0

// Save all servo offset calibrations to EEPROM for persistent storage
// Offsets are used to fine-tune servo home positions across power cycles
void saveOffsets() {  
  Serial.print("Saving rawOffsets to EEPROM. ");
  // Write each of the 18 offset values (3 per leg × 6 legs)
  for (int i = 0; i < 18; i++) {
    EEPROM.put(EEPROM_OFFSETS_ADDR + i * sizeof(int8_t), rawOffsets[i]);
  }
  Serial.println("Done");
}

// Load servo offset calibrations from EEPROM memory at startup
// These calibrations are used to correct for manufacturing tolerances and servo drift
void loadRawOffsetsFromEEPROM() {
  Serial.println("Filling rawOffsets from EEPROM.");
  // Read all 18 offset values from EEPROM
  for (int i = 0; i < 18; i++) {
    int8_t val;
    EEPROM.get(EEPROM_OFFSETS_ADDR + i * sizeof(int8_t), val);
    rawOffsets[i] = val;
  }
  // Convert raw offsets into Vector3 format for each leg
  updateOffsetVariables();
  // Print calibration values to serial monitor for verification
  printRawOffsets();
}

// Convert raw offset array into Vector3 offsets array
// Raw offsets (int8_t × 18) → Vector3 offsets[6] (each containing x, y, z offsets)
// Also adds baseOffset to position legs at correct distance from body
void updateOffsetVariables() {  
  // Build offset Vector3 for each leg by combining raw offsets with base offset
  for (int i = 0; i < 6; ++i) {
    offsets[i] = Vector3(rawOffsets[i * 3] + baseOffset.x, 
                         rawOffsets[i * 3 + 1] + baseOffset.y, 
                         rawOffsets[i * 3 + 2] + baseOffset.z);
  }
}

// Print servo calibration data to serial monitor for debugging
// Currently disabled with early return (modify to enable output)
void printRawOffsets() {
  return;  // Early return disables all serial output below
  
  Serial.print("Raw Offsets: ");
  for (int i = 0; i < 18; i++) {
    Serial.print(rawOffsets[i]);
    if (i < 17) {
      Serial.print(" ");
    }
  }
  Serial.println();

  Serial.print("Offsets: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(offsets[i].toString());
    if (i < 5) {
      Serial.print(" ");
    }
  }
  Serial.println();

  Serial.print("EEPROM: ");
  for (int i = 0; i < 18; i++) {
    int8_t val;
    EEPROM.get(i * sizeof(int8_t), val);
    Serial.print(val);
    if (i < 17) {
      Serial.print(" ");
    }
  }
  Serial.println();
}
