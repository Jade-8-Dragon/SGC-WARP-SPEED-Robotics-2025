#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include "../../vectors.h"
#include "../../Helpers.h"
#include "../../leg_survos_config.h"
#include "../../bezier.cpp"

// Pin definitions for leg 1
#define SHOULDER_PIN 5
#define ELBOW_PIN 6
#define WRIST_PIN 7

enum LegState {
  Propelling,
  Lifting,
  Standing,
  Reset
};

// Single leg variables
struct Leg {
    Servo wrist;
    Servo shoulder;
    Servo elbow;
};

Leg leg1;
Vector3 currentPoint;
Vector3 cycleStartPoint;
Vector3 offset;
LegState legState;

// Gait parameters
float points = 1000;
int cycleProgress = 0;
float forwardAmount = 50;
float turnAmount = 0;
float t = 0;

int ControlPointsAmount = 0;
int RotateControlPointsAmount = 0;

// Gait configuration
float pushFraction = 3.1/6.0;
float speedMultiplier = 1;
float strideLengthMultiplier = 1.2;
float liftHeightMultiplier = 1.1;
float maxStrideLength = 240;
float maxSpeed = 200;

// Leg parameters
float distanceFromGroundBase = -60;
float distanceFromGround = 0;
float liftHeight = 130;
float landHeight = 70;
float strideOvershoot = 10;
float distanceFromCenter = 173;
float legPlacementAngle = 56;
float strideMultiplier = 1.0;
float rotationMultiplier = 1.0;

// Global multipliers
float globalSpeedMultiplier = 0.55;
float globalRotationMultiplier = 0.55;

bool dynamicStrideLength = true;

// Bezier curve arrays for single leg
Vector3 ControlPoints[4];2
Vector3 RotateControlPoints[5];

// Target rotation
Vector3 targetRot;

long elapsedTime = 0;
long loopStartTime = 0;

void setup() {
  Serial.begin(9600);
  leg1.shoulder.attach(SHOULDER_PIN);
  leg1.elbow.attach(ELBOW_PIN);
  leg1.wrist.attach(WRIST_PIN);
  
  // Initialize starting position
  currentPoint = Vector3(distanceFromCenter, 0, distanceFromGround);
  cycleStartPoint = currentPoint;
  legState = Standing;
}

void loop() {
  elapsedTime = millis() - loopStartTime;
  loopStartTime = millis();
  
  // Update gait progress
  float progressChangeAmount = (max(abs(forwardAmount), abs(turnAmount)) * speedMultiplier) * globalSpeedMultiplier;
  progressChangeAmount = constrain(progressChangeAmount, 0, maxSpeed * globalSpeedMultiplier);
  
  cycleProgress += progressChangeAmount;
  if (cycleProgress >= points) {
    cycleProgress = cycleProgress - points;
  }
  
  t = (float)cycleProgress / points;
  
  // Move leg to calculated position
  moveToPos(getGaitPoint(pushFraction));
  
  delay(10);
}


void setCycleStartPoint() {
  cycleStartPoint = currentPoint;
}

int angleToMicroseconds(double angle) {
  double val = 500.0 + (((2500.0 - 500.0) / 180.0) * angle);
  return (int)val;
}

void moveToPos(Vector3 pos) {
  currentPoint = pos;

  float dis = Vector3(0, 0, 0).distanceTo(pos);
  if (dis > legLength) {
    Serial.print("Point impossible to reach: ");
    Serial.println(dis);
    return;
  }

  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  // Apply offsets (set to 0 for now)
  float o1 = 0; // offset.x;
  float o2 = 0; // offset.y;
  float o3 = 0; // offset.z;

  float theta1 = atan2(y, x) * (180 / PI) + o1;
  float l = sqrt(x * x + y * y);
  float l1 = l - a1;
  float h = sqrt(l1 * l1 + z * z);

  float phi1 = acos(constrain((pow(h, 2) + pow(a2, 2) - pow(a3, 2)) / (2 * h * a2), -1, 1));
  float phi2 = atan2(z, l1);
  float theta2 = (phi1 + phi2) * 180 / PI + o2;
  float phi3 = acos(constrain((pow(a2, 2) + pow(a3, 2) - pow(h, 2)) / (2 * a2 * a3), -1, 1));
  float theta3 = 180 - (phi3 * 180 / PI) + o3;

  targetRot = Vector3(theta1, theta2, theta3);

  int shoulderMicroseconds = angleToMicroseconds(targetRot.x);
  int elbowMicroseconds = angleToMicroseconds(targetRot.y);
  int wristMicroseconds = angleToMicroseconds(targetRot.z);

  leg1.shoulder.writeMicroseconds(shoulderMicroseconds);
  leg1.elbow.writeMicroseconds(elbowMicroseconds);
  leg1.wrist.writeMicroseconds(wristMicroseconds);
}

Vector3 getGaitPoint(float pushFraction) {
  float rotateStrideLength = turnAmount * globalRotationMultiplier;
  Vector2 v = Vector2(forwardAmount, 0);

  if (!dynamicStrideLength) {
    v.normalize();
    v = v * 70;
  }

  v = v * Vector2(1, strideLengthMultiplier);
  v.y = constrain(v.y, -maxStrideLength / 2, maxStrideLength / 2);
  v = v * globalSpeedMultiplier;

  if (!dynamicStrideLength) {
    if (rotateStrideLength < 0) rotateStrideLength = -70;
    else rotateStrideLength = 70;
  }

  float weightSum = abs(forwardAmount) + abs(turnAmount);
  if (weightSum < 0.01) weightSum = 1.0;

  // Propelling phase
  if (t < pushFraction) {
    if (legState != Propelling) setCycleStartPoint();
    legState = Propelling;

    // Bezier path for forward movement
    ControlPoints[0] = cycleStartPoint;
    ControlPoints[1] = Vector3(v.x * strideMultiplier + distanceFromCenter, -v.y * strideMultiplier, distanceFromGround).rotate(legPlacementAngle * rotationMultiplier, Vector2(distanceFromCenter, 0));
    ControlPointsAmount = 2;
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t, 0, pushFraction, 0, 1));

    // Bezier path for rotation
    RotateControlPoints[0] = cycleStartPoint;
    RotateControlPoints[1] = Vector3(distanceFromCenter + 40, 0, distanceFromGround);
    RotateControlPoints[2] = Vector3(distanceFromCenter, rotateStrideLength, distanceFromGround);
    RotateControlPointsAmount = 3;
    Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t, 0, pushFraction, 0, 1));

    return (straightPoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / weightSum;
  }
  // Lifting phase
  else {
    if (legState != Lifting) setCycleStartPoint();
    legState = Lifting;

    ControlPoints[0] = cycleStartPoint;
    ControlPoints[1] = cycleStartPoint + Vector3(0, 0, liftHeight * liftHeightMultiplier);
    ControlPoints[2] = Vector3(-v.x * strideMultiplier + distanceFromCenter, (v.y + strideOvershoot) * strideMultiplier, distanceFromGround + landHeight).rotate(legPlacementAngle * rotationMultiplier, Vector2(distanceFromCenter, 0));
    ControlPoints[3] = Vector3(-v.x * strideMultiplier + distanceFromCenter, v.y * strideMultiplier, distanceFromGround).rotate(legPlacementAngle * rotationMultiplier, Vector2(distanceFromCenter, 0));
    ControlPointsAmount = 4;
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, mapFloat(t, pushFraction, 1, 0, 1));

    RotateControlPoints[0] = cycleStartPoint;
    RotateControlPoints[1] = cycleStartPoint + Vector3(0, 0, liftHeight * liftHeightMultiplier);
    RotateControlPoints[2] = Vector3(distanceFromCenter + 40, 0, distanceFromGround + liftHeight * liftHeightMultiplier);
    RotateControlPoints[3] = Vector3(distanceFromCenter, -(rotateStrideLength + strideOvershoot), distanceFromGround + landHeight);
    RotateControlPoints[4] = Vector3(distanceFromCenter, -rotateStrideLength, distanceFromGround);
    RotateControlPointsAmount = 5;
    Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, mapFloat(t, pushFraction, 1, 0, 1));

    return (straightPoint * abs(forwardAmount) + rotatePoint * abs(turnAmount)) / weightSum;
  }
}
