
float  tArray[6];
int ControlPointsAmount = 0;
int RotateControlPointsAmount = 0;
float pushFraction = 3.1/6.0;
float speedMultiplier = 0.5;
float strideLengthMultiplier = 0.5;
float liftHeightMultiplier = 1.3;
float maxStrideLength = 200;
float maxSpeed = 100;
float legPlacementAngle = 54;
// int leftSlider = 50;
float globalSpeedMultiplier = 0.55;
float globalRotationMultiplier = 0.55;

void WalkState() {

  if(currentState != Walk){
    Serial.println("Walk State.");
    currentState = Walk;

    for(int i = 0; i < 6; i++){
      legStates[i] = Reset;
    }

    // Tripod groups
    cycleProgress[0] = 0;
    cycleProgress[3] = 0;
    cycleProgress[4] = 0;

    cycleProgress[1] = points/2;
    cycleProgress[2] = points/2;
    cycleProgress[5] = points/2;

    pushFraction = 0.5;     // half stance
    speedMultiplier = 1.0;
    strideLengthMultiplier = 1.2;
    liftHeightMultiplier = 1.1;
    maxStrideLength = 200;
    maxSpeed = 200;
  }

  for(int i = 0; i < 6; i++){
    tArray[i] = (float)cycleProgress[i] / points;
  }

  for(int i = 0; i < 6; i++){
    moveToPos(i, getTripodGaitPoint(i));
  }

  float progressChangeAmount =
      max(abs(forwardAmount), abs(turnAmount)) * speedMultiplier * globalSpeedMultiplier;

  progressChangeAmount = constrain(progressChangeAmount,0,maxSpeed);

  for(int i = 0; i < 6; i++){
    cycleProgress[i] += progressChangeAmount;

    if(cycleProgress[i] >= points){
      cycleProgress[i] -= points;
    }
  }
}



Vector3 getTripodGaitPoint(int leg){

  float t = tArray[leg];
  Vector3 p;
  Vector2 d;

  float stepLength = 15;

  //Leg Pushing the Body Forward (leg moves backword on ground)
  if(0.0 <= t && t < 0.25){
    if(legStates[leg] != Propelling)
      setCycleStartPoints(leg);
    legStates[leg] = Propelling;

    d = Vector2(stepLength,0).bodyToLeg(legAngles[leg]);
    float dx = d.x;
    float dy = d.y;
    
    float s = t / 0.25 ;

    float x = cycleStartPoints[leg].x - (dx * s);
    float y = cycleStartPoints[leg].y - (dy * s);
    float z = cycleStartPoints[leg].z;

    p = Vector3(x,y,z);
  }

  //Leg Going Vertically Up
  else if(0.25 < t && t < 0.5){
    if(legStates[leg] != Up)
      setCycleStartPoints(leg);
    legStates[leg] = Up;

    float s = (t - 0.25) / (1.0 - 0.5);

    float x = cycleStartPoints[leg].x;
    float y = cycleStartPoints[leg].y;
    float z = cycleStartPoints[leg].z + (liftHeight * s);

    p = Vector3(x,y,z);
  }

  // Lifting so leg is moving in the air
  else if(0.5 <= t && t < 0.75){
    if(legStates[leg] != Lifting)
      setCycleStartPoints(leg);
    legStates[leg] = Lifting;
    
    d = Vector2(stepLength,0).bodyToLeg(legAngles[leg]);
    float dx = d.x;
    float dy = d.y;

    float s = t / pushFraction;   // normalize 0→1
    float x = cycleStartPoints[leg].x + (dx * s);
    float y = cycleStartPoints[leg].y + (dy * s);
    float z = liftHeight;
    
    p = Vector3(x,y,z);
  }

  // leg is going down
  else if(0.75 <= t && t < 1){
    if(legStates[leg] != Down)
      setCycleStartPoints(leg);
    legStates[leg] = Down;

    float s = t / pushFraction;   // normalize 0→1
    float x = cycleStartPoints[leg].x;
    float y = cycleStartPoints[leg].y;
    float z = cycleStartPoints[leg].z - (liftHeight * s);
    
    p = Vector3(x,y,z);
  }
  


  p = p.bodyToLeg(legAngles[leg]);
  Serial.print("x:");
  Serial.print(p.x);
  Serial.print(" y:");
  Serial.print(p.y);
  Serial.print(" z:");
  Serial.println(p.z);

  return p;

}

void simpleWalkState(){
  // Compute stride vectors for each leg
  Vector2 stepVec[6];

  for(int i = 0; i < 6; i++){
    stepVec[i] = Vector2(40.0, 0.0).bodyToLeg(legAngles[i]);
    Serial.print(i);
    Serial.print(" -> ");
    Serial.print(stepVec[i].x);
    Serial.print(" , ");
    Serial.println(stepVec[i].y);
  }

  // ---------- Tripod A Lift ----------
  moveToPos(0, Vector3(-stepVec[0].x, 150 + stepVec[0].y, -100));
  moveToPos(3, Vector3(0,150, -100));
  moveToPos(4, Vector3(stepVec[4].x, 150 + stepVec[4].y, -100));


  delay(3000);

  // ---------- Tripod A Step Forward ----------
  moveToPos(0, Vector3( stepVec[0].x, 150 + stepVec[0].y, -100));
  moveToPos(3, Vector3(-30, 150, -100));
  moveToPos(4, Vector3( -stepVec[4].x, 150 + stepVec[4].y, -100));

  delay(3000);

  // ---------- Tripod A Down ----------
  moveToPos(0, Vector3( stepVec[0].x, 150 + stepVec[0].y, -150));
  moveToPos(3, Vector3(-30, 150, -150));
  moveToPos(4, Vector3( -stepVec[4].x, 150 + stepVec[4].y, -150));

  delay(3000);

  // ---------- Tripod B Lift ----------
  moveToPos(1, Vector3(stepVec[1].x, 150 + -stepVec[1].y, -100));
  moveToPos(2, Vector3(0, 150, -100));
  moveToPos(5, Vector3(-stepVec[5].x, 150 + -stepVec[5].y, -100));

  delay(3000);

  // ---------- Tripod A Push Back ----------
  moveToPos(0, Vector3(-stepVec[0].x, 150 - stepVec[0].y, -150));
  moveToPos(3, Vector3( 30, 150, -150));
  moveToPos(4, Vector3(stepVec[4].x, 150 - stepVec[4].y, -150));

  delay(3000);

  // ---------- Tripod B Step Forward ----------
  moveToPos(1, Vector3(-stepVec[1].x, 150 + -stepVec[1].y, -100));
  moveToPos(2, Vector3(30, 150, -100));
  moveToPos(5, Vector3(stepVec[5].x, 150 + -stepVec[5].y, -100));

  delay(3000);

  // ---------- Tripod B Down ----------
  moveToPos(1, Vector3(-stepVec[1].x, 150 + -stepVec[1].y, -150));
  moveToPos(2, Vector3(30, 150, -150));
  moveToPos(5, Vector3( stepVec[5].x, 150 + -stepVec[5].y, -150));

  delay(3000);

  // ---------- Tripod A Lift Again ----------
  moveToPos(0, Vector3(-stepVec[0].x, 150 + stepVec[0].y, -100));
  moveToPos(3, Vector3(0, 150, -100));
  moveToPos(4, Vector3(stepVec[4].x, 150 + stepVec[4].y, -100));

  delay(3000);

  // ---------- Tripod B Push Back ----------
  moveToPos(1, Vector3( stepVec[1].x, 150 + -stepVec[1].y, -150));
  moveToPos(2, Vector3(-30, 150, -150));
  moveToPos(5, Vector3(-stepVec[5].x, 150 + -stepVec[5].y, -150));

  delay(3000);

}