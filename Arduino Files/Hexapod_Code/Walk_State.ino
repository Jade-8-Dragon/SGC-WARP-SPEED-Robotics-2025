Vector3 homeFoot[6];
float legPhaseOffset[6];

float gaitPhase = 0.0f;
float gaitIncrement = 0.0;

void gethomeFootpos(){
  for(int i = 0; i < 6; i++){
    homeFoot[i] = currentPoints[i];
  }
return;
}

void WalkState(){

    
    legPhaseOffset[0] = 0.0;
    legPhaseOffset[3] = 0.0;
    legPhaseOffset[4] = 0.0;

    legPhaseOffset[1] = 0.5;
    legPhaseOffset[2] = 0.5;
    legPhaseOffset[5] = 0.5;

    for (int i = 0; i < 6; i++) {
        float phase = gaitPhase + legPhaseOffset[i];
        if (phase >= 1.0f) phase -= 1.0f;

        Vector3 p = getRectFootPos(i, phase);
        moveToPos(i, p);
    }


    gaitIncrement = forwardAmount / 100.0f * 0.13f;
    //Serial.println(gaitIncrement);
    gaitPhase += gaitIncrement;   // small increment each loop
    //gaitPhase += 0.13; (MAX SPEED)
    if (gaitPhase >= 1.0f) gaitPhase -= 1.0f;
}

Vector3 getRectFootPos(int leg, float phase) {
    Vector3 center = homeFoot[leg];

    // Convert body-forward step into this leg's frame/direction
    Vector2 d = Vector2(stepLength * 0.5, 0.0).bodyToLeg(legAngles[leg]);

    // Serial.print("Leg ");
    // Serial.print(leg);
    // Serial.print(" ");
    // Serial.print("xleg:");
    // Serial.print(d.x * strideMultiplier[leg]);
    // Serial.print("yleg:");
    // Serial.println(d.y);
    
    float turnScale = turnAmount / 100.0f;
    d.x = strideMultiplier[leg] * d.x + (stepLength * turnScale);

    Vector3 backPoint(
        center.x - d.x,
        center.y - d.y,
        center.z
    );

    Vector3 frontPoint(
        center.x + d.x,
        center.y + d.y,
        center.z
    );

    

    Vector3 p;

    // segment boundaries
    const float stanceEnd = 0.50f;
    const float upEnd     = 0.65f;
    const float swingEnd  = 0.85f;
    const float downEnd   = 1.00f;

    if (phase < stanceEnd) {
        // 1) Stance: front -> back on ground
        legStates[leg] = Propelling;
        float s = phase / stanceEnd;

        p.x = frontPoint.x + (backPoint.x - frontPoint.x) * s;
        p.y = frontPoint.y + (backPoint.y - frontPoint.y) * s;
        p.z = center.z;
    }
    else if (phase < upEnd) {
        // 2) Up: raise vertically at back point
        legStates[leg] = Up;
        float s = (phase - stanceEnd) / (upEnd - stanceEnd);

        p.x = backPoint.x;
        p.y = backPoint.y;
        p.z = center.z + liftHeight * s;
    }
    else if (phase < swingEnd) {
        // 3) Swing: back -> front at raised height
        legStates[leg] = Lifting;
        float s = (phase - upEnd) / (swingEnd - upEnd);

        p.x = backPoint.x + (frontPoint.x - backPoint.x) * s;
        p.y = backPoint.y + (frontPoint.y - backPoint.y) * s;
        p.z = center.z + liftHeight;
    }
    else {
        // 4) Down: lower vertically at front point
        legStates[leg] = Down;
        float s = (phase - swingEnd) / (downEnd - swingEnd);

        p.x = frontPoint.x;
        p.y = frontPoint.y;
        p.z = center.z + liftHeight * (1.0f - s);
    }

    return p;
}


void simpleWalkState(){
  // Compute stride vectors for each leg
  Vector2 stepVec[6];

  for(int i = 0; i < 6; i++){
    stepVec[i] = Vector2(60.0, 0.0).bodyToLeg(legAngles[i]);
    Serial.print(i);
    Serial.print(" -> ");
    Serial.print(stepVec[i].x);
    Serial.print(" , ");
    Serial.println(stepVec[i].y);
  }

  // ---------- Tripod A Lift ----------
  moveToPos(0, Vector3(-stepVec[0].x, 150 + stepVec[0].y, -50));
  moveToPos(3, Vector3(0,150, -100));
  moveToPos(4, Vector3(stepVec[4].x, 150 + stepVec[4].y, -50));


  delay(3000);

  // ---------- Tripod A Step Forward ----------
  moveToPos(0, Vector3( stepVec[0].x, 150 + stepVec[0].y, -50));
  moveToPos(3, Vector3(-30, 150, -100));
  moveToPos(4, Vector3( -stepVec[4].x, 150 + stepVec[4].y, -50));

  delay(3000);

  // ---------- Tripod A Down ----------
  moveToPos(0, Vector3( stepVec[0].x, 150 + stepVec[0].y, -150));
  moveToPos(3, Vector3(-30, 150, -150));
  moveToPos(4, Vector3( -stepVec[4].x, 150 + stepVec[4].y, -150));

  delay(3000);

  // ---------- Tripod B Lift ----------
  moveToPos(1, Vector3(stepVec[1].x, 150 + -stepVec[1].y, -50));
  moveToPos(2, Vector3(0, 150, -100));
  moveToPos(5, Vector3(-stepVec[5].x, 150 + -stepVec[5].y, -50));

  delay(3000);

  // ---------- Tripod A Push Back ----------
  moveToPos(0, Vector3(-stepVec[0].x, 150 - stepVec[0].y, -150));
  moveToPos(3, Vector3( 30, 150, -150));
  moveToPos(4, Vector3(stepVec[4].x, 150 - stepVec[4].y, -150));

  delay(3000);

  // ---------- Tripod B Step Forward ----------
  moveToPos(1, Vector3(-stepVec[1].x, 150 + -stepVec[1].y, -50));
  moveToPos(2, Vector3(30, 150, -100));
  moveToPos(5, Vector3(stepVec[5].x, 150 + -stepVec[5].y, -50));

  delay(3000);

  // ---------- Tripod B Down ----------
  moveToPos(1, Vector3(-stepVec[1].x, 150 + -stepVec[1].y, -150));
  moveToPos(2, Vector3(30, 150, -150));
  moveToPos(5, Vector3( stepVec[5].x, 150 + -stepVec[5].y, -150));

  delay(3000);

  // ---------- Tripod A Lift Again ----------
  moveToPos(0, Vector3(-stepVec[0].x, 150 + stepVec[0].y, -50));
  moveToPos(3, Vector3(0, 150, -100));
  moveToPos(4, Vector3(stepVec[4].x, 150 + stepVec[4].y, -50));

  delay(3000);

  // ---------- Tripod B Push Back ----------
  moveToPos(1, Vector3( stepVec[1].x, 150 + -stepVec[1].y, -150));
  moveToPos(2, Vector3(-30, 150, -150));
  moveToPos(5, Vector3(-stepVec[5].x, 150 + -stepVec[5].y, -150));

  delay(3000);

}