// Gait cycle timing
float tArray[6];                    // Normalized time (0-1) for each leg in current cycle
int ControlPointsAmount = 0;        // Number of Bezier control points for straight (forward) movement
int RotateControlPointsAmount = 0;  // Number of Bezier control points for rotational movement

// ============ GAIT TIMING PARAMETERS ============
float pushFraction = 3.0/6.0;       // Percentage of cycle legs spend pushing (propelling phase)
float speedMultiplier = 0.5;        // Gait-specific speed multiplier (adjusted per gait type)
float strideLengthMultiplier = 1.5; // Gait-specific stride length multiplier
float liftHeightMultiplier = 1.0;   // How high legs lift (multiplies base liftHeight)
float maxStrideLength = 200;        // Maximum stride distance per gait
float maxSpeed = 100;               // Maximum speed limit per gait
float legPlacementAngle = 56;       // Angle for rotating leg placement footprint

// ============ GLOBAL MULTIPLIERS ============
int leftSlider = 50;                  // (Unused) Slider value for future balance control
float globalSpeedMultiplier = 0.55;   // Global scaling for all movement speeds
float globalRotationMultiplier = 0.55; // Global scaling for rotation movement

// ============ WALK STATE - MAIN WALKING CONTROLLER ============
// This function runs continuously when the hexapod is in Walk state
// Handles: gait initialization, leg position updates, cycle progression
void WalkState() {
  // Log state transition to serial monitor
  if(currentState != Walk)Serial.println("Walk State."); 
  
  // Initialize gait setup when entering Walk state or when gait changes
  if (currentState != Walk || previousGait != currentGait) {
    currentState = Walk;

    // Reset all legs to Reset state before starting new gait
    for(int i = 0; i < 6; i++){
      legStates[i] = Reset;
    }   

    // Configure timing and parameters based on selected gait pattern
    switch (currentGait) {
      case TRI:
        // Tripod gait: 2 groups of 3 alternating legs
        // Group 1 (legs 0,2,4) and Group 2 (legs 1,3,5) push alternately
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 2);  // Start half cycle offset
        cycleProgress[2] = 0;
        cycleProgress[3] = (points / 2);
        cycleProgress[4] = 0;
        cycleProgress[5] = (points / 2);

        // 51.7% of cycle on ground (more stable, faster)
        pushFraction = 3.1/6.0;
        speedMultiplier = 1;
        strideLengthMultiplier = 1.2;
        liftHeightMultiplier = 1.1;
        maxStrideLength = 240;
        maxSpeed = 200;
        break;

      case WAVE:
        // Wave gait: continuous 6-leg wave pattern (most stable, slowest)
        // Each leg offset by 1/6 of cycle from the previous one
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 6);     // 1/6 offset
        cycleProgress[2] = (points / 6)*2;   // 2/6 offset
        cycleProgress[3] = (points / 6)*5;   // 5/6 offset (wraps around)
        cycleProgress[4] = (points / 6)*4;   // 4/6 offset
        cycleProgress[5] = (points / 6)*3;   // 3/6 offset

        // 81.7% of cycle on ground (maximum stability, slow movement)
        pushFraction = 4.9/6.0; 

        speedMultiplier = 0.40;
        strideLengthMultiplier = 2;
        liftHeightMultiplier = 1.2;
        maxStrideLength = 150;
        maxSpeed = 160;
        break;

      case RIPPLE:
        // Ripple gait: variant of wave with different timing
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 6)*4;
        cycleProgress[2] = (points / 6)*2;
        cycleProgress[3] = (points / 6)*5;
        cycleProgress[4] = (points / 6);
        cycleProgress[5] = (points / 6)*3;

        // 53.3% of cycle on ground
        pushFraction = 3.2/6.0;

        speedMultiplier = 1;
        strideLengthMultiplier = 1.3;
        liftHeightMultiplier = 1.1;
        maxStrideLength = 220;
        maxSpeed = 200;
        break;

      case BI:
        // Bipedal-style gait: 3 pairs of opposite legs alternate
        // (legs 0,3), (legs 1,4), (legs 2,5) push sequentially
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 3);     // Group 2
        cycleProgress[2] = (points / 3)*2;   // Group 3
        cycleProgress[3] = 0;                // Synchronized with leg 0
        cycleProgress[4] = (points / 3);     // Synchronized with leg 1
        cycleProgress[5] = (points / 3)*2;   // Synchronized with leg 2

        // 35% of cycle on ground (high energy, unstable)
        pushFraction = 2.1/6.0;

        // High speed multiplier for aggressive movement
        speedMultiplier = 4;        
        strideLengthMultiplier = 1;
        liftHeightMultiplier = 1.8;  // Big leg lifts for dramatic motion
        maxStrideLength = 230;
        maxSpeed = 130;
        break;

      case QUAD:
        // Quadruped gait: opposite front/side/back pairs move together
        cycleProgress[0] = 0;
        cycleProgress[1] = (points / 3);
        cycleProgress[2] = (points / 3)*2;
        cycleProgress[3] = 0;
        cycleProgress[4] = (points / 3);
        cycleProgress[5] = (points / 3)*2;

        // 68.3% of cycle on ground (stable)
        pushFraction = 4.1/6.0;

        speedMultiplier = 1;        
        strideLengthMultiplier = 1.2;
        liftHeightMultiplier = 1.1;
        maxStrideLength = 220;
        maxSpeed = 200;
        break;

      case HOP:
        // Hop gait: all legs synchronized - simultaneous push and lift
        cycleProgress[0] = 0;
        cycleProgress[1] = 0;  // All legs at same phase
        cycleProgress[2] = 0;
        cycleProgress[3] = 0;
        cycleProgress[4] = 0;
        cycleProgress[5] = 0;

        // 50% of cycle on ground (all legs synchronized)
        pushFraction = 3/6.0;

        speedMultiplier = 1;
        strideLengthMultiplier = 1.6;   // Long strides
        liftHeightMultiplier = 2.5;     // Maximum lift height
        maxStrideLength = 240;
        maxSpeed = 200;
        break;
    }      
  }

  // ===== MAIN WALKING LOOP - RUNS EVERY ITERATION =====
  
  // Convert cycle progress counters to normalized time (0.0 to 1.0) for each leg
  for(int i = 0; i < 6; i++){
    tArray[i] = (float)cycleProgress[i] / points;    
  }  

  // Autonomous movement - no joystick input
  forwardAmount = 50;  // Set forward speed (0-100 recommended)
  turnAmount = 0;      // Set rotation (negative = left, positive = right)

  // Move all 6 legs to their calculated gait positions based on current cycle progress
  moveToPos(0, getGaitPoint(0, pushFraction));
  moveToPos(1, getGaitPoint(1, pushFraction));
  moveToPos(2, getGaitPoint(2, pushFraction));
  moveToPos(3, getGaitPoint(3, pushFraction));
  moveToPos(4, getGaitPoint(4, pushFraction));
  moveToPos(5, getGaitPoint(5, pushFraction));
  
  // ===== CYCLE PROGRESSION =====
  
  // Calculate how much to advance the gait cycle
  // Based on the larger of forward OR turn input, scaled by gait multipliers
  float progressChangeAmount = (max(abs(forwardAmount),abs(turnAmount))* speedMultiplier)*globalSpeedMultiplier;

  // Clamp progress to gait's maximum speed limit
  progressChangeAmount = constrain(progressChangeAmount,0,maxSpeed*globalSpeedMultiplier);

  // Advance all leg cycle counters and wrap around when reaching cycle end
  for(int i = 0; i < 6; i++){
    cycleProgress[i] += progressChangeAmount;

    // Reset to beginning of cycle when reaching the end
    if(cycleProgress[i] >= points){
      cycleProgress[i] = cycleProgress[i] - points;
    }
  } 
}



// ============ GAIT POINT CALCULATION - CORE LEG POSITIONING ============
// Calculates the target 3D position for a leg based on gait phase and cycle progress
// Parameters:
//   leg: which leg (0-5)
//   pushFraction: percentage of cycle spent in propelling (pushing) phase (0-1)
// Returns: Vector3 position for the leg foot to move toward
Vector3 getGaitPoint(int leg, float pushFraction){  
 
  // ===== STRIDE VECTOR CALCULATION =====
  // Calculate how far to stride based on forward/turn movement inputs
  
  // Create stride vector for rotation movement (turn)
  float rotateStrideLength = turnAmount * globalRotationMultiplier;  
  
  // Create stride vector for forward movement
  Vector2 v = Vector2(forwardAmount, 0);  // Forward movement only

  // Fixed stride length (not dynamic based on input magnitude)
  v.normalize();              // Make unit vector
  v = v * 70;                 // Fixed magnitude for autonomous mode (70mm stride base)

  // Apply gait-specific stride multiplier and constrain length
  v = v * Vector2(1, strideLengthMultiplier);
  v.y = constrain(v.y, -maxStrideLength/2, maxStrideLength/2);
  
  // Apply global speed multiplier
  v = v * globalSpeedMultiplier;

  // Fixed rotation stride magnitude for autonomous mode
  if(rotateStrideLength < 0) rotateStrideLength = -70;
  else if(rotateStrideLength > 0) rotateStrideLength = 70;

  // Calculate weight for blending between forward and rotational movements
  // Prevents division by zero if both inputs are zero
  float weightSum = abs(forwardAmount) + abs(turnAmount);

  // Get normalized cycle progress for this leg (0.0 to 1.0)
  float t = tArray[leg];

  // ===== LEG PHASE DETERMINATION =====
  // Determine if leg is in propelling (pushing) or lifting (swinging) phase
  
  // PROPELLING PHASE - Leg pushes body forward (foot on ground)
  if(t < pushFraction){ 
    // Transition to Propelling state if just entering this phase
    if(legStates[leg] != Propelling)setCycleStartPoints(leg);
    legStates[leg] = Propelling;

    // ===== FORWARD MOVEMENT BEZIER CURVE =====
    // Define 2-point Bezier curve for straight forward push
    ControlPoints[0] = cycleStartPoints[leg];  // Start point
    
    // End point: moved forward by stride amount, rotated around body center
    // strideMultiplier flips direction for front vs back leg pairs
    ControlPoints[1] = Vector3(v.x * strideMultiplier[leg] + distanceFromCenter, 
                               -v.y * strideMultiplier[leg], 
                               distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], 
                                                          Vector2(distanceFromCenter,0));
    ControlPointsAmount = 2;
    
    // Interpolate position along the Bezier curve (0% to 100% through push phase)
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, 
                                                  mapFloat(t, 0, pushFraction, 0, 1));

    // ===== ROTATIONAL MOVEMENT BEZIER CURVE =====
    // Define 3-point Bezier curve for rotational turning movement
    RotateControlPoints[0] = cycleStartPoints[leg];  // Start point
    // Intermediate point: midway between start and rotation center
    RotateControlPoints[1] = { distanceFromCenter + 40, 0, distanceFromGround };
    // End point: moved along turning arc
    RotateControlPoints[2] = { distanceFromCenter, rotateStrideLength, distanceFromGround };
    RotateControlPointsAmount = 3;
    
    // Interpolate position along rotational Bezier curve
    Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, 
                                                mapFloat(t, 0, pushFraction, 0, 1));

    // Blend between forward and rotational movement based on input magnitudes
    return (straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount)) / weightSum;
  }

  // LIFTING PHASE - Leg lifts up and swings forward (foot in air)
  else{
    // Transition to Lifting state if just entering this phase
    if(legStates[leg] != Lifting)setCycleStartPoints(leg);
    legStates[leg] = Lifting;

    // ===== FORWARD MOVEMENT BEZIER CURVE (LIFTING) =====
    // Define 4-point Bezier curve for swinging leg forward
    ControlPoints[0] = cycleStartPoints[leg];  // Starting position
    
    // Lift up point: vertical lift above current position
    ControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, liftHeight * liftHeightMultiplier);
    
    // Landing point: ahead by stride amount, at landing height
    // Use opposite stride direction (moving backward in global frame while leg moves forward relative to body)
    ControlPoints[2] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, 
                               (v.y + strideOvershoot) * strideMultiplier[leg], 
                               distanceFromGround + landHeight).rotate(legPlacementAngle * rotationMultiplier[leg], 
                                                                        Vector2(distanceFromCenter, 0));
    
    // Final position: on ground at new foot position (end of landing)
    ControlPoints[3] = Vector3(-v.x * strideMultiplier[leg] + distanceFromCenter, 
                               v.y * strideMultiplier[leg], 
                               distanceFromGround).rotate(legPlacementAngle * rotationMultiplier[leg], 
                                                          Vector2(distanceFromCenter, 0));
    ControlPointsAmount = 4;
    
    // Interpolate position (0% to 100% through lift/swing phase, inverted progress)
    Vector3 straightPoint = GetPointOnBezierCurve(ControlPoints, ControlPointsAmount, 
                                                  mapFloat(t, pushFraction, 1, 0, 1));

    // ===== ROTATIONAL MOVEMENT BEZIER CURVE (LIFTING) =====
    // Define 5-point Bezier curve for rotational swing movement
    RotateControlPoints[0] = cycleStartPoints[leg];  // Start point
    
    // Lift up point (same as straight lift)
    RotateControlPoints[1] = cycleStartPoints[leg] + Vector3(0, 0, liftHeight * liftHeightMultiplier);
    
    // Apex point: high point in the swing arc
    RotateControlPoints[2] = { distanceFromCenter + 40, 0, distanceFromGround + liftHeight * liftHeightMultiplier};
    
    // Landing point: swung around rotation center with overshoot
    RotateControlPoints[3] = { distanceFromCenter, -(rotateStrideLength + strideOvershoot), 
                               distanceFromGround + landHeight};
    
    // Final position: on ground at new foot position after rotation
    RotateControlPoints[4] = { distanceFromCenter, -rotateStrideLength, distanceFromGround};
    RotateControlPointsAmount = 5;
    
    // Interpolate position along rotational swing curve
    Vector3 rotatePoint = GetPointOnBezierCurve(RotateControlPoints, RotateControlPointsAmount, 
                                                mapFloat(t, pushFraction, 1, 0, 1));

    // Blend between forward and rotational movement based on input magnitudes
    return (straightPoint*abs(forwardAmount) + rotatePoint*abs(turnAmount)) / weightSum;
  }  
}








