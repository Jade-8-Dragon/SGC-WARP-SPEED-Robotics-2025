// ============ LEG SERVO STRUCTURE ============
// Data structure to store the three servo PWM pins for each leg
struct Leg{
  int shoulder;  //sholder channel/pin (rotation at body)
  int elbow;     //elbow servo pin (upper leg joint)
  int wrist;     //wrist servo pin (lower leg joint)
};

// Macro to quickly define all 6 legs with their servo pins
#define DEFINE_LEG(n, shoulder, elbow, wrist) \
  Leg leg##n = {shoulder, elbow, wrist};

// Create leg objects with servo pin assignments (Arduino pins)
DEFINE_LEG(0, 10, 11, 12)  // Leg 0: shoulder=22, elbow=23, wrist=24
DEFINE_LEG(1, 2, 1, 0)  // Leg 1: shoulder=25, elbow=26, wrist=27
DEFINE_LEG(2, 7,8, 9)  // Leg 2: shoulder=28, elbow=29, wrist=30
DEFINE_LEG(3, 6, 5, 4)  // Leg 3: shoulder=31, elbow=32, wrist=33
DEFINE_LEG(4, 13, 14, 15)  // Leg 4: shoulder=34, elbow=35, wrist=36
DEFINE_LEG(5, 10, 11, 3)  // Leg 5: shoulder=37, elbow=38, wrist=39

// ============ LEG DIMENSIONS ============
// Physical lengths of each leg segment (in mm)
const float shoulderLength = 50;   //shoulder to first joint
const float elbowLength = 100;  // first joint to second joint
const float wristLength = 203;  //second joint to foot
float legLength = shoulderLength+elbowLength+wristLength;  // Total extended leg length (353mm)

// ============ WALKING STATE VARIABLES ============
// Movement control values (set by autonomous mode or user input)
Vector2 directionVector; //The Direction of movement with x being foward y sideways
float forwardAmount;  // Desired forward/backward speed (0-100)
float turnAmount;     // Desired rotation amount (negative=left, positive=right)
float liftHeight;

// ============ POSITION TRACKING ============
// Current 3D position (x, y, z) of each leg's endpoint
// ============ POSITION TRACKING ============
// Current 3D position (x, y, z) of each leg's foot endpoint in space
Vector3 currentPoints[6];
// Starting position at the beginning of a gait cycle for each leg
Vector3 cycleStartPoints[6];

// ============ ROTATION/ANGLE TRACKING ============
// Current rotation angles (coxa, femur, tibia) for each servo (degrees)
Vector3 currentRot(180, 0, 180);
// Target rotation angles to move servos toward (degrees)
Vector3 targetRot(180, 0, 180);

// ============ GAIT PATTERN MULTIPLIERS ============
// Stride direction multiplier for each leg (flips front/back pairs)
// Legs 0,1,2: stride forward (+1), Legs 3,4,5: stride backward (-1)
float strideMultiplier[6] = {-1, 1, -1, 1, -1, 1};
// Rotation movement multiplier during turning (controls direction bias)
// Pattern: -1 (left), 0 (center), 1 (right) repeated for pairs
float rotationMultiplier[6] = {-1, 1, 0, 0, -1 , 1};

//=========== LEG PIVOT POSITIONS AND ANGLES ==========
Vector2 legPivots[6] = {
  Vector2(-77,  107),
  Vector2(77, 107),
  Vector2(-100,   0),   
  Vector2(100,   0),   
  Vector2(-77, -107),  
  Vector2(77,  -107)   
};

float legAngles[6] = {
   -35.739,   // front left 0
    35.739,   // front right 1
   -90,   // middle left 2
    -270,   // middle right 3
  -125.739,   // rear left 4
   125.739    // rear right 5
};


// ============ BEZIER CURVE CONTROL POINTS ============
// Control points for Bezier curve during propelling phase (leg pushing)
Vector3 ControlPoints[10];
// Control points for Bezier curve during rotation/turning movement
Vector3 RotateControlPoints[10];

// ============ STATE ENUMERATIONS ============
// Main hexapod operating state machine
enum State {
  Assembly,  //used for attaching all of the servos 
  Initialize,  // Startup/calibration state - attach servos, load settings
  Stand,
  SimpleWalk,       // Standing still (not walking, stay in position)
  Walk
};

// Individual leg phase within the gait cycle
enum LegState {
  Lifting,  // Leg up and forward (foot on air)
  Up,     // Leg lifting up
  Down,    // Leg going down
  Propelling,   //Leg pushing body (foot on surface)
  Reset        // Leg resetting to initial position before next cycle
};

// ============ TIMING & CYCLE TRACKING ============
// Total system runtime in milliseconds since startup
long elapsedTime = 0;

// ============ GAIT CYCLE CONTROL ============
// Total number of discrete steps/points in one complete gait cycle
float points = 1000;
// Current progress through gait cycle for each leg (0 to points, then resets)
int cycleProgress[6];
// Current state (Propelling/Lifting/Standing/Reset) of each leg
LegState legStates[6];
// Progress counter for standing state animations when not walking
int standProgress = 0;

// ============ STATE VARIABLES ============
// Current main operating state (Initialize, Stand, or Walk)
State currentState = Initialize;

//Previous State
State previousState = Initialize;

// ============ BODY POSITION & HEIGHT CONTROL ============
// Vertical adjustment applied when standing (allows raising/lowering body)
float standingDistanceAdjustment = 0;

// Base Z-distance from body center to ground (negative = below body)
float distanceFromGroundBase = -60;
// Current height of leg feet from ground (can be adjusted for terrain)
float distanceFromGround = 50;
// Target height to gradually move toward
float targetDistanceFromGround = 0;

