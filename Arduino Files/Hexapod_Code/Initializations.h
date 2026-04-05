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
float stepLength;  
float turnAmount;     // Desired rotation speed amount (negative=left, positive=right)
float liftHeight;

// ============ POSITION TRACKING ============
// Current 3D position (x, y, z) of each leg's foot endpoint in space
Vector3 currentPoints[6];

// ============ GAIT PATTERN MULTIPLIERS ============
// Stride direction multiplier for each leg (flips front/back pairs)
// Legs 0,1,2: stride forward (+1), Legs 3,4,5: stride backward (-1)
float strideMultiplier[6] = {1, -1, 1, -1, 1, -1};

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
  35.739,   // front left 0
  35.739,   // front right 1
  0,   // middle left 2
  180,   // middle right 3
  144.261,   // rear left 4
  144.261   // rear right 5
};

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


// Current state (Propelling/Lifting/Standing/Reset) of each leg
LegState legStates[6];


// ============ STATE VARIABLES ============
// Current main operating state (Initialize, Stand, or Walk)
State currentState = Initialize;

//Previous State
State previousState = Initialize;

