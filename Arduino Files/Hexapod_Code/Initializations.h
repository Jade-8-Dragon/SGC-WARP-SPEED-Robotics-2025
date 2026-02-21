// ============ LEG SERVO STRUCTURE ============
// Data structure to store the three servo PWM pins for each leg
struct Leg{
  uint8_t shoulder;  //sholder channel/pin (rotation at body)
  uint8_t elbow;     //elbow servo pin (upper leg joint)
  uint8_t wrist;     //wrist servo pin (lower leg joint)
};

// Macro to quickly define all 6 legs with their servo pins
#define DEFINE_LEG(n, shoulder, elbow, wrist) \
  Leg leg##n = {shoulder, elbow, wrist};

// Create leg objects with servo pin assignments (Arduino pins)
DEFINE_LEG(0, 22, 23, 24)  // Leg 0: shoulder=22, elbow=23, wrist=24
DEFINE_LEG(1, 25, 26, 27)  // Leg 1: shoulder=25, elbow=26, wrist=27
DEFINE_LEG(2, 28, 29, 30)  // Leg 2: shoulder=28, elbow=29, wrist=30
DEFINE_LEG(3, 31, 32, 33)  // Leg 3: shoulder=31, elbow=32, wrist=33
DEFINE_LEG(4, 34, 35, 36)  // Leg 4: shoulder=34, elbow=35, wrist=36
DEFINE_LEG(5, 37, 38, 39)  // Leg 5: shoulder=37, elbow=38, wrist=39

// ============ LEG DIMENSIONS ============
// Physical lengths of each leg segment (in mm)
const float shoulderLength = 46;   //shoulder to first joint
const float elbowLength = 108;  // first joint to second joint
const float wristLength = 200;  //second joint to foot
float legLength = shoulderLength+elbowLength+wristLength;  // Total extended leg length (354mm)

// ============ WALKING STATE VARIABLES ============
// Movement control values (set by autonomous mode or user input)
float forwardAmount;  // Desired forward/backward speed (0-100)
float turnAmount;     // Desired rotation amount (negative=left, positive=right)

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
float strideMultiplier[6] = {1, 1, 1, -1, -1, -1};
// Rotation movement multiplier during turning (controls direction bias)
// Pattern: -1 (left), 0 (center), 1 (right) repeated for pairs
float rotationMultiplier[6] = {-1, 0, 1, -1, 0 , 1};

// ============ BEZIER CURVE CONTROL POINTS ============
// Control points for Bezier curve during propelling phase (leg pushing)
Vector3 ControlPoints[10];
// Control points for Bezier curve during rotation/turning movement
Vector3 RotateControlPoints[10];

// ============ STATE ENUMERATIONS ============
// Main hexapod operating state machine
enum State {
  Initialize,  // Startup/calibration state - attach servos, load settings
  Stand,       // Standing still (not walking, stay in position)
  Walk         // Walking forward/turning based on movement inputs
};

// Individual leg phase within the gait cycle
enum LegState {
  Propelling,  // Leg pushing body forward (foot on ground)
  Lifting,     // Leg lifting up to swing forward (foot in air)
  Standing,    // Leg standing still (not active in current gait)
  Reset        // Leg resetting to initial position before next cycle
};

// Different walking gait patterns (movement styles)
enum Gait {
  TRI,     // Tripod gait: 3 legs on ground, alternating groups (fast, unstable)
  RIPPLE,  // Ripple gait: continuous wave pattern of leg movement (stable, slow)
  WAVE,    // Wave gait: slow stable gait with maximum ground contact (most stable)
  QUAD,    // Quadruped-like gait: opposite pairs move together
  BI,      // Bipedal-style high-energy gait: alternating leg pairs with big hops
  HOP      // Hopping gait: simultaneous leg movement for dynamic motion
};

// ============ TIMING & CYCLE TRACKING ============
// Total system runtime in milliseconds since startup
long elapsedTime = 0;

// (Disabled) Feature flag - when enabled, stride length scales with input speed
//bool dynamicStrideLength = true;

// ============ GAIT MANAGEMENT ============
// Total number of available gait patterns
int totalGaits = 6;
// Array containing all available gaits in order (TRI, RIPPLE, WAVE, QUAD, BI, HOP)
Gait gaits[6] = { TRI, RIPPLE, WAVE, QUAD, BI, HOP };

// ============ SERVO CALIBRATION ============
// Raw angle offsets for calibrating servo home positions (3 servos × 6 legs = 18 values)
// Index: 0-2 (leg 0), 3-5 (leg 1), 6-8 (leg 2), 9-11 (leg 3), 12-14 (leg 4), 15-17 (leg 5)
int8_t rawOffsets[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
// Base offset position (x, y, z) from body center to leg attachment point
Vector3 baseOffset = {90,50,-10};
// Offset position applied to each individual leg (6 legs)
Vector3 offsets[6];

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
// Currently active gait pattern being executed
Gait currentGait = TRI;
// Previous gait pattern (used to detect when gait changes)
Gait previousGait = TRI;
// Index of current gait in the gaits[] array (0-5)
int currentGaitID = 0;

// ============ BODY POSITION & HEIGHT CONTROL ============
// Vertical adjustment applied when standing (allows raising/lowering body)
float standingDistanceAdjustment = 0;

// Base Z-distance from body center to ground (negative = below body)
float distanceFromGroundBase = -60;
// Current height of leg feet from ground (can be adjusted for terrain)
float distanceFromGround = 0;
// Target height to gradually move toward
float targetDistanceFromGround = 0;

// ============ LEG MOVEMENT PARAMETERS ============
// How high legs lift above ground during swing phase (millimeters)
float liftHeight = 130;
// How far down legs land before beginning propulsion phase
float landHeight = 70;
// Extra forward distance to overshoot when placing leg (increases stride)
float strideOvershoot = 10;
// Distance from body center to leg attachment point (millimeters)
float distanceFromCenter = 173;

