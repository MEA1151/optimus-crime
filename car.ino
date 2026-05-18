/*
  ============================================================
  ESP32 Bluetooth Classic — Robotic Car
  and Battery monitoring 
  ============================================================
  Library  : BluetoothSerial (built-in, no installation needed)
  Pair Name: "RoboticCar"
  ============================================================
*/

#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <NewPing.h>
#include <Ultrasonic.h>

// 3bhkem waz here
#define PIN_BATT_ADC       34
#define BATT_MAX_V         7.4f
#define BATT_MIN_V         6.6f
#define BATT_DIVIDER_RATIO 0.3197f
#define BATT_LOW_PCT       20
#define BATT_SEND_INTERVAL 3000

// sayed waz here

// sayed waz here

//abdallah waز here
#define IN1 22 //left 0,1 back
#define IN2 23 //left
#define ENA 18 //left
#define IN3 19 //right
#define IN4 21//right
#define ENB 5 //right


bool waits(unsigned long seconds)
{
    static unsigned long startTime = 0;
    static bool started = false;

    
    if (!started)
    {
        startTime = millis();
        started = true;
    }

    
    if (millis() - startTime >= seconds * 1000UL)
    {
        started = false; 
        return true;     
    }

    return false; 
}

int DEFAULT_SPEED  = 180;
unsigned long lastBattSend = 0;
byte parkingState  = 0;

// ─── Teach-and-Repeat Memory ──────────────────────────────────────
#define MAX_TEACH_STEPS 200
struct TeachStep {
  char command;
  uint16_t duration;  // milliseconds
};
TeachStep teachBuffer[MAX_TEACH_STEPS];
int teachCount = 0;
bool isTeaching = false;
bool isReplaying = false;
int replayStep = 0;
unsigned long replayStepStart = 0;
unsigned long teachStartTime = 0;        // track when teaching starts
unsigned long lastCommandTime = 0;        // track last command time for duration calc

// ─── Swarm Control Variables ──────────────────────────────────────
#define THIS_CAR_ID 1  // Change to 2, 3, etc. for different cars
#define SWARM_DISTANCE_TARGET 20  // cm - maintain this distance from lead car
bool isSwarmFollower = false;
bool isSwarmLeader = false;

// ─── Parking State Variables ──────────────────────────────────────
bool seenObjA    = false;  
bool seenObjB = false; // first parked car detected
bool inGap       = false;   // open gap confirmed after first car
int  nearCount   = 0;       // debounce counter for "close" readings
int  farCount    = 0;       // debounce counter for "far"  readings
unsigned long stateStartTime = 0;

enum Mode { MANUAL, AUTONOMOUS, AUTO_PARKING };
Mode currentMode = MANUAL;

void forward(int v){
  v= constrain(v, 0, 255);
  analogWrite(ENA,v);
  analogWrite(ENB,v);
  digitalWrite(IN1,1);
  digitalWrite(IN2,0);
  digitalWrite(IN3,1);
  digitalWrite(IN4,0);
}

void backward(int v){
  v= constrain(v, 0, 255);
  analogWrite(ENA,v);
  analogWrite(ENB,v);
  digitalWrite(IN1,0);
  digitalWrite(IN2,1);
  digitalWrite(IN3,0);
  digitalWrite(IN4,1);
}

void stop(){
  digitalWrite(IN1,0);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,0);
}

void brake(){
  digitalWrite(IN1,1);
  digitalWrite(IN2,1);
  digitalWrite(IN3,1);
  digitalWrite(IN4,1);
}

void clockwise_rotate(int v){
  v= constrain(v, 0, 255);
  analogWrite(ENA,v);
  analogWrite(ENB,v);
  digitalWrite(IN1,1);
  digitalWrite(IN2,0);
  digitalWrite(IN3,0);
  digitalWrite(IN4,1);
}

void anticlockwise_rotate(int v){
  v= constrain(v, 0, 255);
  analogWrite(ENA,v);
  analogWrite(ENB,v);
  digitalWrite(IN1,0);
  digitalWrite(IN2,1);
  digitalWrite(IN3,1);
  digitalWrite(IN4,0);
}

BluetoothSerial SerialBT;

char command = 'S';

int getBatteryPercent() {
  int raw = analogRead(PIN_BATT_ADC);
  float adcVoltage = (raw / 4095.0f) * 3.3f;
  float battVoltage = adcVoltage / BATT_DIVIDER_RATIO;
  int pct = (int)((battVoltage - BATT_MIN_V) /
                  (BATT_MAX_V - BATT_MIN_V) * 100.0f);
  return constrain(pct, 0, 100);
}

void sendBattery() {
  int pct = getBatteryPercent();
  SerialBT.print("BATT:");
  SerialBT.println(pct);
  if (pct <= BATT_LOW_PCT) {
    SerialBT.println("BATT_LOW");
  }
}

void handleCommand(char cmd) {
  // Record command if teaching (but not stop commands)
  if (isTeaching && cmd != 'E' && cmd != 'T' && teachCount < MAX_TEACH_STEPS) {
 unsigned long now = millis();
    teachBuffer[teachCount].command = cmd;
    // FIXED: Calculate actual duration since last command
    if (lastCommandTime == 0) {
      teachBuffer[teachCount].duration = 0;  // first command
    } else {
      teachBuffer[teachCount].duration = (uint16_t)(now - lastCommandTime);
    }
    lastCommandTime = now;  // update for next command
    teachCount++;
  }

  switch (cmd) {

    case 'M':
      currentMode = MANUAL;
      isTeaching = false;
      isReplaying = false;
      stopCar();
      SerialBT.println("MODE:MANUAL");
      break;

    case 'A':
      currentMode = AUTONOMOUS;
      isTeaching = false;
      isReplaying = false;
      stopCar();
      SerialBT.println("MODE:AUTO");
      break;

    case 'P':
      if (currentMode != AUTO_PARKING) {
        parkingState   = 0;
        seenObjA       = false;
        seenObjB = false;
        inGap          = false;
        nearCount      = 0;
        farCount       = 0;
        stateStartTime = millis();
      }
      currentMode = AUTO_PARKING;
      isTeaching = false;
      isReplaying = false;
      SerialBT.println("MODE:PARKING");
      break;

    // ─── Diagonal Movements ───────────────────────────────────
    case 'G':  // Forward-Left
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        moveForwardLeft(DEFAULT_SPEED);
      }
      break;

    case 'I':  // Forward-Right
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        moveForwardRight(DEFAULT_SPEED);
      }
      break;

    case 'H':  // Backward-Left
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        moveBackwardLeft(DEFAULT_SPEED);
      }
      break;

    case 'J':  // Backward-Right
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        moveBackwardRight(DEFAULT_SPEED);
      }
      break;

    // ─── Cardinal Movements ───────────────────────────────────
    case 'F': 
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        moveForward();
      }
      break;

    case 'B': 
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        moveBackward();
      }
      break;

    case 'L': 
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        turnLeft();
      }
      break;

    case 'R': 
      if (currentMode == MANUAL || (currentMode == MANUAL && isTeaching)) {
        turnRight();
      }
      break;

    case 'S': 
      stopCar(); 
      break;

    // ─── Teach-and-Repeat ─────────────────────────────────────
    case 'T':  // Start Teaching
      if (!isTeaching && !isReplaying) {
        isTeaching = true;
        teachCount = 0;
        currentMode = MANUAL;
        teachStartTime = millis();
        lastCommandTime = 0;  // reset command timer
        SerialBT.println("MODE:TEACHING");
      }
      break;

    case 'E':  // End Teaching
      if (isTeaching) {
        isTeaching = false;
        int recordedSteps = teachCount;
        SerialBT.print("STATUS:RECORDED:");
        SerialBT.println(recordedSteps);
        currentMode = MANUAL;
        SerialBT.println("MODE:MANUAL");
      }
      break;

    case 'X':  // Start Replay
      if (!isReplaying && !isTeaching && teachCount > 0) {
        isReplaying = true;
        replayStep = 0;
        replayStepStart = millis();
        currentMode = MANUAL;
        SerialBT.println("MODE:REPLAYING");
      } else if (teachCount == 0) {
        SerialBT.println("STATUS:NOTHING_RECORDED");
      }
      break;

    // ─── Swarm Control ────────────────────────────────────────
    case 'W':  // Enter Swarm Leader mode (or toggle follower)
      isSwarmLeader = !isSwarmLeader;
      if (isSwarmLeader) {
        SerialBT.println("STATUS:SWARM_LEADER");
      } else {
        SerialBT.println("STATUS:SWARM_FOLLOWER_OFF");
      }
      break;
 default:
      // Speed commands (0-9)
      if (cmd >= '0' && cmd <= '9') {
        DEFAULT_SPEED = map(cmd - '0', 0, 9, 0, 255);
        SerialBT.print("STATUS:SPEED:");
        SerialBT.println(cmd);
      } 
      // Otherwise stop
      else {
        stopCar();
      }
      break;
  }
}

// ── Motor control functions ─────────────────────────────
void moveForward()  { forward(DEFAULT_SPEED); }
void moveBackward() { backward(DEFAULT_SPEED);     }
void turnLeft()     { anticlockwise_rotate(DEFAULT_SPEED);          }
void turnRight()    { clockwise_rotate(DEFAULT_SPEED);            }
void stopCar()      { stop();                              }

// ── Diagonal movement functions ─────────────────────────
void moveForwardRight(int v) {
  v = constrain(v, 0, 255);
  analogWrite(ENA, v);
  analogWrite(ENB, constrain((int)(v * 0.75f), 0, 255));
  digitalWrite(IN1, 1); digitalWrite(IN2, 0);  // right forward
  digitalWrite(IN3, 1); digitalWrite(IN4, 0);  // left forward
}

void moveForwardLeft(int v) {
  v = constrain(v, 0, 255);
  analogWrite(ENB, v);
  analogWrite(ENA, constrain((int)(v * 0.75f), 0, 255));
  digitalWrite(IN1, 1); digitalWrite(IN2, 0);  // right forward
  digitalWrite(IN3, 1); digitalWrite(IN4, 0);  // left forward
}

void moveBackwardRight(int v) {
  v = constrain(v, 0, 255);
  analogWrite(ENB, constrain((int)(v * 0.7f), 0, 255));
  analogWrite(ENA, v);
  digitalWrite(IN1, 0); digitalWrite(IN2, 1);  // right backward
  digitalWrite(IN3, 0); digitalWrite(IN4, 1);  // left backward
}

void moveBackwardLeft(int v) {
  v = constrain(v, 0, 255);
  analogWrite(ENB, v);
  analogWrite(ENA, constrain((int)(v * 0.7f), 0, 255));
  digitalWrite(IN1, 0); digitalWrite(IN2, 1);  // right backward
  digitalWrite(IN3, 0); digitalWrite(IN4, 1);  // left backward
}

/*<MOUSTAFA WAZ HERE>*/
#define servo_pin 15
#define trig_pin  4
#define echo_pin  13

int check_dist = 20;

Servo my_servo;

NewPing ultra(trig_pin, echo_pin, 200);

int distance;
int dist_R;
int dist_L;

void avoiding_obstacles(){
  distance = ultra.ping_cm();
  if(distance > 0 && distance < check_dist){
    stopCar();
    delay(200);
    moveBackward();
    delay(200);
    stopCar();
    delay(500);
    my_servo.write(0);
    delay(500);
    dist_R = ultra.ping_cm();
    my_servo.write(180);
    delay(500);
    dist_L = ultra.ping_cm();
    my_servo.write(90);
    delay(500);
    if(dist_L == 0){
      turnLeft();
      delay(200);
    }
    else if(dist_R == 0){
      turnRight();
      delay(200);
    }
    else if(dist_L >= dist_R){
      turnLeft();
      delay(200);
    }
    else{
      turnRight();
      delay(200);
    }
    {stopCar();
    delay(200);}
  }
  else{
    moveForward();
  }
}
/*<MOUSTAFA WAZ HERE/>*/

/*<Eltyeb & Mohamed>*/
#define REAR_TRIG 32
#define REAR_ECHO 33

Ultrasonic rearSensor(REAR_TRIG, REAR_ECHO);

// ─── Parking Constants ────────────────────────────────────────────
#define PARK_SPEED              210   // slower speed during maneuver
#define PARK_SCAN_ANGLE         0     // servo angle to look right
#define PARK_NEAR_CM            30    // <= this = object present
#define PARK_FAR_CM             23    // >= this = open gap
#define PARK_DEBOUNCE_COUNT     3     // consecutive readings to confirm
#define PARK_REAR_WALL_CM      15  // stop reversing when rear sensor detects wall
#define PARK_REVERSE_ANGLE_MS   1200  // time to reverse at sharp angle into gap
#define PARK_REVERSE_STRAIGHT_MS 1200  // additional straight reverse for final positioning
#define PARK_IDLE_AFTER_PARK_MS 2000   // pause after parking complete

// ─── Sensor Helpers ───────────────────────────────────────────────
long getRearDistance() {
  return rearSensor.read();
}

long scanRight() {
  my_servo.write(PARK_SCAN_ANGLE);
  delay(150);
  long d = ultra.ping_cm();
  if (d == 0) d = 200;
  return d;
}

// NEW: point servo forward and read
long scanForward() {
  my_servo.write(90);
  delay(150);
  long d = ultra.ping_cm();
  if (d == 0) d = 200;
  return d;
}

void centerServo() {
  my_servo.write(90);
}
 // ─── Asymmetric Reverse Helpers ───────────────────────────────────
void reverseRight() {
  // stop right wheel, full left backward → rear swings into right-side gap
 
   // right stopped
   moveBackwardRight(DEFAULT_SPEED);
}

void reverseLeft() {
  // stop left wheel, full right backward → rear swings into left-side gap
   moveBackwardLeft(DEFAULT_SPEED);  // right backward
}
// ─── Auto Park State Machine ──────────────────────────────────────
// Simplified parallel parking: scan→detect gap→enter→stop at rear wall
long rearDist;
long rightDist;
void autoPark() {
   rightDist = ultra.ping_cm();
  if (rightDist == 0) rightDist = 200;
  
   rearDist = getRearDistance();

  // ─ STATE 0: Drive forward and scan for parking slot (object A → gap → object B)
  if (parkingState == 0) {
    my_servo.write(180);  // point servo right
    moveForward();
    // Phase A: Detect first parked car (object A)
    if (!seenObjA) {
      if (rightDist < PARK_NEAR_CM) {
        nearCount++;
        if (nearCount >= PARK_DEBOUNCE_COUNT) {
          seenObjA = true;
          nearCount = 0;
          SerialBT.println("STATUS:OBJ_A_FOUND");
        }
      } else {
        nearCount = 0;
      }
      return;
    }

    // Phase B: Detect open gap (empty space)
    if (!inGap && seenObjA ) {
      if (rightDist > PARK_FAR_CM) {
        farCount++;
        if (farCount >= PARK_DEBOUNCE_COUNT) {
          inGap = true;
          farCount = 0;
          SerialBT.println("STATUS:GAP_FOUND");
        }
      } else {
        farCount = 0;
      }
      return;
    }

    // Phase C: Detect second parked car (object B) → trigger entry into gap
     if (!seenObjB && inGap && seenObjA ) {
      if ((rightDist <= PARK_NEAR_CM)) {
        nearCount++;
        if (nearCount >= PARK_DEBOUNCE_COUNT) {
          delay(400);
          seenObjB = true;
          stop();
          my_servo.write(90);
          parkingState = 1;
          nearCount = 0;
          SerialBT.println("STATUS:OBJ_B_FOUND");
        }
      } else {
        nearCount = 0;
      }
      return;
    }
   
  }

  // diagonal back
  else if (parkingState == 1) {
   SerialBT.println(rearDist);
     if ((rearDist > 10)||(rearDist==0)) {
      reverseLeft(); 
      
       
  }else{ nearCount++;
     if (nearCount >= PARK_DEBOUNCE_COUNT){stop();
    my_servo.write(90);
    nearCount = 0;
    parkingState = 2;}
  
  }}
  else if (parkingState == 2){
if (rightDist > 18) {
       SerialBT.println("ALIGN");
        SerialBT.println(rearDist);
  analogWrite(ENB,DEFAULT_SPEED);
  analogWrite(ENA,0);
  digitalWrite(IN3,1);
  digitalWrite(IN4,0);
  }else{
     nearCount++;
     if (nearCount >= PARK_DEBOUNCE_COUNT){
      nearCount = 0;
    stop();
       parkingState = 3;}
       
        
        
        }

  }

  // ─ STATE 2: Reverse at angle into gap, monitored by rear sensor
  
  // ─ STATE 3: Parked - wait a moment then return to manual mode
  }


void a_p() {
  if (parkingState >= 0 && parkingState <= 3) {
    autoPark();
  }
}

// ─── Teach-and-Repeat Replay Handler ────────────────────────────
// FIXED: Use actual recorded durations instead of fixed 150ms
void handleReplay() {
  if (!isReplaying || teachCount == 0) return;

  unsigned long elapsed = millis() - replayStepStart;

  // Execute current step for recorded duration
  if (replayStep < teachCount) {
    char cmd = teachBuffer[replayStep].command;
    uint16_t stepDuration = teachBuffer[replayStep].duration;
    
    // Get next step's duration (or use 100ms minimum)
    uint16_t nextDuration = 100;
    if (replayStep + 1 < teachCount) {
      nextDuration = teachBuffer[replayStep + 1].duration;
      if (nextDuration == 0) nextDuration = 100;  // fallback for first step
    }

    handleCommand(cmd);  // This will NOT add to buffer since isTeaching = false

    // Move to next step after recorded duration
    if (elapsed >= nextDuration) {
      stopCar();
      replayStep++;
      replayStepStart = millis();
    }
  } else {
    // Replay complete
    stopCar();
    isReplaying = false;
    replayStep = 0;
    currentMode = MANUAL;
    SerialBT.println("STATUS:REPLAY_DONE");
    SerialBT.println("MODE:MANUAL");
  }
}

// ─── Swarm Distance Maintenance (rear sensor feedback) ────────────
void handleSwarmDistance() {
  if (!isSwarmLeader) return;
  
  long rearDist = getRearDistance();
  if (rearDist == 0) return;  // sensor error
  
  // If following car detected (behind), maintain ~20cm distance
  if (rearDist < SWARM_DISTANCE_TARGET - 5) {
// Follower too close, slow down or reverse a bit
   anticlockwise_rotate(constrain(DEFAULT_SPEED / 2, 0, 255));
    delay(50);
  } else if (rearDist > SWARM_DISTANCE_TARGET + 5) {
    // Follower too far, no correction needed in leader mode
    // (followers should handle their own distance via commands)
  }
}
/*<Eltyeb & Mohamed/>*/

void setup() {
  
  Serial.begin(115200);
  SerialBT.begin("RoboticCar");
  Serial.println("Bluetooth ready. Waiting for connection...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopCar();

  my_servo.attach(servo_pin);
  my_servo.write(90);
}

void loop() {
  rightDist = ultra.ping_cm();
 
   rearDist = getRearDistance();
   SerialBT.println(rearDist);
  if (SerialBT.available()) {
    command = SerialBT.read();
    Serial.print("Command: ");
    Serial.println(command);
    handleCommand(command);
  }

  // Execute mode-based logic
  if      (currentMode == AUTONOMOUS)   avoiding_obstacles();
  else if (currentMode == AUTO_PARKING) a_p();

  // Handle teach-and-repeat replay
  if (isReplaying) handleReplay();

  // Handle swarm follower maintenance (if enabled)
  if (isSwarmLeader) handleSwarmDistance();

  // Send battery status periodically
  if (millis() - lastBattSend >= BATT_SEND_INTERVAL) {
    sendBattery();
    lastBattSend = millis();
  }
}