/*
  ============================================================
  ESP32 Bluetooth Classic — Robotic Car
  and Battery monitoring 
  ============================================================
  Library  : BluetoothSerial (built-in, no installation needed)
  Pair Name: "RoboticCar"
  ============================================================
*/
#include "Ultrasonic.h"
// 3bhkem waz here
#include "BluetoothSerial.h"
#define PIN_BATT_ADC       34       // GPIO34 (ADC-only pin)
#define BATT_MAX_V         7.4f     // Fully charged 2S LiPo
#define BATT_MIN_V         6.6f     // Minimum safe voltage
#define BATT_DIVIDER_RATIO 0.3197f  // R2 / (R1 + R2)
#define BATT_LOW_PCT       20       // Low battery warning threshold
#define BATT_SEND_INTERVAL 3000     // Send battery data every 3 seconds

// sayed waz here

//abdallah waز here
#define IN1 22
#define IN2 23
#define ENA 18  
#define IN3 19
#define IN4 21
#define ENB 5   

int DEFAULT_SPEED = 180;
unsigned long lastBattSend = 0;

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
  digitalWrite(IN1,0);  // FIX 2: was 1 — left motor now goes backward
  digitalWrite(IN2,1);  // FIX 2: was 0
  digitalWrite(IN3,1);
  digitalWrite(IN4,0);
}

BluetoothSerial SerialBT;

char command = 'S';   // Default command = Stop

int getBatteryPercent() {

  // Step 1: Read raw ADC value (0–4095 on ESP32)
  int raw = analogRead(PIN_BATT_ADC);

  // Step 2: Convert ADC reading to voltage
  float adcVoltage = (raw / 4095.0f) * 3.3f;

  // Step 3: Reverse voltage divider equation
  float battVoltage = adcVoltage / BATT_DIVIDER_RATIO;

  // Step 4: Convert voltage to percentage
  int pct = (int)((battVoltage - BATT_MIN_V) /
                  (BATT_MAX_V - BATT_MIN_V) * 100.0f);

  // Step 5: Clamp result between 0% and 100%
  return constrain(pct, 0, 100);
}

void sendBattery() {

  int pct = getBatteryPercent();

  // Send battery percentage
  SerialBT.print("BATT:");
  SerialBT.println(pct);

  // Send low battery warning
  if (pct <= BATT_LOW_PCT) {
    SerialBT.println("BATT_LOW");
  }
}

void handleCommand(char cmd) {

  switch (cmd) {

    case 'F':
      moveForward();
      break;

    case 'B':
      moveBackward();
      break;

    case 'L':
      turnLeft();
      break;

    case 'R':
      turnRight();
      break;

    case 'S':
      stopCar();
      break;

    case 'P':
      autoPark();
      break;

    default:
      stopCar();   // Safe fallback
      break;
  }
}

// ── Motor control functions ─────────────────────────────
// FIX 3: filled in empty functions using already-defined primitives above
void moveForward()  { forward(DEFAULT_SPEED);              }
void moveBackward() { backward(DEFAULT_SPEED);             }
void turnLeft()     { anticlockwise_rotate(DEFAULT_SPEED); }
void turnRight()    { clockwise_rotate(DEFAULT_SPEED);     }
void stopCar()      { stop();                              }

/*<MOUSTAFA WAZ HERE>*/
#include <Servo.h>
#include <NewPing.h>

int servo_pin = 25;    
int trig_pin  = 14;    
int echo_pin  = 15;

int check_dist = 20;


Servo my_servo;

NewPing ultra (trig_pin,echo_pin,200);


int distance;
int dist_R;
int dist_L;


void avoiding_obstacles(){

  distance = ultra.ping_cm();

  if(distance > 0 && distance < check_dist){
    //stop motor
   stopCar();
    delay(200);

    //Reverse motor
    moveBackward();
    delay(200);

    //stop motor
   stopCar();
    delay(500);

    //Rotate the servo
    my_servo.write(0);
    delay(500);

    dist_R = ultra.ping_cm();

    my_servo.write(180);
    delay(500);

    dist_L = ultra.ping_cm();

    my_servo.write(90);
    delay(500);

    if(dist_L == 0){
      //motor move to left
     turnLeft();
      delay(200);
    }
    else if(dist_R == 0){
      //motor move to right
      turnRight();
      delay(200);
    }
    else if(dist_L >= dist_R){  //comparing between right $ left
      turnLeft();
      delay(200);
    }
    else{              //move to right
      turnRight();
      delay(200);
    }  // STOP THE MOTOR FOR A SHORT TIME
    {stopCar();
    delay(200); }
    
  }
  else{  //MOVE FORWARD
    moveForward();

  }
} 

/*<MOUSTAFA WAZ HERE/>*/

/*<Eltyeb & Mohamed>*/
//
// =====================================================
//                  ULTRASONIC PINS
// =====================================================
//

// Front Ultrasonic
#define FRONT_TRIG 26
#define FRONT_ECHO 27

// Rear Ultrasonic
#define REAR_TRIG 32
#define REAR_ECHO 33

Ultrasonic frontSensor(FRONT_TRIG, FRONT_ECHO);
Ultrasonic rearSensor(REAR_TRIG, REAR_ECHO);

//
// =====================================================
//                     SERVOS
// =====================================================
//

Servo frontServo;
Servo rearServo;

#define FRONT_SERVO_PIN 13
#define REAR_SERVO_PIN 12

//
// =====================================================
//                 PARKING VARIABLES
// =====================================================
//

byte parkingState = 0;

//
//                 ULTRASONIC FUNCTIONS
// =====================================================
//

// Scan right side using front servo
long scanRightSide()
{
  frontServo.write(180);

  delay(300);

  long distance = frontSensor.Ranging(CM);

  return distance;
}

// Front distance

long getFrontDistance()
{
  frontServo.write(90);

  delay(200);

  long distance = frontSensor.Ranging(CM);

  return distance;
}

// Rear distance

long getRearDistance()
{
  rearServo.write(90);

  delay(200);

  long distance = rearSensor.Ranging(CM);

  return distance;
}

//
// =====================================================
//                    AUTO PARKING
// =====================================================
//

void autoPark()
{
  //
  // STATE 0
  // Search for parking slot
  //

  if (parkingState == 0)
  {
    forward(DEFAULT_SPEED);

    long rightDistance = scanRightSide();

    //
    // Parking slot found
    //

    if (rightDistance > 35)
    {
      stopCar();

      delay(1000);

      parkingState = 1;
    }
  }

  //
  // STATE 1
  // Turn right
  //

  else if (parkingState == 1)
  {
    turnRight();

    delay(700);

    stopCar();

    delay(500);

    parkingState = 2;
  }

  //
  // STATE 2
  // Reverse into parking slot
  //

  else if (parkingState == 2)
  {
    backward(DEFAULT_SPEED - 30);

    while (1)
    {
      long rearDistance = getRearDistance();

      //
      // Stop near obstacle
      //

      if (rearDistance <= 12 && rearDistance > 0)
      {
        stopCar();

        parkingState = 3;

        break;
      }
    }
  }

  //
  // STATE 3
  // Align vehicle
  //

  else if (parkingState == 3)
  {
    turnLeft();

    delay(600);

    stopCar();

    parkingState = 4;
  }

  //
  // STATE 4
  // Parking complete
  //

  else if (parkingState == 4)
  {
    stopCar();
  }
}

//
void a_p(){

   if (parkingState > 0 && parkingState < 4)
  {
    autoPark();
  }
}
/*<Eltyeb & Mohamed/>*/
void setup() {

  Serial.begin(115200);

  SerialBT.begin("RoboticCar");

  Serial.println("Bluetooth ready. Waiting for connection...");

  pinMode(IN1, OUTPUT);  // FIX 4: was 1, changed to OUTPUT
  pinMode(IN2, OUTPUT);  // FIX 4
  pinMode(IN3, OUTPUT);  // FIX 4
  pinMode(IN4, OUTPUT);  // FIX 4
  pinMode(ENA, OUTPUT);  // FIX 4
  pinMode(ENB, OUTPUT);  // FIX 4
 //
  // Attach Servos
  //

  frontServo.attach(FRONT_SERVO_PIN);

  rearServo.attach(REAR_SERVO_PIN);

  //
  // Initial Servo Position
  //

  frontServo.write(90);

  rearServo.write(90);

  //
  // Stop Motors
  //

  stopCar();

  my_servo.attach(servo_pin);
  my_servo.write(90);
}

void loop() {

  if (SerialBT.available()) {

    command = SerialBT.read();

    Serial.print("Command: ");
    Serial.println(command);

    handleCommand(command);
  }

  // Send battery data periodically
  if (millis() - lastBattSend >= BATT_SEND_INTERVAL) {

    sendBattery();

    lastBattSend = millis();
  }
}
