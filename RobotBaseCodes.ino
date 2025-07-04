#include <Servo.h>           //Need for Servo pulse output
#include <SoftwareSerial.h>  // For wireless communication
#include <Arduino.h>
#include "SensorFilter.h"
#include "vector.h"

// Define Kalman filter instance (Q, R, initial estimate, initial error covariance)
SensorFilter lR(0.2, 0.5, 0, 1, 4246.1, -0.913, A6);
SensorFilter lL(0.2, 0.5, 0, 1, 1432.6, -0.53, A7);
SensorFilter sR(0.2, 0.5, 0, 1, 1817.7, -0.897, A4);
SensorFilter sL(0.2, 0.5, 0, 1, 2200.9, -1.001, A5);

// intialise the vector for the box mapping
Vector2D boxMap = Vector2D();

// Gyro stuff
const int gyroPin = A3;
int sensorValue = 0;
float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 0;      // Voltage when not rotating
float gyroSensitivity = 0.007;  // Taken from data sheets
float rotationThreshold = 0.1;  // For gyro drift correction// ? ***
float gyroRate = 0;
float currentAngle = 0;
unsigned long lastTime = 0;
float desiredAngle = 0;
float reading_middle = 0;
float reading_right = 0;
float reading_left = 0;
float turn_error = 900;
float ultraSonic = 0;
float Distance = 900;

double mapping[10][2];


#define BLUETOOTH_RX 10  // Serial Data input pin
#define BLUETOOTH_TX 11  // Serial Data output pin
// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0
// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 50;
const byte left_rear = 51;
const byte right_rear = 47;
const byte right_front = 46;

//IR range sensors
int rightShort = analogRead(A4);
int leftShort = analogRead(A5);
int rightLong = analogRead(A6);
int leftLong = analogRead(A7);

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;  //48
const int ECHO_PIN = 49;  //49

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;    // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

/* ----------------------------------------------- CONTROL SYS ---------------------------------------------- */
//State machine states
enum State {
  TOWALL,
  AWAYWALL,
  FIRSTLANE,
  NEXTLANE,
  CWSPIN,
  CCWSPIN,
  ALIGN,
  STOP,
  TURNTOLIGHT,
  DRIVETOLIGHT,
  DRIVETOLIGHTCLOSE,
  STRAFELEFT,
  STRAFERIGHT
};


//MISC
double sens_x = 0;  //US signal processed sensor value  for control sys
double sens_y = 0;
double sens_z = 0;  // angle
int current_lane = 0;

// error
double error_x = 0;
double error_y = 0;
double error_z = 0;
double sum_error_z = 0;
double previous_error_z = 0;
double error_z_derivative = 0;

// Control sys Arrays
double speed_array[4][1];           // array of speed values for the motors
double control_effort_array[3][1];  // array of xyz control efforts from pid controlers

// CONTROL GAIN VALUES
double kp_x = 1;
double kp_y = 60;
double kp_z = 0.4;
double ki_z = 0;
double kd_z = 0;
double power_lim = 500;  // max vex motor power

//timing
double accel_start_time = 0;
double accel_elasped_time = 0;

// float front = 300; // Initialise US reading
// float left_front = 300; // Initialise IR reading over left wheel
// float right_front = 300;
// float left = 300; // Initialise left IR reading (90 deg from straight, for strafing)
// float right = 300;
// float left_pt = 300; // Initialise left PT
// float right_pt = 300;
// float mid_pt; // If we use


// // Function to update readings
// void updateReadings() {
  
// }

// initial speed value (for serial movement control)
int speed_val = 200;
/*------------------------------------------------------------------------------------- */

//Serial Pointer for USB com
HardwareSerial *SerialCom;

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

void setup(void) {
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("SETUP");

  Serial.begin(115200);

  // Gyro Calibration
  pinMode(gyroPin, INPUT);
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sensorValue = analogRead(gyroPin);
    sum += (float)sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;

  // Motor initalization
  left_front_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_front_motor.attach(right_front);
  right_rear_motor.attach(right_rear);
}

void loop(void)  //main loop
{
  
  

  // forward();

  // right_front_motor.writeMicroseconds(1500 + 400);

  // while (1) {
    
  //   // delay(500);
  //   // SerialCom->println(HC_SR04_range());
    // SerialCom->println(analogRead(A4)); // Short right
    // SerialCom->println(analogRead(A5)); // Short left
    // SerialCom->println(analogRead(A6)); // Long right
    // SerialCom->println(analogRead(A7)); // Long left
  //   // updateAngle();
  //   // SerialCom->println(currentAngle); // Short right
  //   // SerialCom->println(analogRead(A14)); // PT left
  //   // SerialCom->println(analogRead(A12)); // PT mid
  //   // SerialCom->println(analogRead(A13)); // PT right
  //   delay(1000);
  //   Serial.println(analogRead(A14)); // PT l
  //   Serial.println(analogRead(A12)); // PT m
  //   Serial.println(analogRead(A13)); // PT r
  //   Serial.println("\n"); // PT mid


  // }

  float reading_left = analogRead(A14);
  float reading_right = analogRead(A13);
  // rightShort = sR.read();
  // leftShort = sL.read();
  // rightLong = lR.read();
  // leftLong = lL.read();
  // delay(100);
  // SerialCom->println(HC_SR04_range());
 
  float turn_error = 900;
  desiredAngle = turn_error;
  driveToLight();
  driveToLightClose();
  while(1);
}
void turnToLight(){
   do {
        State state = TURNTOLIGHT;
        
        reading_left = analogRead(A14);
        reading_right = analogRead(A13);
        turn_error = reading_left - reading_right;
        desiredAngle = turn_error;
       
        control(0, 0, 1, state);


      } while ((abs(turn_error) > 100));
}

void strafeRight(){
  int count = 0;
   do {
    
    rightShort = sR.read();
    leftShort = sL.read();
    rightLong = lR.read();
    leftLong = lL.read();
    State state = STRAFERIGHT;
    rightShort = analogRead(A4);
    leftShort = analogRead(A5);
    delay(200);
    ultraSonic = HC_SR04_range();
    SerialCom->println("right");
    SerialCom->println(ultraSonic);
    control(0, 1, 0, state);
    } while ((rightShort < 15 || leftShort < 15 || ultraSonic < 10));
}

void strafeLeft(){
  int count = 0;
   do {
    
    SerialCom->println("left");
    rightShort = sR.read();
    leftShort = sL.read();
    rightLong = lR.read();
    leftLong = lL.read();
    State state = STRAFELEFT;
    rightShort = analogRead(A4);
    leftShort = analogRead(A5);
    delay(200);
    ultraSonic = HC_SR04_range();
    control(0, 1, 0, state);
    } while ((rightShort < 15 || leftShort < 15 || ultraSonic < 10));
}

void driveToLight(){

   do {
        SerialCom->println("straight");
        State state = DRIVETOLIGHT;
        
        reading_middle = analogRead(A12);
        reading_left = analogRead(A14);
        reading_right = analogRead(A13);
        
        ultraSonic = HC_SR04_range();
        SerialCom->println(ultraSonic);
        rightShort = sR.read();
        leftShort = sL.read();
        rightLong = lR.read();
        leftLong = lL.read();
        turn_error = reading_left - reading_right;
        desiredAngle = turn_error;
        // SerialCom->print(turn_error);
        // SerialCom->print(", ");
        // SerialCom->println(reading_middle);
        control(1, 0, 1, state);

        // light too far so must be obstacle
        if (reading_middle < 800){
          // detect object on right ir sensor
          if(rightShort < 15){
            if(leftLong > 10){
              strafeLeft();
            }else if(rightLong > 10){
              strafeRight();
            }
          }
          // detect object on left IR sesnor
          else if(leftShort < 15){
            if(rightLong > 10){
              strafeRight();
            }else if(leftLong > 10){
              strafeLeft();
            }
          }
          else if(ultraSonic < 10){
           if(rightLong > 10){
              strafeRight();
            }else if(leftLong > 10){
              strafeLeft();
               }
        }
      }


      } while ((abs(turn_error) > 100) || (reading_middle < 950));
}

void driveToLightClose() {
  do {
        State state = DRIVETOLIGHTCLOSE;
        Distance = HC_SR04_range();
        reading_left = analogRead(A14);
        reading_right = analogRead(A13);
        turn_error = reading_left - reading_right;
        desiredAngle = turn_error;
        kp_x = 20;
        SerialCom->print(turn_error);
        SerialCom->print(", ");
        SerialCom->println(reading_middle);
        control(1, 0, 1, state);


      } while ((abs(turn_error) > 100) || (Distance > 2));
}

void updateAngle() {
  // Time calculation (in seconds)
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - lastTime) / 1e6;  // Convert µs to seconds
  lastTime = currentTime;

  // Read gyro and calculate angular velocity
  float gyroVoltage = (analogRead(gyroPin) * gyroSupplyVoltage) / 1023.0;
  float gyroRate = gyroVoltage - (gyroZeroVoltage * gyroSupplyVoltage / 1023.0);
  float angularVelocity = gyroRate / gyroSensitivity;  // °/s from datasheet

  // Update angle (integrate angular velocity)
  if (abs(angularVelocity) > rotationThreshold) {
    currentAngle += angularVelocity * deltaTime;  // θ = ∫ω dt
  }
  // if(currentAngle>361){
  //   currentAngle =- 360;
  // }
}

void turnTo(float Angle) {
  desiredAngle = Angle;
  controlReset();
  do {
    State state = CWSPIN;
    updateAngle();
    control(0, 0, 1, state);
    // serialOutput(3,3,error_z);
  } while (abs(error_z) > 1);
  stop();
}

// charlies magnum XL func
void findCorner() {

  cw();  // Rotate cw

  float step = 0.5;
  // float lastReading = HC_SR04_range();

  while (currentAngle <= 360) {
    // serialOutput(0,0,currentAngle);
    // delayMicroseconds(3500); // Time (ish) of serialOutput (seems to work)
    // Delay needs to be the right size to fill the vector

    int currentReading = HC_SR04_range();
    updateAngle();
    if ((currentAngle - step) > 0.5) {  //&& abs(lastReading - currentReading) < 50
      boxMap.insert_pair(currentAngle, currentReading);
      step += 1;
      serialOutput(0, currentReading, currentAngle);
    }
  }
  stop();
  delay(200);
  updateAngle();




  // get indexs of of walls
  size_t smallest_dist_index = boxMap.get_index_of_smallest_distance();
  size_t index90 = boxMap.find_angle_offset_from_index(smallest_dist_index, 90);
  size_t index180 = boxMap.find_angle_offset_from_index(smallest_dist_index, 180);
  size_t index270 = boxMap.find_angle_offset_from_index(smallest_dist_index, 270);

  // // get distances
  int smallest_dist = boxMap.get_distance(smallest_dist_index);
  int dist90 = boxMap.get_distance(index90);
  int dist180 = boxMap.get_distance(index180);
  int dist270 = boxMap.get_distance(index270);

  int smallAngle = boxMap.get_angle(smallest_dist_index);
  int angle90 = boxMap.get_angle(index90);
  int angle180 = boxMap.get_angle(index180);
  int angle270 = boxMap.get_angle(index270);

  int boxLength1 = smallest_dist + dist180;
  int boxLength2 = dist90 + dist270;

  // ///SERIALLLLLLLLLL CHECKSSSSSSSSSSSSSS////////////////////////////////

  // serialOutput(0, smallest_dist, smallAngle);
  // serialOutput(0, dist90, angle90);
  // serialOutput(0, dist180, angle180);
  // serialOutput(0, dist270, angle270);

  int turn_flag = -1;
  // find the long side of box and turn towards
  if (boxLength1 < boxLength2) {
    turnTo(smallAngle);
    turn_flag = 1;
  } else {
    turn_flag = 0;
    if (dist90 < dist270) {
      turnTo(angle90);
    } else {
      turnTo(angle270);
    }
  }

  currentAngle = 0;

  //zero angle against wall
  controlReset();
  accel_start_time = 3.5;
  do {
    State state = NEXTLANE;
    updateAngle();
    control(0, 1, 1, state);
  } while (abs(error_y) > 1); 
  left_front_motor.writeMicroseconds(1500 + 400);
  left_rear_motor.writeMicroseconds(1500 - 400);
  right_rear_motor.writeMicroseconds(1500 - 500);
  right_front_motor.writeMicroseconds(1500 + 400);
  delay(200);
  left_front_motor.writeMicroseconds(1500 + 400);
  left_rear_motor.writeMicroseconds(1500 - 400);
  right_rear_motor.writeMicroseconds(1500 - 0);
  right_front_motor.writeMicroseconds(1500 + 0);
  delay(200);
  left_front_motor.writeMicroseconds(1500 + 200);
  left_rear_motor.writeMicroseconds(1500 - 200);
  right_rear_motor.writeMicroseconds(1500 - 0);
  right_front_motor.writeMicroseconds(1500 + 0);
  delay(100);
    left_front_motor.writeMicroseconds(1500 + 100);
  left_rear_motor.writeMicroseconds(1500 - 100);
  right_rear_motor.writeMicroseconds(1500 - 0);
  right_front_motor.writeMicroseconds(1500 + 0);
  delay(100);
  stop();
  currentAngle = -2;
  delay(300);
  
  // figuire out and go to closest wall on X axis
  controlReset();
  if (dist90 > dist270 && turn_flag == 1) {
    do {
      State state = TOWALL;
      updateAngle();
      control(1, 1, 1, state);
    } while (abs(error_x) > 1);
    stop();
  } else if (dist90 < dist270 && turn_flag == 1) {
    do {
      State state = AWAYWALL;
      updateAngle();
      control(1, 1, 1, state);
    } while (abs(error_x) > 1);
    stop();
  } else if (dist90 < dist270 && turn_flag == 0) {
    do {
      State state = TOWALL;
      updateAngle();
      control(1, 1, 1, state);
    } while (abs(error_x) > 1);
    stop();
  } else {
    do {
      State state = AWAYWALL;
      updateAngle();
      control(1, 1, 1, state);
    } while (abs(error_x) > 1);
    stop();
  }

  stop();
}

boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}

float HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: NOT found");
      return -1;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      // SerialCom->println("HC-SR04: Out of range");
      return -1;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    // SerialCom->println("HC-SR04: Out of range");
  } else {
    // SerialCom->print("HC-SR04:");
    // SerialCom->print(cm);
    // SerialCom->println("cm");
    return cm;
  }
}

void controlReset() {
  stop();
  error_x = 0;
  error_y = 0;
  error_z = 0;
  sum_error_z = 0;
  accel_start_time = millis();
}

void plow(bool direction) {
  // serialOutput(0,0,HC_SR04_range());
  //delay(3000);

  // move foward first
  if (direction) {
    for (int i = 0; i < 10; i += 2) {
      
      current_lane = i;
    
      controlReset();
      accel_start_time = 3.5;
      do {
        State state = NEXTLANE;
        updateAngle();
        control(0, 1, 1, state);

      } while (abs(error_y) > 1);

      //GET Y START DIST
      mapping[current_lane][0] = HC_SR04_range();

      currentAngle -= 1;

      controlReset();
      do {
        State state = TOWALL;
        updateAngle();
        control(1, 1, 1, state);
      } while (abs(error_x) > 1);

      //GET Y END DIST
      mapping[current_lane][1] = HC_SR04_range();
      //KNOWING MAX X IS 2000CM, INTERPOLATE NUM POINTS (LOOK AT SOFTWARE BOXES) BETWEEN YSTART AND Y END
      //OUTPUT LANE NUMBER AND ^^^ DATA

      current_lane = i + 1;
     
      controlReset();
      accel_start_time = 3.5;
      do {
        State state = NEXTLANE;
        updateAngle();
        control(0, 1, 1, state);
      } while (abs(error_y) > 1);

      //GET Y START DIST
      mapping[current_lane][0] = HC_SR04_range();

      currentAngle -= 1;  //correct for drift

      controlReset();
      do {
        State state = AWAYWALL;
        updateAngle();
        control(1, 1, 1, state);
      } while (abs(error_x) > 1);
      //GET Y END DIST
      mapping[current_lane][1] = HC_SR04_range();
      //KNOWING MAX X IS 2000CM, INTERPOLATE NUM POINTS (LOOK AT SOFTWARE BOXES) BETWEEN YSTART AND Y END
      //OUTPUT LANE NUMBER AND ^^^ DATA
    }
  } else {  //move back first
    for (int i = 0; i < 10; i += 2) {
      current_lane = i;

      controlReset();
      accel_start_time = 3.5;
      do {
        State state = NEXTLANE;
        updateAngle();
        control(0, 1, 1, state);

      } while (abs(error_y) > 1);
      mapping[current_lane][0] = HC_SR04_range();
      //GET Y START DIST
      currentAngle -= 1;

      controlReset();
      do {
        State state = AWAYWALL;
        updateAngle();
        control(1, 1, 1, state);
      } while (abs(error_x) > 1);

      //GET Y END DIST
      //KNOWING MAX X IS 2000CM, INTERPOLATE NUM POINTS (LOOK AT SOFTWARE BOXES) BETWEEN YSTART AND Y END
      //OUTPUT LANE NUMBER AND ^^^ DATA
      mapping[current_lane][1] = HC_SR04_range();
      current_lane = i + 1;
     
      controlReset();
      accel_start_time = 3.5;
      do {
        State state = NEXTLANE;
        updateAngle();
        control(0, 1, 1, state);
      } while (abs(error_y) > 1);

      //GET Y START DIST
      mapping[current_lane][0] = HC_SR04_range();
      currentAngle -= 1;  //correct for drift

      controlReset();
      do {
        State state = TOWALL;
        updateAngle();
        control(1, 1, 1, state);
      } while (abs(error_x) > 1);
      mapping[current_lane][1] = HC_SR04_range();
      //GET Y END DIST
      //KNOWING MAX X IS 2000CM, INTERPOLATE NUM POINTS (LOOK AT SOFTWARE BOXES) BETWEEN YSTART AND Y END
      //OUTPUT LANE NUMBER AND ^^^ DATA
    }
  }
}

void control(bool toggle_x, bool toggle_y, bool toggle_z, State run_state) {
  // implement states for different control directions (to wall / away from wall
  sens_x = 0;
  if(current_lane == 0){
    sens_y = HC_SR04_range() - 6;
  } else if(current_lane == 8){
    sens_y = HC_SR04_range() - 11;
  } else {
    sens_y = HC_SR04_range() - 8;
  }
  
  sens_z = currentAngle;
  // Serial.println(currentAngle);


  //calc error_x based on to wall or away from wall
  switch (run_state) {
    case TOWALL:
      error_x = sL.read() - 6;
      error_y = 10 * current_lane - sens_y;
      error_z = 0 + sens_z;
      break;
    case AWAYWALL:
      error_x = 10 - sR.read();
      error_y = 10 * current_lane - sens_y;
      error_z = 0 + sens_z;
      break;
    case FIRSTLANE:
      error_x = 0;
      error_y = 4 - sens_y;  // not actually
      error_z = 0 + sens_z;
      break;
    case NEXTLANE:
      error_x = 0;
      error_y = 10 * current_lane - sens_y;
      error_z = 0 + sens_z;
      break;
    case CCWSPIN:
      error_x = 0;
      error_y = 0;  // not actually
      error_z = desiredAngle - sens_z;
      break;
    case CWSPIN:
      error_x = 0;
      error_y = 0;  // not actually
      error_z = sens_z - desiredAngle;
      break;
    case ALIGN:
      error_x = 0;
      error_y = 0;  // not actually
      error_z = 0 + sens_z;
      break;
    case STOP:
      error_x = 0;
      error_y = 0;  // not actually
      error_z = 0;
      break;
    case TURNTOLIGHT:
      error_x = 0;
      error_y = 0; // not actually
      error_z = desiredAngle;
      break;
    case DRIVETOLIGHT:
      error_x = 1000-reading_middle;
      error_y =  0; // not actually
      error_z = desiredAngle;
      break;
    case DRIVETOLIGHTCLOSE:
      error_x = Distance - 2;
      error_y =  0; // not actually
      error_z = desiredAngle;
      break;
    case STRAFELEFT:
      error_x = 0;
      error_y = 10;
      error_z = 0;
      break;
    case STRAFERIGHT:
      error_x = 0;
      error_y = -10;
      error_z = 0;
      break;

  }

  //ki_z
  if (abs(error_z) < 15) {
    sum_error_z += error_z;
  }
  else{
    sum_error_z = 0;
  }

  //kd_z
  // Calculate derivative term (rate of change of error)
  error_z_derivative = error_z - previous_error_z;

  // Update previous error for next iteration
  previous_error_z = error_z;



  // calc control efforts
  control_effort_array[0][0] = (toggle_x)
                                 ? error_x * kp_x
                                 : 0;
  control_effort_array[1][0] = (toggle_y)
                                 ? error_y * kp_y
                                 : 0;
  control_effort_array[2][0] = (toggle_z)
                                 ? error_z * kp_z + sum_error_z * ki_z + error_z_derivative * kd_z  // + sum_error_z * ki_z
                                 : 0;

  calcSpeed();
  move();

  //4hz control
  delay(25);
}

void calcSpeed() {

  float y = 1;
  float z = 1;

  control_effort_array[0][0] = constrain(control_effort_array[0][0], -power_lim, power_lim);  //500
  float controlEffortSum = abs(control_effort_array[0][0]) + abs(control_effort_array[1][0]);

  //scale Y control to still compensate when robot is moving
  if (abs(control_effort_array[0][0]) != 0) {
    y = 2 * abs(control_effort_array[0][0]) / power_lim;
    control_effort_array[1][0] *= y;
  } else {
    y = 0.8;
    control_effort_array[1][0] *= y;
    control_effort_array[1][0] = constrain(control_effort_array[1][0],-200,200); // constrain y when only strafing to avoid overshoot
  }

  

  // Check if the sum exceeds powerlim
  if (controlEffortSum > power_lim) {
    // Scale both control efforts to keep the ratio and sum <= powerlim
    float scaleFactor = power_lim / controlEffortSum;
    control_effort_array[0][0] *= scaleFactor;
    control_effort_array[1][0] *= scaleFactor;
  }

  speed_array[0][0] = control_effort_array[0][0] - control_effort_array[1][0];
  speed_array[1][0] = control_effort_array[0][0] + control_effort_array[1][0];
  speed_array[2][0] = control_effort_array[0][0] - control_effort_array[1][0];
  speed_array[3][0] = control_effort_array[0][0] + control_effort_array[1][0];

  // Find the maximum absolute value in speed_array
  float maxValue = max(max(abs(speed_array[0][0]), abs(speed_array[1][0])), max(abs(speed_array[2][0]), abs(speed_array[3][0])));

  //scale Z control to still compensate when robot is moving
  if (maxValue != 0) {
    z = 4 * maxValue / power_lim; 
  } else {
    z = 1;
  }

  // Apply the corrections with the dynamically adjusted z
  speed_array[0][0] -= z * control_effort_array[2][0];
  speed_array[1][0] += z * control_effort_array[2][0];
  speed_array[2][0] += z * control_effort_array[2][0];
  speed_array[3][0] -= z * control_effort_array[2][0];

  // Find the maximum absolute value in speed_array
  maxValue = max(max(abs(speed_array[0][0]), abs(speed_array[1][0])), max(abs(speed_array[2][0]), abs(speed_array[3][0])));

  // Check if the max absolute value exceeds 700
  if (maxValue > power_lim) {
    // Calculate the scale factor to bring the max absolute value down to 700
    float scaleFactor = power_lim / maxValue;

    // Scale all speed_array values by the same factor
    speed_array[0][0] *= scaleFactor;
    speed_array[1][0] *= scaleFactor;
    speed_array[2][0] *= scaleFactor;
    speed_array[3][0] *= scaleFactor;
  }

  //smooth accell for 1sec via multiplicative approach
  accel_elasped_time = (millis() - accel_start_time) / 1000;

  // Serial.print(accel_elasped_time);
  // Serial.print(" ");
  if (accel_elasped_time <= 5.5) {
    speed_array[0][0] *= (1 - exp(-1.0 * accel_elasped_time));  // 5 so reaches full value in 1 sec
    speed_array[1][0] *= (1 - exp(-1.0 * accel_elasped_time));
    speed_array[2][0] *= (1 - exp(-1.0 * accel_elasped_time));
    speed_array[3][0] *= (1 - exp(-1.0 * accel_elasped_time));
  }
}


void move() {
  left_front_motor.writeMicroseconds(1500 + speed_array[0][0]);
  left_rear_motor.writeMicroseconds(1500 + speed_array[3][0]);
  right_rear_motor.writeMicroseconds(1500 - speed_array[2][0]);
  right_front_motor.writeMicroseconds(1500 - speed_array[1][0]);
}

void printMap(){

  for(int i=0; i<10;i++){
    SerialCom->println("------- LANE [] ------- START X : Y ------- END X : Y ------");
    SerialCom->print(i,DEC);
    SerialCom->print("  ");
    SerialCom->print(mapping[i][0]);
    SerialCom->print(", ");
    //SerialCom->print(mapping[i][0],DEC)
    SerialCom->print("  ");
    SerialCom->print(mapping[i][1]);
    SerialCom->print(", ");
    //BluetoothSerial.println(mapping[i][1],DEC)

  }

delay(2000);
}




//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_front_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();    // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();   // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_front_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_front_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);      // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);    // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void forward() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}


void reverse() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void PIN_reading(int pin) {
  Serial.print("PIN ");
  Serial.print(pin);
  Serial.print(": ");
  Serial.println(analogRead(pin));
}


// From wireless module
void serialOutputMonitor(int32_t Value1, int32_t Value2, float Value3) {
  String Delimiter = ", ";
  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void serialOutputPlotter(int32_t Value1, int32_t Value2, float Value3) {
  String Delimiter = ", ";
  Serial.print(Value1, DEC);
  Serial.print(Delimiter);
  Serial.print(Value2, DEC);
  Serial.print(Delimiter);
  Serial.println(Value3, DEC);
}

void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2, float Value3) {
  String Delimiter = ", ";
  BluetoothSerial.print(Value1, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.print(Value2, DEC);
  BluetoothSerial.print(Delimiter);
  BluetoothSerial.println(Value3, DEC);

  // char out_char[100];
  // sprintf(out_char,"%d , %d , %d\n", Value1, Value2, Value3);
  // BluetoothSerial.print(out_char);
}

void serialOutput(int32_t Value1, int32_t Value2, float Value3) {
  if (OUTPUTMONITOR) {
    serialOutputMonitor(Value1, Value2, Value3);
  }
  if (OUTPUTPLOTTER) {
    serialOutputPlotter(Value1, Value2, Value3);
  }
  if (OUTPUTBLUETOOTHMONITOR) {
    bluetoothSerialOutputMonitor(Value1, Value2, Value3);
    
  }
}
