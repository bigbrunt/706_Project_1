#include <Servo.h>           //Need for Servo pulse output
#include <SoftwareSerial.h>  // For wireless communication
#include <Arduino.h>
#include "SensorFilter.h"
#include "vector.h"

// Define Kalman filter instance (Q, R, initial estimate, initial error covariance)
SensorFilter l1(0.2, 0.5, 0, 1, 16705, -1.22, A9);
SensorFilter l2(0.2, 0.5, 0, 1, 5434.6, -1.006, A5);
SensorFilter s1(0.2, 0.5, 0, 1, 1625.2, -0.899, A8);
SensorFilter s2(0.2, 0.5, 0, 1, 2482, -1.033, A4);

// intialise the vector for the box mapping
Vector2D boxMap = Vector2D();

// Gyro stuff
const int gyroPin = A3;
int sensorValue = 0;
float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 0;      // Voltage when not rotating
float gyroSensitivity = 0.007;  // Taken from data sheets
float rotationThreshold = 1.5;    // For gyro drift correction// ? ***
float gyroRate = 0;
float currentAngle = 0;
unsigned long lastTime = 0;


#define BLUETOOTH_RX 10 // Serial Data input pin
#define BLUETOOTH_TX 11 // Serial Data output pin
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
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

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
  FULLSPIN,
  ALIGN,
  STOP
};

// Kinematic Constants
double lx = 0.0759; // x radius (m) (robot)
double ly = 0.09; // y radius (m) (robot  )
double rw = 0.0275; //wheel radius (m)

//MISC
double sens_x = 0; //US signal processed sensor value  for control sys
double sens_y = 0;
double sens_z = 0; // angle
double max_x = 165; // max drivable x length m
int current_lane = 0;

// error
double error_x = 0;
double error_y = 0;
double error_z = 0;
double sum_error_z = 0;

// Control sys Arrays
double speed_array[4][1]; // array of speed values for the motors
double control_effort_array[3][1]; // array of xyz control efforts from pid controlers
double ki_memory_array[3][1];

// CONTROL GAIN VALUES
double kp_x = 15;
double kp_y = 25;
double kp_z = 40;
// double kp_z_straight = 40;
double ki_x = 0;
double ki_y = 0;
double ki_z = 5;
double power_lim = 500; // max vex motor power

//timing
double accel_start_time = 0;
double accel_elasped_time = 0;


// initial speed value (for serial movement control)
int speed_val = 100;
/*------------------------------------------------------------------------------------- */

//Serial Pointer for USB com
HardwareSerial *SerialCom;

int pos = 0;

volatile int32_t Counter = 1;  // Used to delay serial outputs

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

void setup(void) {
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  BluetoothSerial.begin(115200);

  //delay(1000);  //settling time but no really needed

  // Gyro stuff
  int i;
  float sum = 0;
  for (i = 0; i < 100; i++) {
    sensorValue = analogRead(gyroPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;

  // Needed to get the robot moving
  left_front_motor.attach(left_front);
  left_rear_motor.attach(left_rear);
  right_front_motor.attach(right_front);
  right_rear_motor.attach(right_rear);

  // findCorner();
}

void loop(void)  //main loop
{
     l1.read();
     l2.read();
     s1.read();
     s2.read();
    //  delay(500);
    //  Serial.println(l2_data);
    
    // forward();
    // float range = HC_SR04_range();
    // while(range > 5){
    //   serialOutput(2,2,range);
    //   range = HC_SR04_range();
    // }
   
    // stop();
    // delay(1000);
   
    plow();
    while(1){}
    //goToWall();
};

void updateSensors() {
    l1.read();
    l2.read();
    s1.read();
    s2.read();
}


void updateAngle() {
     // Time calculation (in seconds)
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1e6;  // Convert µs to seconds
    lastTime = currentTime;

    // Read gyro and calculate angular velocity
    float gyroVoltage = (analogRead(gyroPin) * gyroSupplyVoltage) / 1023.0;
    float gyroRate = gyroVoltage - (gyroZeroVoltage * gyroSupplyVoltage / 1025.0);
    float angularVelocity = gyroRate / gyroSensitivity;  // °/s from datasheet

    // Update angle (integrate angular velocity)
    if (abs(angularVelocity) > rotationThreshold) {
      currentAngle += angularVelocity * deltaTime;  // θ = ∫ω dt
    } 
}

void turnTo(float desiredAngle) { 
  lastTime = micros();  // Record the starting time
  float angleDifference = desiredAngle - currentAngle;

  // Normalize the angle difference to be within -180 to 180 degrees
  if (angleDifference > 180) {
    angleDifference -= 360;
  } else if (angleDifference < -180) {
    angleDifference += 360;
  }

  // If the angle difference is positive, rotate clockwise (cw)
  // If the angle difference is negative, rotate counterclockwise (acw)
  if (angleDifference > 0) {
    cw();  // Rotate clockwise
  } else {
    ccw();  // Rotate counterclockwise
  }

  // Rotate until the current angle is close enough to the desired angle
  while (abs(currentAngle - desiredAngle) > 0.5) {
    // delayMicroseconds(3500);
    // serialOutput(0, 0, currentAngle);

    // Time calculation (in seconds)
    // updates current angle
    delayMicroseconds(3500);
    updateAngle();
    updateSensors();
  }
  stop();  // Stop the motors when the desired angle is reached
  delay(1000); // For now
}

// charlies magnum XL func
void findCorner() {
  
  lastTime = micros();  // Record the starting time
  cw(); // Rotate cw

  float step = 0.5;

  while (currentAngle <= 360) {
    // serialOutput(0,0,currentAngle);
    // delayMicroseconds(3500); // Time (ish) of serialOutput (seems to work)
    // Delay needs to be the right size to fill the vector

    int currentReading = HC_SR04_range();
    updateAngle();
    if ((currentAngle - step) > 0.5) {
      boxMap.insert_pair(currentAngle, currentReading);
      step += 1;
    }

    updateSensors();
    // delay(100); // Think we are overflowing the vector without a delay (may need to trial/error for this value)
    
  }

  currentAngle = 0;

  stop();
  delay(1000);

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

  int boxLength1 = smallest_dist + dist180;
  int boxLength2 = dist90 + dist270;

   ///SERIALLLLLLLLLL CHECKSSSSSSSSSSSSSS////////////////////////////////

  serialOutput(0, 0, smallest_dist);
  serialOutput(0, 0, dist90);
  serialOutput(0, 0, dist180);
  serialOutput(0, 0, dist270); 

  // serialOutput(0, 0, smallest_dist_index);
  // serialOutput(0, 0, index90);
  // serialOutput(0, 0, index180);
  // serialOutput(0, 0, index270); 

  // Serial.println("Stored angles:");
  //   for (size_t i = 0; i < boxMap.length(); i++) {
  //   Serial.print(boxMap.get_angle(i));
  //   Serial.print(" ");
  // }
  // Serial.println(" ");
  // Serial.print("0 angle: ");
  // Serial.println(boxMap.get_angle(smallest_dist_index));

  // Serial.print("90 angle: ");
  // Serial.println(boxMap.get_angle(index90));

  // Serial.print("180 angle: ");
  // Serial.println(boxMap.get_angle(index180));

  // Serial.print("270 angle: ");
  // Serial.println(boxMap.get_angle(index270));

  // find the long side of box and turn towards
  if(boxLength1>boxLength2){
    // Turn to the 90 or 180 wall, which ever is closest
    turnTo(boxMap.get_angle(smallest_dist_index));
  } else {
    // Turn to smallest reading
    // turnTo(boxMap.get_angle(smallest_dist_index));
    if (dist90 < dist270) {
      turnTo(boxMap.get_angle(index90));
    } else {
      turnTo(boxMap.get_angle(index270));
    }
  }
  
  forward();
  while ( HC_SR04_range() > 3) {
    // Continue
  }

  stop();
  delay(1000);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  float leftLIR = l2.read();
  float rightLIR = l1.read(); // Hopefully these both give valid readings

  if (leftLIR < rightLIR) {
    strafe_left();
    while (s2.read() > 3) {
      // Keep going
      updateAngle();
      updateSensors();
    }
  } else {
    strafe_right();
    while (s1.read() > 3) {
      // Keep going
      updateAngle();
      updateSensors();
    }
  }
  stop();
  delay(1000);
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

void controlReset(){
  
  sum_error_z = 0;
  accel_start_time = millis();
}
void plow() {
    // serialOutput(0,0,HC_SR04_range());
    
    // ------------------  plow lane 0  ---------------
    controlReset();
    
    do  {
      current_lane = 0;
      State state = FIRSTLANE;
      updateAngle();
      control(0, 1, 0, state);
      serialOutput(0,0,currentAngle);
    } while (abs(error_y) > 1);
    
    controlReset();
    
    do  {
      State state = TOWALL;
      updateAngle();
      control(1, 0, 1, state);
      // serialOutput(1,1,HC_SR04_range());
      serialOutput(1, 1,currentAngle);
    } while (abs(error_x) > 1);

    // --------------- plow lane 1  -----------------

    controlReset();
    
    do  {
      current_lane = 1;
      State state = NEXTLANE;
      updateAngle();
      control(0, 1, 0, state);  
      serialOutput(2, 2,currentAngle);    
    } while (abs(error_y) > 1); 
    
    controlReset();
    
    do  {
      State state = AWAYWALL;
      updateAngle();
      control(1, 0, 1, state);
      serialOutput(3, 3,currentAngle);
    } while (abs(error_x) > 1);

  // --------------  plow lane 2  -----------------
    controlReset();
    
    do  {
      current_lane = 2;
      State state = NEXTLANE;
      updateAngle();
      control(0, 1, 0, state);      
    } while (abs(error_y) > 1);
    
    controlReset();
      
    do  {
      State state = TOWALL;
      updateAngle();
      control(1, 0, 1, state);
    } while (abs(error_x) > 1);

  // -------------    plow lane 3  ----------------------
    controlReset();
    
    do  {
      current_lane = 3;
      State state = NEXTLANE;
      updateAngle();
      control(0, 1, 0, state);
    } while (abs(error_y) > 1);
    
    controlReset();
    
    do  {
      State state = AWAYWALL;
      updateAngle();
      control(1, 0, 1, state);
    } while (abs(error_x) > 1);

    stop();

    // done

}


void control(bool toggle_x, bool toggle_y, bool toggle_z, State run_state) {
  // implement states for different control directions (to wall / away from wall
  sens_x = HC_SR04_range() - 4; //conv to m
  sens_y = 0;
  sens_z = currentAngle;


  //calc error_x based on to wall or away from wall
  switch (run_state) {
    case TOWALL:
    if(current_lane == 0){
        //Serial.println("TO WALL");
      error_x = constrain(sens_x, 0, 9999);
      error_y = s2.read() - 8;
      error_z = 0 + sens_z;
      }
      if(current_lane == 1){
        //Serial.println("TO WALL");
      error_x = constrain(sens_x, 0, 9999);
      error_y = l2.read() - 28;
      error_z = 0 + sens_z;
      }
      if(current_lane == 2){
        //Serial.println("TO WALL");
      error_x = constrain(sens_x, 0, 9999);
      error_y = l2.read() - 48;
      error_z = 0 + sens_z;
      }
      if(current_lane == 3){
        //Serial.println("TO WALL");
      error_x = constrain(sens_x, 0, 9999);
      error_y = l1.read() - 28;
      error_z = 0 + sens_z;
      }
      if(current_lane == 4){
        //Serial.println("TO WALL");
      error_x = constrain(sens_x, 0, 9999);
      error_y = s1.read() - 8;
      error_z = 0 + sens_z;
      }
      
      break;
    case AWAYWALL:
      if(current_lane == 0){
        //Serial.println("TO WALL");
      error_x = constrain((sens_x - max_x), -9999, 0);
      error_y = s2.read() - 8;
      error_z = 0 + sens_z;
      }
      if(current_lane == 1){
        //Serial.println("TO WALL");
      error_x = constrain((sens_x - max_x), -9999, 0);
      error_y = l2.read() - 28;
      error_z = 0 + sens_z;
      }
      if(current_lane == 2){
        //Serial.println("TO WALL");
      error_x = constrain((sens_x - max_x), -9999, 0);
      error_y = l2.read() - 48;
      error_z = 0 + sens_z;
      }
      if(current_lane == 3){
        //Serial.println("TO WALL");
      error_x = constrain((sens_x - max_x), -9999, 0);
      error_y = l1.read() - 28;
      error_z = 0 + sens_z;
      }
      if(current_lane == 4){
        //Serial.println("TO WALL");
      error_x = constrain((sens_x - max_x), -9999, 0);
      error_y = s1.read() - 8;
      error_z = 0 + sens_z;
      }
      break;
    case FIRSTLANE:
      error_x = 0;
      error_y = s2.read() - 8; // errror =0 when sense = 8 // not actually
      error_z = 0 + sens_z;
      break;
    case NEXTLANE:
      if(current_lane == 1){
        error_x = 0;
        error_y = l2.read() - 28; // need to update target lane
        error_z = 0 + sens_z;
      }
      if(current_lane == 2){
        error_x = 0;
        error_y = l2.read() - 48; // need to update target lane
        error_z = 0 + sens_z;
      }
      if(current_lane == 3){
        error_x = 0;
        error_y = l1.read() - 28; // need to update target lane
        error_z = 0 + sens_z;
      }
      if(current_lane == 4){
        error_x = 0;
        error_y = s1.read() - 8; // need to update target lane
        error_z = 0 + sens_z;
      }
      break;
    case FULLSPIN:
      error_x = 0;
      error_y = 0; // not actually
      error_z = 90 + sens_z;
      break;
    case ALIGN:
      error_x = 0;
      error_y = 0; // not actually
      error_z = 0 + sens_z;
    break;
    case STOP:
      error_x = 0;
      error_y = 0; // not actually
      error_z = 0;
      break;
  }

  //ki_z
  if (error_z < 3) {
    sum_error_z += error_z;
  }

  // calc control efforts
  control_effort_array[0][0] = (toggle_x)
                               ? error_x * kp_x
                               : 0;
  control_effort_array[1][0] = (toggle_y)
                               ? error_y * kp_y
                               : 0;
  control_effort_array[2][0] = (toggle_z)
                               ? error_z * kp_z + sum_error_z*ki_z
                               : 0;
  calcSpeed();
  move();
}

void calcSpeed() {

  // proitize power for spin correction
  double available_power = power_lim - abs(control_effort_array[2][0]);
  // Prevent negative available power (in case rotation alone exceeds limit)
  available_power = max(0.0, available_power);

  // Compute total speed-related correction demand
  double speed_correction_total = abs(control_effort_array[0][0]) + abs(control_effort_array[1][0]);

  // Compute scaling factor for speed corrections
  double speed_scaling_factor = (speed_correction_total > available_power)
                                ? available_power / speed_correction_total
                                : 1.0;

  // Apply scaling to speed corrections only
  control_effort_array[0][0] *= speed_scaling_factor;
  control_effort_array[1][0] *= speed_scaling_factor;

  
  speed_array[0][0] =  control_effort_array[0][0] - control_effort_array[1][0] - control_effort_array[2][0];
  speed_array[1][0] =  control_effort_array[0][0] + control_effort_array[1][0] +  control_effort_array[2][0];
  speed_array[2][0] =  control_effort_array[0][0] - control_effort_array[1][0] +  control_effort_array[2][0];
  speed_array[3][0] =  control_effort_array[0][0] + control_effort_array[1][0] -  control_effort_array[2][0];

  // Apply constraints to ensure motor speeds stay within limits
  speed_array[0][0] = constrain(speed_array[0][0], -power_lim, power_lim);
  speed_array[1][0] = constrain(speed_array[1][0], -power_lim, power_lim);
  speed_array[2][0] = constrain(speed_array[2][0], -power_lim, power_lim);
  speed_array[3][0] = constrain(speed_array[3][0], -power_lim, power_lim);


  //smooth accell for 1sec via multiplicative approach
  accel_elasped_time = (millis() - accel_start_time) / 1000;
  
  // Serial.print(accel_elasped_time);
  // Serial.print(" ");
    if (accel_elasped_time <= 2) {
      speed_array[0][0] *= (1 - exp(-2.0 * accel_elasped_time) ); // 5 so reaches full value in 1 sec
      speed_array[1][0] *= (1 - exp(-2.0 * accel_elasped_time) );
      speed_array[2][0] *= (1 - exp(-2.0 * accel_elasped_time) );
      speed_array[3][0] *= (1 - exp(-2.0 * accel_elasped_time) );
    }

  // Serial.print(speed_array[0][0]);
  // Serial.print(" ");
  // Serial.print(speed_array[1][0]);
  // Serial.print(" ");
  // Serial.print(speed_array[2][0]);
  // Serial.print(" ");
  // Serial.print(speed_array[3][0]);
  // Serial.println(" ");

}


void move() {
  left_front_motor.writeMicroseconds(1500 + speed_array[0][0]);
  left_rear_motor.writeMicroseconds(1500 + speed_array[3][0]);
  right_rear_motor.writeMicroseconds(1500 - speed_array[2][0]);
  right_front_motor.writeMicroseconds(1500 - speed_array[1][0]);
//  left_front_motor.writeMicroseconds(1500 + 100);
//  left_rear_motor.writeMicroseconds(1500 + 100);
//  right_rear_motor.writeMicroseconds(1500 - 100);
//  right_front_motor.writeMicroseconds(1500 - 100);
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
    ;
  }
}
