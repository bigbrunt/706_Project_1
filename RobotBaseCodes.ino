/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h> // For wireless communication
#include <Arduino.h>

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11

// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0

// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

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
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

// Kinematic Constants
double lx = 0.0759; // x radius (m) (robot)
double ly = 0.09; // y radius (m) (robot  )
double rw = 0.0275; //wheel radius (m)

//
double sens_x = 0; //US signal processed sensor value  for control sys
double max_x = 100; // max drivable x length
double error_x = 0;
// Control sys Arrays
double speed_array[4][1]; // array of speed values for the motors
double control_effort_array[3][1]; // array of xyz control efforts from pid controlers
double ki_memory_array[3][1];

// CONTROL GAIN VALUES
double kp_x = 0.75;
double kp_y = 0;
double kp_z = 0;
double ki_x = 0;
double ki_y = 0;
double ki_z = 0;
double power_lim = 700;

// initial speed value (for serial movement control)
int speed_val = 100;
int speed_change;


const int bufferSize = 400;  // Fixed buffer size of 100
int buffer[bufferSize];      // Array to hold the buffer
int front = 0;               // Index for the front of the buffer
int rear = 0;                // Index for the rear of the buffer
int size = 0;                // Current number of elements in the buffer
long sum = 0;                // Sum of the elements in the buffer (long for large sums)

//Serial Pointer for USB com
HardwareSerial *SerialCom;

int pos = 0;

volatile int32_t Counter = 1; // Used to delay serial outputs

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);


class RingBuf {
  private:
    int buffer[bufferSize];      // Array to hold the buffer
    int front = 0;               // Index for the front of the buffer
    int rear = 0;                // Index for the rear of the buffer
    int size = 0;                // Current number of elements in the buffer
    int pin;                    // pin number
    double coefficent;             // coefficent for ADC -> cm equation
    double exponent;            // exponent for ADC -> cm equation
    long sum = 0;                // Sum of the elements in the buffer (long for large sums)

  public:
    RingBuf(int p, double c, double e) {
      pin = p;
      coefficent = c;
      exponent = e;
      pinMode(pin, OUTPUT);
    }
    void intialise_buf() {
      // Initialize the buffer with 0s (optional)
      for (int i = 0; i < bufferSize; i++) {
        buffer[i] = 0;
      }
    }

    void push() {
      if (size == bufferSize) {
        // If the buffer is full, subtract the oldest value from the sum
        sum -= buffer[front];
        front = (front + 1) % bufferSize; // Move the front pointer forward
      } else {
        size++;
      }

      // Add the new value to the buffer and update the sum
      int value =  (int) coefficent * pow(analogRead(pin), exponent);
      // int value = analogRead(pin);
      buffer[rear] = value;  // Reading from analog pin;
      sum += value;

      // Move the rear pointer forward
      rear = (rear + 1) % bufferSize;
    }

    // Function to calculate the moving average
    int32_t movingAverage() {
      if (size == 0) {
        return 0; // Avoid division by zero if the buffer is empty
      }
      return (int32_t)sum / size;
    }
};

RingBuf IR_Long1(21, 3536.6, -0.86);
RingBuf IR_Long2(19, 5928.2, -1.062);
RingBuf IR_Short1(20, 27126, -1.038);
RingBuf IR_Short2(18, 2261.8, -0.981);


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

  

}

void loop(void)  //main loop
{
  enable_motors();
  goToWall();
}

void fast_flash_double_LED_builtin() {
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin() {
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

//for serial speed control
void speed_change_smooth() {
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
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
#endif

#ifndef NO_HC - SR04
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
      SerialCom->println("HC-SR04: NOT found");
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
      SerialCom->println("HC-SR04: Out of range");
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
    SerialCom->println("HC-SR04: Out of range");
  } else {
//        SerialCom->print("HC-SR04:");
//        SerialCom->print(cm);
//        SerialCom->println("cm");
    return cm;
  }
}
#endif

void Analog_Range_A4() {
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading() {
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif


void goToWall() {

  while (1) {
    control(1, 0, 0,1);
    calcSpeed();
    move();
    delay(1000);
  }


}


void control(bool toggle_x, bool toggle_y, bool toggle_z,bool to_wall) {
  // implement states for different control directions (to wall / away from wall
  sens_x = HC_SR04_range()-4;

  //calc error_x based on to wall or away from wall
  to_wall ? error_x = constrain(sens_x,0,9999) : error_x = constrain((sens_x - max_x),-9999,0);

// calc control efforts
  control_effort_array[0][0] = (toggle_x)
                               ? error_x* kp_x
                               : 0;
  control_effort_array[1][0] = (toggle_y)
                               ? 0
                               : 0;
  control_effort_array[2][0] = (toggle_x)
                               ? 0
                               : 0;

  

  ki_memory_array[0][0] += control_effort_array[0][0];
  ki_memory_array[1][0] += control_effort_array[1][0];
  ki_memory_array[2][0] += control_effort_array[2][0];

  //  Serial.println(HC_SR04_range());
  Serial.println(control_effort_array[0][0]);
}

void calcSpeed() {
  speed_array[0][0] =  constrain( (1 / rw) * (control_effort_array[0][0] - control_effort_array[1][0] - ((lx + ly) * control_effort_array[2][0])), -power_lim, power_lim);
  speed_array[1][0] =  constrain( (1 / rw) * (control_effort_array[0][0] + control_effort_array[1][0] + ((lx + ly) * control_effort_array[2][0])), -power_lim, power_lim);
  speed_array[2][0] =  constrain( (1 / rw) * (control_effort_array[0][0] - control_effort_array[1][0] + ((lx + ly) * control_effort_array[2][0])), -power_lim, power_lim);
  speed_array[3][0] =  constrain( (1 / rw) * (control_effort_array[0][0] + control_effort_array[1][0] - ((lx + ly) * control_effort_array[2][0])), -power_lim, power_lim);
  Serial.print(speed_array[0][0]);
  Serial.print(" ");
  Serial.print(speed_array[1][0]);
  Serial.print(" ");
  Serial.print(speed_array[2][0]);
  Serial.print(" ");
  Serial.print(speed_array[3][0]);
  Serial.print(" ");
  Serial.println(".");

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
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_front_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_front_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
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
