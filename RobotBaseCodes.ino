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

// Gyro stuff
const int gyroPin = A3;
int T = 100; // For 1 super loop
int sensorValue = 0;
float gyroSupplyVoltage = 5;
float gyroZeroVoltage = 0; // Voltage when not rotating
float gyroSensitivity = 0.007; // Taken from data sheets
float rotationThreshold = 3; // For gyro drift correction
float gyroRate = 0;
float currentAngle = 0;
// const float referenceVoltage = 5.0; // For Arduino
// const int zeroRate = 512; // Half of 1023, implies no rotation (check this is calibrated)
// float angle = 0; // Rotation from initial placement
// unsigned long lastTime = 0; 
// float rotationThreshold = 1.5; // For gyro drift
//


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

Servo left_font_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

double lx = 0.0759; // x radius (m) (robot)
double ly = 0.09; // y radius (m) (robot  )
double rw = 0.0275; //wheel radius (m)

double speed_array[4][1]; // array of speed values for the motors
double control_effort_array[3][1]; // array of xyz control efforts from pid controlers


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
      RingBuf(int p, double c, double e){
        pin = p;
        coefficent = c;
        exponent = e;
        pinMode(pin, OUTPUT);
      }
      void intialise_buf(){
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

RingBuf IR_Long1(21, 3536.6,-0.86);
RingBuf IR_Long2(19, 5928.2,-1.062);
RingBuf IR_Short1(20,27126,-1.038);
RingBuf IR_Short2(18,2261.8,-0.981);


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
  for (i=0; i<100; i++) {
    sensorValue = analogRead(gyroPin);
    sum += sensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum/100;
  //
}

void loop(void)  //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING:  //Lipo Battery Volage OK
      machine_state = running();
      break;
    case STOPPED:  //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state = stopped();
      break;
  };
//   Counter = Counter + 1;
//  // if (Counter > 100000) {
//     //GYRO_reading(); // Serial output gyro reading, not wireless at this stage
//     //PIN_reading(A4); // Serial output IR sensor reading
//     int32_t cm = (int32_t) HC_SR04_range();
//     int distancec =  2712.6 * pow(analogRead(A4), -1.038);
//     int newValue = analogRead(A4);  // Reading from analog pin A0 (can be replaced with any other input)
  
//    // Push the new value into the buffer
//     push(newValue);
//     //serialOutput(0, 0, PIN_get_reading(A4)); // Using counter as data index gives weird output
//     serialOutput(0, 0, movingAverage());
//     Counter = 0;
//   //}

  // Gyro stuff
  // unsigned long currentTime = micros(); // Get current time (ms)
  // float deltaTime = (currentTime - lastTime) / 1e6; // Convert to s
  // lastTime = currentTime;

  // int rawValue = analogRead(gyroPin);
  // float voltage = (float)(rawValue / 1023.0) * referenceVoltage; // Convert to voltage
  // float angularVelocity = (voltage - (zeroRate * referenceVoltage / 1023)) / sensitivity;

  // if ((angularVelocity >= rotationThreshold) || (angularVelocity <= -rotationThreshold)) {
  //   angle += angularVelocity * deltaTime; // Equivalent to integrating
  // }

  gyroRate = (analogRead(gyroPin)*gyroSupplyVoltage)/1023; // Convert to voltage
  gyroRate -= (gyroZeroVoltage/1023*gyroSupplyVoltage); // Gyro drift?
  float angularVelocity = gyroRate/gyroSensitivity; // From data sheet

  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    float angleChange = angularVelocity/(1000/T);
    currentAngle += angleChange;
  }

  serialOutput(0, 0, currentAngle); // Output wirelessly
  delay(T);
  //

  // goToWall();

    // IR_Long1.push(); // Push the new value into the buffer
    // IR_Long2.push(); // Push the new value into the buffer
    // IR_Short1.push(); // Push the new value into the buffer
    // IR_Short2.push(); // Push the new value into the buffer

    // int32_t long1val = IR_Long1.movingAverage();
    // int32_t long2val = IR_Long2.movingAverage();
    // int32_t short1val = IR_Short1.movingAverage();
    // int32_t short2val = IR_Short2.movingAverage();
    
    // serialOutput(0, 0, long1val); // print ir value
}

void push(int value) {
  if (size == bufferSize) {
    // If the buffer is full, subtract the oldest value from the sum
    sum -= buffer[front];
    front = (front + 1) % bufferSize; // Move the front pointer forward
  } else {
    size++;
  }

  // Add the new value to the buffer and update the sum
  buffer[rear] = value;
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

STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000);  //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC - SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif


    turret_motor.write(pos);

    if (pos == 0) {
      pos = 45;
    } else {
      pos = 0;
    }
  }

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) {  //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) {  //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
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
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
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

//Serial command pasing
void read_serial_command() {
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w':  //Move Forward
      case 'W':
        forward();
        SerialCom->println("Forward");
        break;
      case 's':  //Move Backwards
      case 'S':
        reverse();
        SerialCom->println("Backwards");
        break;
      case 'a':  //Strafe Left
      case 'A':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'd':  //Strafe Right
      case 'D':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'q':  //Turn Left
      case 'Q':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'e':  //Turn Right
      case 'E':
        cw();
        SerialCom->println("cw");
        break;
      case '-':  // - speed
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+': // + speed
        speed_change = 100;
        SerialCom->println("+");
        break;
      case 'x':
      case 'X':
        stop();
        SerialCom->println("stop");
        break;
      case 'r':
      case 'R':
        goToWall();
        // bluetoothSerial.print("Go To Wall");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }
  }
}


void calcSpeed(){
speed_array[1][1] = (int) (1/rw) * (control_effort_array[1][1] - control_effort_array[2][1] - (lx+ly) * control_effort_array[3][1]);
speed_array[2][1] = (int) (1/rw) * (control_effort_array[1][1] + control_effort_array[2][1] + (lx+ly) * control_effort_array[3][1]);
speed_array[3][1] = (int) (1/rw) * (control_effort_array[1][1] - control_effort_array[2][1] + (lx+ly) * control_effort_array[3][1]);
speed_array[4][1] = (int) (1/rw) * (control_effort_array[1][1] + control_effort_array[2][1] - (lx+ly) * control_effort_array[3][1]);

}

void control(){
  control_effort_array[1][1] = 10; // x
  control_effort_array[2][1] = 0; // y
  control_effort_array[3][1] = 0; // z

  

}

void goToWall(){
 
  while(HC_SR04_range() > 10){
    control_effort_array[1][1] = 10;
    calcSpeed();
    move();
    SerialCom->println(control_effort_array[2][1]);
  }
  stop();
  
}

void move() {
  left_font_motor.writeMicroseconds(1500 + speed_array[1][1]);
  left_rear_motor.writeMicroseconds(1500 + speed_array[4][1]);
  right_rear_motor.writeMicroseconds(1500 - speed_array[3][1]);
  right_font_motor.writeMicroseconds(1500 - speed_array[2][1]);
}


//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors() {
  left_font_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();   // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors() {
  left_font_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);     // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);   // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop()  //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void forward() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}


void reverse() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void ccw() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void cw() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left() {
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right() {
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
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