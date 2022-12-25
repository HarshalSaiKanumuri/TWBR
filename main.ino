#include <Wire.h>

// define MPU-6050 I2C address
const int MPU_ADDR = 0x68;

// register addresses
const int ACCEL_XOUT_H = 0x3B;
const int ACCEL_YOUT_H = 0x3D;
const int ACCEL_ZOUT_H = 0x3F;
const int GYRO_XOUT_H = 0x43;
const int GYRO_YOUT_H = 0x45;
const int GYRO_ZOUT_H = 0x47;

int16_t ax, ay, az; // raw accelerometer data
int16_t gx, gy, gz; // raw gyroscope data
float ax_offset, ay_offset, az_offset; // accelerometer offsets
float gx_offset, gy_offset, gz_offset; // gyroscope offsets
float pitch, roll; // pitch and roll angles
float pitch_gyro, roll_gyro; // pitch and roll angles from gyroscope
float pitch_acc, roll_acc; // pitch and roll angles from accelerometer
float dt; // time step
float k; // complimentary filter coefficient

#define motor1_pwm 5 //Channel A direction
#define motor1_dir A1 //Channel A direction
#define motor2_pwm 6 //Channel B direction
#define motor2_dir A3  //Channel B direction

// define PID controller parameters
double Kp = 0.5;
double Ki = 0.1;
double Kd = 0.1;
double setpoint = 0; // desired pitch angle
double output; // control signal
double error; // error between setpoint and pitch angle
double integral; // integral of error
double derivative; // derivative of error
double last_error; // error on previous sample
double sample_time = 0.01; // sample time in seconds


void setup() {
  // initialize serial communication
  Serial.begin(9600);

  // initialize I2C communication
  Wire.begin();

  // initialize MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero to wake up the MPU-6050
  Wire.endTransmission(true);

  // calibrate accelerometer and gyroscope
  calibrateSensors();

  // initialize pitch and roll angles to zero
  pitch = 0;
  roll = 0;

  // set time step and complimentary filter coefficient
 
  dt = 0.01;
  k = 0.98;  
  // initialize motor control pins
  pinMode(motor1_pwm, OUTPUT);
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  pinMode(motor2_dir, OUTPUT);
}

void loop() {
  // read raw accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  // read raw gyroscope data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  // subtract offsets from raw data
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // calculate pitch and roll angles from accelerometer
  pitch_acc = atan2(ay, az);
  roll_acc = atan2(-ax, sqrt(ay*ay + az*az));

  // calculate pitch and roll rates from gyroscope
  pitch_gyro = gy * 180/M_PI * dt;
  roll_gyro = gx * 180/M_PI * dt;

  // update pitch and roll angles using complimentary filter
  pitch = pitch * k + pitch_gyro * (1 - k);
  roll = roll * k + roll_gyro * (1 - k);

  // print the pitch and roll angles to the serial monitor
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("\tRoll: ");
  Serial.println(roll);

  // calculate error, integral, and derivative of error
  error = setpoint - pitch;
  integral += error * sample_time;
  derivative = (error - last_error) / sample_time;

  // calculate control signal using PID controller
  output = Kp * error + Ki * integral + Kd * derivative;

  // set motor speeds based on control signal
  if (output > 0) {
    // set motor 1 to forward
    digitalWrite(motor1_dir, HIGH);
    analogWrite(motor1_pwm, output);

    // set motor 2 to reverse
    digitalWrite(motor2_dir, LOW);
    analogWrite(motor2_pwm, -output);
  } else {
    // set motor 1 to reverse
    digitalWrite(motor1_dir, LOW);
    analogWrite(motor1_pwm, -output);

    // set motor 2 to forward
    digitalWrite(motor2_dir, HIGH);
    analogWrite(motor2_pwm, output);
  }
  // update error for next sample
  last_error = error;

  // delay for a small amount of time
  delay(100);
}

void calibrateSensors() {
  // initialize variables for storing sensor data
  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;

  // read sensor data 1000 times and average to reduce noise
  for (int i = 0; i < 1000; i++) {
    // read raw accelerometer data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    ax_sum += Wire.read() << 8 | Wire.read();
    ay_sum += Wire.read() << 8 | Wire.read();
    az_sum += Wire.read() << 8 | Wire.read();

    // read raw gyroscope data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    gx_sum += Wire.read() << 8 | Wire.read();
    gy_sum += Wire.read() << 8 | Wire.read();
    gz_sum += Wire.read() << 8 | Wire.read();
  }

  // calculate accelerometer offsets
  ax_offset = ax_sum / 1000;
  ay_offset = ay_sum / 1000;
  az_offset = az_sum / 1000;

  // calculate gyroscope offsets
  gx_offset = gx_sum / 1000;
  gy_offset = gy_sum / 1000;
  gz_offset = gz_sum / 1000;
}


