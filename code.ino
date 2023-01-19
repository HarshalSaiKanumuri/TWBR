#include <PID_v1.h>
#include <Wire.h>

#define Sample 100;
#define Rad2Deg  180 / M_PI;
float Alpha =  0.95;
float Roll_r, Pitch_r, Yaw_r;
float AccX, AccY, AccZ,GyroX,GyroZ,GyroY;
float AngleRoll, AnglePitch, AngleSide ;
float LoopTimer;
float offset_z_ACC , offset_y_ACC , offset_x_ACC , offset_x_GYR , offset_y_GYR , offset_z_GYR ;

#define enA 5   // EnableA command line - should be a PWM pin
#define enB 6   // EnableB command line - should be a PWM pin

// name the motor control pins - replace the ** with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa A0  // Channel A direction 
#define INb A1  // Channel A direction 
#define INc A2  // Channel B direction 
#define INd A3  // Channel B direction 


double Setpoint ; // will be the desired value
double Input; // photo sensor
double Output ; //motor speed
//PID parameters
double Kp=2.0, Ki=1.25, Kd=0; 

void acc() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // Register for low-passfilter
  Wire.write(0x05);  // Bandwidth 10Hz
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // register for adjusting scale
  Wire.write(0x10); // +/- 16g Full Scale Range
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  AccX=(AccXLSB/4096) - offset_x_ACC;
  AccY=(AccYLSB/4096) - offset_y_ACC;
  AccZ=(AccZLSB/4096) - offset_z_ACC;
  AngleRoll = atan(AccY/sqrt(AccX*AccX+AccZ*AccZ)) * Rad2Deg;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ)) * Rad2Deg;
}

void gyro(void){

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // Register for low-passfilter
  Wire.write(0x05);  // Bandwidth 10Hz
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Registor for sensitivity
  Wire.write(0x02); // +/- 1000 degrees per second ( Important for Integration )
  Wire.endTransmission();          

  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // first register of Gyroscope
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  Roll_r=GyroX/16.4; // LSB Sensitivity 65.5 LSB/*/s
  Pitch_r=GyroY/16.4;
  Yaw_r=GyroZ/16.4;
}

void offset_Calc(void){
  // Average a number of gyroscope readings to determine the offset
  for (int i = 0; i < 100; i++)
  {
    offset_x_ACC += AccX;
    offset_y_ACC += AccY;
    offset_z_ACC += AccZ;

    offset_x_GYR += GyroX;
    offset_y_GYR += GyroY;
    offset_z_GYR += GyroZ;
  }
  offset_x_ACC /= Sample;
  offset_y_ACC /= Sample;
  offset_z_ACC /= Sample;

  offset_x_GYR /= Sample;
  offset_y_GYR /= Sample;
  offset_z_GYR /= Sample;
   
}


void runMotors(int leftMotor_speed, int rightMotor_speed){
  // limit the speed value between -255 and 255 as the PWM value can only be between 0 and 255 - the negative is handled below
  leftMotor_speed = constrain(leftMotor_speed, -255, 255);
  rightMotor_speed = constrain(rightMotor_speed, -255, 255);

  // vary the motor speeds - use the absolute value to remove the negative
  analogWrite(enA, abs(leftMotor_speed));
  analogWrite(enB, abs(rightMotor_speed));

  // if the speed value is negative, run the motor backwards
  if (leftMotor_speed < 0) {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
  // else, run the motor forwards
  else {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);    
  }

  // if the speed value is negative, run the motor backwards
  if (rightMotor_speed < 0) {
    digitalWrite(INc, LOW);
    digitalWrite(INd, HIGH);
  }
  // else run the motor forwards
  else {
    digitalWrite(INc, HIGH);
    digitalWrite(INd, LOW);    
  }
}

//create PID instance 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B); // Turn on MPU
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);  // for getting ACC values for offset
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();  

  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // first register of Gyroscope
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();

  offset_Calc();    
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Setpoint = 0;
  
  myPID.SetOutputLimits(-255,255);
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp,Ki,Kd);
  

  // configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

}
void loop() {
  gyro();
  acc();
  delay(40);

  Serial.print("Pitch Angle : ");
  Serial.print(AnglePitch);
  Serial.print(" Roll Angle : ");
  Serial.print(AngleRoll);
  Serial.print("Angle Side : ");
  AngleSide = AngleSide + GyroZ;
  Serial.println(AngleSide);


  //read the value from the MPU-6050 kalman filter output
  Input = map(AnglePitch,90,-90,-255,255);
  myPID.Compute();
  Serial.print("\nThis is the motor speeds : ");
  

  Serial.print(Output);
  
  runMotors(Output, Output);


}
