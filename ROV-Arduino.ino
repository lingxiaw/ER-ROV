#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 accelgyro;
MPU6050 initialize;
int16_t ax,ay,az;
int16_t gx,gy,gz;
int signal; 

#define Gry_offset 0
#define Gyr_Gain 131
#define Angle_offset 
#define pi 3.14159

byte servoPin1 = 13, servoPin2 =12, servoPin4 = 11, servoPin3 = 9, servoPin5 = 8, servoPin6 = 7;
Servo servo1,servo2,servo3,servo4,servo5,servo6;
float Angle_Delta, Angle_Recursive, Angle_Confidence;
float kp, ki, kd;
float kpr,kir,kdr;
float Angle_Raw, Angle_Filtered, omega, omega_roll, dt;
float Output;
float roll,yaw,pitch,gyro_roll,gyro_pitch,gyro_yaw;
float acc_pitch,acc_roll;
unsigned long preTime, lastTime;
float errSum, dErr, error, lastErr;
int timeChange;
int trusterPower1, trusterPower2;


void setup() {

  Serial.begin(9600);
  Wire.begin();
  accelgyro.initialize();
  for (int i = 0; i < 200; i++) // Looping 200 times to get the real gesture when starting
  {
    Filter();
  }
  omega = Angle_Raw = Angle_Filtered = 0;
  Output = error = errSum = dErr = 0;
  Filter();
  //myPID();
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);
  servo5.attach(servoPin5);
  servo6.attach(servoPin6);
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);
  servo5.writeMicroseconds(1500);
  servo6.writeMicroseconds(1500);// send "stop" signal to ESC.
  delay(1000);
}

void loop() {
 
 if (Serial.available()>0){
  String command=Serial.readString();
  int firstnumber=command.indexOf('a')+1;
  int secondnumber=command.indexOf('b');
  int firstchar=command.indexOf('p')+1;
  int secondchar=command.indexOf('i');
  int thirdchar=command.indexOf('d');
  int balance_indicator=command.indexOf('B');
  int stop_indicator=command.indexOf('s');
  int secondindex=secondchar+1;
  int thirdindex=thirdchar+1;
  
  String value1=command.substring(firstchar, secondchar);
  String value2=command.substring(secondindex,thirdchar);
  String value3=command.substring(thirdindex);
  
  String truster1=command.substring(firstnumber, secondnumber);
  String truster2=command.substring(secondnumber+1);

  
  kp = value1.toFloat();
  ki = value2.toFloat();
  kd = value3.toFloat();
  //Serial.println(kp);
  //Serial.println(ki);
  //Serial.println(kd);

  trusterPower1 = truster1.toInt();
  trusterPower2 = truster2.toInt();
    
  if ((trusterPower1>=1500) &&(trusterPower1<=1900)){
    servo1.writeMicroseconds(trusterPower1);
    servo2.writeMicroseconds(trusterPower1);}

  if (stop_indicator>=0){
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  servo3.writeMicroseconds(1500);
  servo4.writeMicroseconds(1500);
  servo5.writeMicroseconds(1500);
  servo6.writeMicroseconds(1500);
  }
}
  Filter();
  myPID_pitch(kp, ki, kd);
  myPID_roll(kp, ki, kd);
 

 //Serial.println(trusterPower1);
 
}

void Filter()
{ int N=7;
  long int gxtot=0;
  long int gytot=0;
  long int gztot=0;
  long int axtot=0;
  long int aytot=0;
  long int aztot=0;
  int16_t gx_corr=68.8;
  int16_t gy_corr=183-364;
  int16_t gz_corr=13;
  for (int i = 0; i < N; i++)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gxtot=gxtot+gx;
    gytot=gytot+gy;
    gztot=gztot+gz;
    axtot=axtot+ax;
    aytot=aytot+ay;
    aztot=aztot+az;
    }
    gx=gxtot/N;
    gy=gytot/N;
    gz=gztot/N; 
    gx=gx+gx_corr;    
    gy=gy+gy_corr;
    gz=gz+gz_corr;
    ax=axtot/N;         
    ay=aytot/N;
    az=aztot/N;
  az=az+598.5+263;  
  ay=ay+91-158.5;
  ax=ax-186+267;
  float azf=az;  
  float ayf=ay;
  float axf=ax;
  Angle_Raw = (atan2(ay, az) * 180 / pi);
  gx=gx+100;
  omega = gx / Gyr_Gain + Gry_offset;
  // Filter datas to get the real gesture
  unsigned long now = micros();
  timeChange = now - preTime;
  preTime = now;
  dt = timeChange*0.000001;
  Angle_Delta = (Angle_Raw - Angle_Filtered) * 0.64;
  Angle_Recursive = Angle_Delta * dt + Angle_Recursive;
  Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 1.6 + omega;
  Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
  
  acc_roll=(atan2(ax,sqrt(azf*azf+ayf*ayf+0.001))* 180/pi);
  gyro_roll=gyro_roll+(gy/131.0)*(dt*0.000001);
  roll=0.02*gyro_roll+0.98*acc_roll;
  omega_roll = gy / Gyr_Gain + Gry_offset;
  //Serial.println(roll);
  //Serial.print(" ");
  //Serial.println(Angle_Filtered);
  }

  void myPID_pitch(float kp, float ki, float kd)
{
  // Calculating the output values using the gesture values and the PID values.
  error = Angle_Filtered;
  errSum += error;
  dErr = error- lastErr;
  Output= kp * error + ki * errSum + kd * omega;
  lastErr = error;
   if (Output>0)
  {Output= min(255,Output);
   Output= min((Output*(400/255)+1500),1600);
   servo3.writeMicroseconds(Output);
   servo4.writeMicroseconds(Output);
   Output=1500-abs(Output-1500);
   servo5.writeMicroseconds(Output);
   }
   else{
  if (Output<0)
  {Output = max(-255, Output);
  Output = max((Output*(400/255)+1500),1400);
  servo3.writeMicroseconds(Output);
  servo4.writeMicroseconds(Output);
  Output=1500+abs(Output-1500);
  servo5.writeMicroseconds(Output);
  
  }
  }
  Serial.println(Output);
}

void myPID_roll(float kp, float ki, float kd)
{ 
  // Calculating the output values using the gesture values and the PID values.
  error = roll;
  errSum += error;
  dErr = error- lastErr;
  Output= kp * error + ki * errSum + kd * omega_roll;
  lastErr = error;
    if (Output>0)
  {Output= min(255,Output);
   Output= min((Output*(400/255)+1500),1700);
   servo6.writeMicroseconds(Output);
   }
   else{
  if (Output<0)
  {Output = max(-255, Output);
  Output = max((Output*(400/255)+1500),1300);
  servo6.writeMicroseconds(Output);
  }
  }
  //Serial.println(Output);
  }
