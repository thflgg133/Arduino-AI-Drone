#include <Wire.h>

const int MOTOR_A =23;
const int MOTOR_B =19;
const int MOTOR_C =18; 
const int MOTOR_D =26;
const int CHANNEL_A =0;
const int CHANNEL_B =1;
const int CHANNEL_C =2; 
const int CHANNEL_D =3;
const int MOTOR_FREQ =5000;
const int MOTOR_RESOLUTION =10;

void drone_setup() {

  Wire.begin();
  Wire.setClock(400000);

  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0x0);
  Wire.endTransmission(true);
  
  ledcAttachPin(MOTOR_A, CHANNEL_A);
  ledcAttachPin(MOTOR_B, CHANNEL_B);
  ledcAttachPin(MOTOR_C, CHANNEL_C);
  ledcAttachPin(MOTOR_D, CHANNEL_D);

  ledcSetup(CHANNEL_A, MOTOR_FREQ, MOTOR_RESOLUTION);
  ledcSetup(CHANNEL_B, MOTOR_FREQ, MOTOR_RESOLUTION);
  ledcSetup(CHANNEL_C, MOTOR_FREQ, MOTOR_RESOLUTION);
  ledcSetup(CHANNEL_D, MOTOR_FREQ, MOTOR_RESOLUTION);

  ledcWrite(CHANNEL_A, 0);
  ledcWrite(CHANNEL_B, 0);
  ledcWrite(CHANNEL_C, 0);
  ledcWrite(CHANNEL_D, 0);
}

int throttle = 0;
double tAngleX = 0.0, tAngleY = 0.0, tAngleZ = 0.0;
void drone_loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)0x68,(uint8_t)14,true);

  int16_t AcXH = Wire.read();  
  int16_t AcXL = Wire.read();
  int16_t AcYH = Wire.read();  
  int16_t AcYL = Wire.read();
  int16_t AcZH = Wire.read();  
  int16_t AcZL = Wire.read(); 
  int16_t TmpH = Wire.read();  
  int16_t TmpL = Wire.read();  
  int16_t GyXH = Wire.read();  
  int16_t GyXL = Wire.read();
  int16_t GyYH = Wire.read();  
  int16_t GyYL = Wire.read();
  int16_t GyZH = Wire.read();  
  int16_t GyZL = Wire.read();

  int16_t AcX = AcXH <<8 |AcXL;
  int16_t AcY = AcYH <<8 |AcYL;
  int16_t AcZ = AcZH <<8 |AcZL;
  int16_t GyX = GyXH <<8 |GyXL;
  int16_t GyY = GyYH <<8 |GyYL;
  int16_t GyZ = GyZH <<8 |GyZL;

  static int32_t AcXSum =0, AcYSum =0, AcZSum =0;
  static int32_t GyXSum =0, GyYSum =0, GyZSum =0;
  static double AcXOff =0.0, AcYOff =0.0, AcZOff =0.0;
  static double GyXOff =0.0, GyYOff =0.0, GyZOff =0.0;
  static int cnt_sample =1000;
  if(cnt_sample >0) {
    AcXSum += AcX; AcYSum += AcY; AcZSum += AcZ;
    GyXSum += GyX; GyYSum += GyY; GyZSum += GyZ;
    cnt_sample --;
    if(cnt_sample ==0) {       
      AcXOff = AcXSum /1000.0;      
      AcYOff = AcYSum /1000.0;      
      AcZOff = AcZSum /1000.0;  
      GyXOff = GyXSum /1000.0;      
      GyYOff = GyYSum /1000.0;      
      GyZOff = GyZSum /1000.0;
    }
    delay(1);
    return;    
  }

  double AcXD = AcX - AcXOff;
  double AcYD = AcY - AcYOff;
  double AcZD = AcZ - AcZOff +16384;

  double GyXD = GyX - GyXOff;
  double GyYD = GyY - GyYOff;
  double GyZD = GyZ - GyZOff;

  static unsigned long t_prev =0;
  unsigned long t_now = micros();
  double dt = (t_now - t_prev)/1000000.0;
  t_prev = t_now;

  const float GYROXYZ_TO_DEGREES_PER_SEC =131;
  double GyXR = GyXD /GYROXYZ_TO_DEGREES_PER_SEC;
  double GyYR = GyYD /GYROXYZ_TO_DEGREES_PER_SEC;
  double GyZR = GyZD /GYROXYZ_TO_DEGREES_PER_SEC;

  static double gyAngleX =0.0, gyAngleY =0.0, gyAngleZ =0.0;
  gyAngleX += GyXR *dt;
  gyAngleY += GyYR *dt;
  gyAngleZ += GyZR *dt;

  const float RADIANS_TO_DEGREES =180 /3.14159;
  double AcYZD = sqrt(pow(AcY,2) + pow(AcZ,2));
  double AcXZD = sqrt(pow(AcX,2) + pow(AcZ,2));
  double acAngleY = atan(-AcXD /AcYZD)*RADIANS_TO_DEGREES;
  double acAngleX = atan(AcYD /AcXZD)*RADIANS_TO_DEGREES;
  double acAngleZ =0;

  const double ALPHA =0.96;  
  static double cmAngleX =0.0, cmAngleY =0.0, cmAngleZ =0.0; 
  cmAngleX=ALPHA*(cmAngleX+GyXR*dt)+(1.0-ALPHA)*acAngleX;
  cmAngleY=ALPHA*(cmAngleY+GyYR*dt)+(1.0-ALPHA)*acAngleY;
  cmAngleZ = gyAngleZ;
  if(throttle == 0) cmAngleX = cmAngleY = cmAngleZ = 0.0;

  double eAngleX = tAngleX - cmAngleX;
  double eAngleY = tAngleY - cmAngleY;
  double eAngleZ = tAngleZ - cmAngleZ;

  double Kp = 1.0;
  double BalX = Kp * eAngleX;
  double BalY = Kp * eAngleY;
  double BalZ = Kp * eAngleZ;

  double Kd = 1.0;
  BalX += Kd *-GyXR;
  BalY += Kd *-GyYR;
  BalZ += Kd *-GyZR;
  if(throttle == 0) BalX = BalY = BalZ = 0.0;

  double Ki = 1.0;
  double ResX = 0.0, ResY =0.0, ResZ =0.0;
  ResX += Ki * eAngleX * dt;
  ResY += Ki * eAngleY * dt;
  ResZ += Ki * eAngleZ * dt;
  if(throttle == 0) ResX = ResY = ResZ = 0.0;
  BalX += ResX;
  BalY += ResY;
  BalZ += ResZ;

  double speedA = throttle + BalX - BalY + BalZ;  
  double speedB = throttle - BalX - BalY - BalZ;  
  double speedC = throttle - BalX + BalY + BalZ;
  double speedD = throttle + BalX + BalY - BalZ;

  int iSpeedA = constrain((int)speedA, 0, 1000);
  int iSpeedB = constrain((int)speedB, 0, 1000);
  int iSpeedC = constrain((int)speedC, 0, 1000);
  int iSpeedD = constrain((int)speedD, 0, 1000);

  ledcWrite(CHANNEL_A, iSpeedA);
  ledcWrite(CHANNEL_B, iSpeedB);
  ledcWrite(CHANNEL_C, iSpeedC);
  ledcWrite(CHANNEL_D, iSpeedD);

}
