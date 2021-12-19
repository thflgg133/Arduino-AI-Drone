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

const int SAMPLE_COUNT = 2800;
const int INPUT_NODES = 6;
const int OUTPUT_NODES = 3;

int16_t (* input)[INPUT_NODES] = NULL;
double (* target)[OUTPUT_NODES] = NULL;

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

  input = (int16_t (*)[INPUT_NODES])malloc(sizeof(int16_t)*INPUT_NODES*SAMPLE_COUNT);
  target = (double (*)[OUTPUT_NODES])malloc(sizeof(double)*OUTPUT_NODES*SAMPLE_COUNT);
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

  extern int data_sampling_en;
  static int i = 0;
  
  if(data_sampling_en == 1 && i<SAMPLE_COUNT) {
    // 입력 데이터 수집
    input[i][0] = AcX; 
    input[i][1] = AcY; 
    input[i][2] = AcZ;   
    input[i][3] = GyX; 
    input[i][4] = GyY; 
    input[i][5] = GyZ; 
    
    // 목표 데이터 수집
    target[i][0] = BalX;// 라벨 시작
    target[i][1] = BalY;
    target[i][2] = BalZ; // 라벨 끝

    i++;
  }

  extern int data_streaming_en;
  if(data_streaming_en == 1) {  
    if(Serial.available()>0) {
      char userInput = Serial.read();
      if(userInput == 'p') {
         // python code
         printf("# python data\n\n");
         printf("import numpy as np\n\n");
         printf("I = np.array([\n");
         int i;
         for(i=0;i<SAMPLE_COUNT-1;i++) {
          printf("\t[[%d,%d,%d,%d,%d,%d]],\n", input[i][0], input[i][1], input[i][2], input[i][3], input[i][4], input[i][5]);
         }         
         printf("\t[[%d,%d,%d,%d,%d,%d]]\n", input[i][0], input[i][1], input[i][2], input[i][3], input[i][4], input[i][5]);
         printf("])\n");
         
         printf("T = np.array([\n");
         for(i=0;i<SAMPLE_COUNT-1;i++) {
          printf("\t[[%f,%f,%f]],\n", target[i][0], target[i][1], target[i][2]);
         }         
         printf("\t[[%f,%f,%f]]\n", target[i][0], target[i][1], target[i][2]);
         printf("])\n");
      } else if(userInput == 'c') {   
         // arduino code
         printf("// c/c++ data\n\n");
         printf("double I[][6] = {\n");
         int i;
         for(i=0;i<SAMPLE_COUNT-1;i++) {
          printf("\t{%d,%d,%d,%d,%d,%d},\n", input[i][0], input[i][1], input[i][2], input[i][3], input[i][4], input[i][5]);
         }         
         printf("\t{%d,%d,%d,%d,%d,%d}\n", input[i][0], input[i][1], input[i][2], input[i][3], input[i][4], input[i][5]);
         printf("};\n");
      }
    }
  }

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
