#include <Wire.h>
#include "Kalman.h"  // 소스:  https://github.com/TKJElectronics/KalmanFilter
#include "SB_PID.h"
#include "SB_Stepper_Manager.h"
#include "Scheduler_SB_Manager.h"


const float x_kp = 0.01, x_ki = 0, x_kd = 0;
const float accX_offset = -1450, accY_offset = 300;

SB_PID pid_x(x_kp,x_ki, x_kd);
SB_PID pid_y(x_kp,x_ki, x_kd);

void x_act(float);

Scheduler_SB PID_X_Func(x_act, 20000);

int button = 2;
int led = 3;
float Input1, Input2, Input3;
const float k = 0.1;

Kalman kalmanX;        // Kalman 인스턴스 생성, X-축
Kalman kalmanY;        // Kalman 인스턴스 생성, Y-축

const uint8_t  IMUAddress = 0x68;    // PCB 상에서 AD0가 LOW (GND) 이면,주소는 0x68
const uint8_t  IMUAddress2 = 0x69;
                                      // AD0가 VDD(+3.3)와 연결되어있으면 주소는 0x69
const uint16_t I2C_TIMEOUT = 1000;    // I2C 통신에서 에러를 확인하기 위함

// IMU data
float G_accX, G_accY;
int16_t    accX, accY, accZ;         // x, y, z 축에 대한 가속도
int16_t    accX2, accY2, accZ2;         // x, y, z 축에 대한 가속도 2번째 mpu6050
int16_t    tempRaw;                  // 온도에 대한 raw 데이터
int16_t    tempRaw2;                  // 온도에 대한 raw 데이터
int16_t    gyroX, gyroY, gyroZ;      // gyro에 대한 값: x,y,z
int16_t    gyroX2, gyroY2, gyroZ2;      // gyro에 대한 값: x,y,z
 
float    accXangle, accYangle;      // 가속도계를 이용하여 각도 계산, X, Y, Z
float    accXangle2, accYangle2;      // 가속도계를 이용하여 각도 계산, X, Y, Z secon mpu6050
float    temp;                      // 온도
float    temp2;                      // 온도
float    gyroXangle, gyroYangle;    // 자이로 를 이용하여 각도 계산
float    gyroXangle2, gyroYangle2;    // 자이로 를 이용하여 각도 계산
float    kalAngleX, kalAngleY;      // Kalman 필터를 이용하여 각도 계산
float    kalAngleX2, kalAngleY2;      // Kalman 필터를 이용하여 각도 계산
float xAndy[2];                     //함수의 각도 저장 배열
float xAndy2[2];                     //함수의 각도 저장 배열
float firstxAngle, firstyAngle;     // 초기 각도 값
float loopx, loopy, loopz;
float calix, caliy;

const float x_k = 0.1;
const float y_k = 0.1;


uint32_t    timer;
uint32_t    timer2;
uint8_t    i2cData[14];        // I2C 데이터를 위한 버퍼: 14-바이트
uint8_t    i2cData2[14];        // I2C 데이터를 위한 버퍼: 14-바이트

float finalx, finaly;

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
               // 기록할 레지스터의 주소, 데이터,       쓰기 후 장치 릴리즈 여부
  return i2cWrite(registerAddress,&data,1,sendStop);     // 성공하면 0 리턴
}                // 레지스터의 주소, 데이터 포인트, 버스 릴리즈 여부(true=release, false=hold)
 
// I2C 장치의 레지스터에 멀티플 바이트 쓰기
uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
        // 기록할 레지스터의 주소, 데이터의 포인터, 데이터 길이, 송신 후 release 여부
  Wire.beginTransmission(IMUAddress);    // I2C 장치의 주소와 통신 시작
  Wire.write(registerAddress);           // 주소 값 쓰기
  Wire.write(data, length);              // 위 주소에 데이터를 length 만큼 기록
  return Wire.endTransmission(sendStop); // 성공하면 0 리턴
}

uint8_t i2cWrite2(uint8_t registerAddress, uint8_t data, bool sendStop) {
               // 기록할 레지스터의 주소, 데이터,       쓰기 후 장치 릴리즈 여부
  return i2cWrite2(registerAddress,&data,1,sendStop);     // 성공하면 0 리턴
}                // 레지스터의 주소, 데이터 포인트, 버스 릴리즈 여부(true=release, false=hold)

uint8_t i2cWrite2(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
        // 기록할 레지스터의 주소, 데이터의 포인터, 데이터 길이, 송신 후 release 여부
  Wire.beginTransmission(IMUAddress2);    // I2C 장치의 주소와 통신 시작
  Wire.write(registerAddress);           // 주소 값 쓰기
  Wire.write(data, length);              // 위 주소에 데이터를 length 만큼 기록
  return Wire.endTransmission(sendStop); // 성공하면 0 리턴
}

 uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
              // 기록할 레지스터의 주소, 데이터의 포인터, 데이터 길이
  uint32_t timeOutTimer;                 // 타임아웃 시간
  Wire.beginTransmission(IMUAddress);    // I2C 장치의 주소와 통신 시작
  Wire.write(registerAddress);           // 주소 값 쓰기
  if(Wire.endTransmission(false))        // 버스를 계속 잡고 있다면
    return 1;                            // 통신 중 에러 발생, 리턴
  Wire.requestFrom(IMUAddress, nbytes,(uint8_t)true); // 반복적으로 데이터 통신 시작하고, 읽기 후 버스를 놓음
  for(uint8_t i = 0; i < nbytes; i++) {               // 데이터 수 만큼 반복
    if(Wire.available())                 // 장치가 사용가능하면
      data[i] = Wire.read();             // 데이터를 읽어서 데이터 버퍼에 저장
    else {    
      timeOutTimer = micros();           // 시간 측정
      while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());    // 타임아웃 시간보다 작거나, 장치가 사용가능하지 않으면 대기
      if(Wire.available())               // 장치가 사용가능하게 되면
        data[i] = Wire.read();           // 데이터를 읽음
      else                               // 사용가능하지 않으면
        return 2;                        // 통신 중 에러 발생
    }
  }
  return 0;                              // 성공
}

 uint8_t i2cRead2(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
              // 기록할 레지스터의 주소, 데이터의 포인터, 데이터 길이
  uint32_t timeOutTimer;                 // 타임아웃 시간
  Wire.beginTransmission(IMUAddress2);    // I2C 장치의 주소와 통신 시작
  Wire.write(registerAddress);           // 주소 값 쓰기
  if(Wire.endTransmission(false))        // 버스를 계속 잡고 있다면
    return 1;                            // 통신 중 에러 발생, 리턴
  Wire.requestFrom(IMUAddress2, nbytes,(uint8_t)true); // 반복적으로 데이터 통신 시작하고, 읽기 후 버스를 놓음
  for(uint8_t i = 0; i < nbytes; i++) {               // 데이터 수 만큼 반복
    if(Wire.available())                 // 장치가 사용가능하면
      data[i] = Wire.read();             // 데이터를 읽어서 데이터 버퍼에 저장
    else {    
      timeOutTimer = micros();           // 시간 측정
      while(((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());    // 타임아웃 시간보다 작거나, 장치가 사용가능하지 않으면 대기
      if(Wire.available())               // 장치가 사용가능하게 되면
        data[i] = Wire.read();           // 데이터를 읽음
      else                               // 사용가능하지 않으면
        return 2;                        // 통신 중 에러 발생
    }
  }
  return 0;                              // 성공
}

// 스텝 모터


SB_Stepper_indi S3(A7, A0, A1, A2, A3, false);
SB_Stepper_indi S2(A7, 8, 9, 10, 11, false);
SB_Stepper_indi S1(A7, 4, 5, 6, 7, true);


void setup()
{
    Serial.begin(9600);        // 시리얼 통신, 9600bps
    Wire.begin();              // I2C 초기화

     pinMode(button,INPUT);
    pinMode(led,OUTPUT);

    
    i2cData[0] = 7;       // 샘플링 속도를 1000Hz - 8Hz/(7+1) = 1000 Hz 로 설정
    i2cData[1] = 0x00;    // FSYNC를 정지, Acc 필터: 260Hz, Gyro 필터: 256Hz, 샘플링 8KHz
    i2cData[2] = 0x00;    // Gyro를 풀 스케일 범위 ±250도/s 로 설정
    i2cData[3] = 0x00;    // 가속도계를 풀 스케일 범위  ±2g 로 설정
    while(i2cWrite(0x19, i2cData, 4, false));    // 4개의 레지스터에 동시에 기록
                                                // 0x19 = sampling rate 설정 레지스터
                                                // 0x1A = config 레지스터
                                                // 0x1B = Gyro config 레지스터
                                                // 0x1C = Accel config 레지스터
    while(i2cWrite(0x6B, 0x01, true));   // PLL을 X축 gyro 레퍼런트로, 슬리모드 해제
                                         // 0x6B = PWR_MGMT_1 레지스터
    while(i2cRead(0x75, i2cData, 1));    // 0x75 = WHO_AM_I 레지스터
    if(i2cData[0] != 0x68)              // 주소값이 0x68이 아니면
    {
        Serial.print(F("Error reading sensor"));    // 주소가 0x68이면, AD0가 GND 된 상태
        while(1);                                     // AD0가 VLOGIC(+3.3V) 이면 0x69
    }
 
    delay(100);        // 센서가 안정화 되기를 기다림

    // kalman과 gyro의 시작 각도 설정
    while(i2cRead(0x3B, i2cData, 6));          // 0x3B 는 저장된 측정 값의 시작 레지스터, HIGH -> LOW
    accX = ((i2cData[0] <<8) | i2cData[1]);    // Accel X-축 값: HIGH | LOW
    accY = ((i2cData[2] <<8) | i2cData[3]);    // Accel Y-축 값: HIGH | LOW
    accZ = ((i2cData[4] <<8) | i2cData[5]);    // Accel Z
 
    // atan2 는 -π~  π (라디안)  값을 출력 : 참고:  http://en.wikipedia.org/wiki/Atan2
    // 여기서는 이것을 0 ~ 2π로 변환하고,  라디안에서 각도로 바꾼다.
    // atan2(높이, 밑변) => 각도(라디안)
    accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG; // roll, Y축으로 회전
    accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG; // pitch, X축으로 회전
    // accZangle = (atan2(accZ, accY) + PI) * RAD_T)_DEG; // yaw ??
    
    kalmanX.setAngle(accXangle);    // 시작 각도 설정
    kalmanY.setAngle(accYangle);    //
     
    timer = micros();
       
    S1.Set_Step_Per_Unit(11.3777);
    S2.Set_Step_Per_Unit(11.3777);
    S3.Set_Step_Per_Unit(11.3777);

    SB_Stepper_Manager::add(&S1);
    SB_Stepper_Manager::add(&S2);
    SB_Stepper_Manager::add(&S3);

    SB_Stepper_Manager::init();

    S1.Set_Sps(50,800);
    S2.Set_Sps(50,800);
    S3.Set_Sps(50,800);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    
    S1.MoveTo(-60,60);
    S2.MoveTo(-60,60);
    S3.MoveTo(-60,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);

    S1.MoveTo(-60,60);
    S2.MoveTo(-60,60);
    S3.MoveTo(-60,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);


    S1.MoveTo(-60,60);
    S2.MoveTo(-60,60);
    S3.MoveTo(-60,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);

    S1.MoveTo(40,60);
    S2.MoveTo(40,60);
    S3.MoveTo(40,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);
    
    S1.MoveTo(30,60);
    S2.MoveTo(30,60);
    S3.MoveTo(30,60);  

    delay(1500);
    
    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);
    delay(100);
//
//    for(int i = 0; i < 40; i++) {
//     anglefind();
//     }
//     
//    for(int i = 0; i < 50; i++) {
//      anglefind();
//      firstxAngle += xAndy[0];
//      firstyAngle += xAndy[1];
//    }
//    firstxAngle = firstxAngle / 50;
//    firstyAngle = firstyAngle / 50;


    Scheduler_SB_Manager::add(PID_X_Func);
}



void loop()
{
   Scheduler_SB_Manager::idle();
   if(digitalRead(button) == HIGH) {
    while(digitalRead(button) == HIGH) {
      
      delay(100);
      
    }
    calibration();
   }
   

}



void anglefind() {
   // 모든 데이터를 업데이트
   
    while(i2cRead(0x3B, i2cData, 14));              // 0x3B 주소에는 데이터가 있는 레지스터, 14바이트를 읽는다
    accX = ((i2cData[0] << 8) | i2cData[1]) + accX_offset;        // x축 가속도 값, Hi, Lo 바이트 값을 조합
    accY = ((i2cData[2] << 8) | i2cData[3]) + accY_offset;        // y
    accZ = ((i2cData[4] << 8) | i2cData[5]);        // z
    tempRaw = ((i2cData[6] << 8) | i2cData[7]);     // 온도 값
    gyroX = ((i2cData[8] << 8) | i2cData[9]);       // x축 자이로 값
    gyroY = ((i2cData[10] << 8) | i2cData[11]);     // y축 자이로 값
    gyroZ = ((i2cData[12] << 8) | i2cData[13]);     // z축
 
    // atan2는 각도 계산 (atan2(높이, 밑변))으로 -π ~ π 의 라디안 값을 반환한다.
    // 그리고 값을 각도로 바꾼다. http://en.wikipedia.org/wiki/Atan2
//    accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;   // pitch
//    accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;   // roll
// 
//    float gyroXrate = (float)gyroX/131.0;    // gyro의 X축 각도 변화량
//    float gyroYrate = (float)gyroY/131.0;    // Y축
// 
//    //gyroXangle += gyroXrate*((float)(micros()-timer)/1000000); // 필터 보정 없이 gyro 각도 계산
//    //gyroYangle += gyroYrate*((float)(micros()-timer)/1000000);
// 
//    gyroXangle += kalmanX.getRate()*((float)(micros()-timer)/1000000); // unbiased rate 를 이용하여 gyro 각도 계산
//    gyroYangle += kalmanY.getRate()*((float)(micros()-timer)/1000000);
//
//    
//    // Kalman 필터로 각도 계산
//    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (float)(micros()-timer)/1000000); 
//    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (float)(micros()-timer)/1000000);
//    timer = micros();
// 
//    temp = ((float)tempRaw + 12412.0) / 340.0;
 
    // 배열에 x축 값, y축 값 저장
    G_accX = accX*k + G_accX*(1 - k);
    G_accY = accY*k + G_accY*(1 - k);

//    finalx = firstxAngle - xAndy[0];
//    finaly = firstyAngle - xAndy[1];
    
}



//void calibration() {
//    i2cData2[0] = 7;       // 샘플링 속도를 1000Hz - 8Hz/(7+1) = 1000 Hz 로 설정
//    i2cData2[1] = 0x00;    // FSYNC를 정지, Acc 필터: 260Hz, Gyro 필터: 256Hz, 샘플링 8KHz
//    i2cData2[2] = 0x00;    // Gyro를 풀 스케일 범위 ±250도/s 로 설정
//    i2cData2[3] = 0x00;    // 가속도계를 풀 스케일 범위  ±2g 로 설정
//    
//    while(i2cWrite2(0x19, i2cData2, 4, false));    // 4개의 레지스터에 동시에 기록
//                                                // 0x19 = sampling rate 설정 레지스터
//                                                // 0x1A = config 레지스터
//                                                // 0x1B = Gyro config 레지스터
//                                                // 0x1C = Accel config 레지스터
//    while(i2cWrite2(0x6B, 0x01, true));   // PLL을 X축 gyro 레퍼런트로, 슬리모드 해제
//                                         // 0x6B = PWR_MGMT_1 레지스터
//    while(i2cRead2(0x75, i2cData2, 1));    // 0x75 = WHO_AM_I 레지스터
//    if(i2cData2[0] != 0x68)              // 주소값이 0x69이 아니면
//    {
//        Serial.print(F("Error reading sensor"));    // 주소가 0x68이면, AD0가 GND 된 상태
//        while(1);                                     // AD0가 VLOGIC(+3.3V) 이면 0x69
//    }
// 
//    delay(100);        // 센서가 안정화 되기를 기다림
//
//    // kalman과 gyro의 시작 각도 설정
//    while(i2cRead2(0x3B, i2cData2, 6));          // 0x3B 는 저장된 측정 값의 시작 레지스터, HIGH -> LOW
//    accX2 = ((i2cData2[0] <<8) | i2cData2[1]);    // Accel X-축 값: HIGH | LOW
//    accY2 = ((i2cData2[2] <<8) | i2cData2[3]);    // Accel Y-축 값: HIGH | LOW
//    accZ2 = ((i2cData2[4] <<8) | i2cData2[5]);    // Accel Z
// 
//    // atan2 는 -π~  π (라디안)  값을 출력 : 참고:  http://en.wikipedia.org/wiki/Atan2
//    // 여기서는 이것을 0 ~ 2π로 변환하고,  라디안에서 각도로 바꾼다.
//    // atan2(높이, 밑변) => 각도(라디안)
//    accYangle2 = (atan2(accX2, accZ2) + PI) * RAD_TO_DEG; // roll, Y축으로 회전
//    accXangle2 = (atan2(accY2, accZ2) + PI) * RAD_TO_DEG; // pitch, X축으로 회전
//    // accZangle = (atan2(accZ, accY) + PI) * RAD_T)_DEG; // yaw ??
//    
//    kalmanX.setAngle(accXangle2);    // 시작 각도 설정
//    kalmanY.setAngle(accYangle2);    //
//     
//    timer2 = micros();
//    
//    for(int i = 0; i < 40; i++) {
//     anglefind2();
//     }
//     
//    for(int i = 0; i < 50; i++) {
//      anglefind2();
//      calix += xAndy2[0];
//      caliy += xAndy2[1];
//    }
//    calix = calix / 50;
//    caliy = caliy / 50;
//    
//}    

unsigned long t;
void x_act(float k)
{
  //t = micros();
  anglefind();

  Serial.print(G_accX);
  Serial.print(" ");
  Serial.println(G_accY);
  
  float loopy = -pid_y.Output(0,G_accY);
  float loopx = pid_x.Output(0,G_accX);
  loopy = loopy*0.1;
  loopx = loopx*0.1;
  Input2 = loopy - loopx;
  Input3 = loopy + loopx;
  S2.MoveTo(Input2 + S2.Get_Step(),70);
  S3.MoveTo(Input3 + S3.Get_Step(),70);
  anglefind();
  Input1 = -loopy;
  S1.MoveTo(Input1 + S1.Get_Step(),70);
  //Serial.println(micros() - t);
}

void calibration() {
   S1.MoveTo(-60,60);
    S2.MoveTo(-60,60);
    S3.MoveTo(-60,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);

    S1.MoveTo(-60,60);
    S2.MoveTo(-60,60);
    S3.MoveTo(-60,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);


    S1.MoveTo(-60,60);
    S2.MoveTo(-60,60);
    S3.MoveTo(-60,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);

    S1.MoveTo(40,60);
    S2.MoveTo(40,60);
    S3.MoveTo(40,60);

    delay(1500);

    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);

    delay(100);
    
    S1.MoveTo(30,60);
    S2.MoveTo(30,60);
    S3.MoveTo(30,60);  

    delay(1500);
    
    S1.Set_Origin(0);
    S2.Set_Origin(0);
    S3.Set_Origin(0);
    delay(100);
}
