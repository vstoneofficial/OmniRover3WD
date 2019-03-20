#include <vs-rc202.h>
#include <Arduino.h>
#include <FS.h>
#include <math.h>

#define GO 0
#define LEFT 1
#define RIGHT 2
#define BACK 3
#define STOP 4
#define FUNC1 5
#define FUNC2 6

#define INTERVAL  40    //moveOmuni3での移動時の制御周期

int motion_time =  0;  //モーション再生時間

//サーボモータ回転速度補正係数
double alpha = -1.33096 * pow(10,-6);
double beta = -6.94394 * pow(10,-5);
double gam = 1.263727114;
double delta = -0.6045050505;

//角度->ラジアン変換
double deg2rad(int deg){
  double rad;
  rad = deg * 3.14 /180;
  
  return rad;
}


//速度(velocity -600～600)、移動方向[°](axis 0～360)、旋回量(omega -600～600)から各サーボの回転速度を算出する
void moveOmuni3(int velocity, int axis, int omega){
  int v1, v2, v3;
  double vx, vy, rad;
  
  rad = deg2rad(axis);
  vy = velocity*cos(rad);
  vx = velocity*sin(rad);

  v1 = vx+omega;
  v1 =  alpha * pow(v1,3) + beta * pow(v1,2) + gam * v1 + delta;  //サーボモータ回転速度補正
  v2 = -0.5*vx + 0.865*vy + omega;
  v2 =  alpha * pow(v2,3) + beta * pow(v2,2) + gam * v2 + delta;  //サーボモータ回転速度補正
  v3 = -0.5*vx - 0.865*vy + omega;
  v3 =  alpha * pow(v3,3) + beta * pow(v3,2) + gam * v3 + delta;  //サーボモータ回転速度補正
  
  setServoDeg(1, v1);
  setServoDeg(2, v2);
  setServoDeg(3, v3);
  setServoMovingTime(INTERVAL); //Set moving time
  motion_time = 0;
  moveServo();

  return;
}

void selectMotion(){
  switch(getMotionNumber()){
    case GO:
      moveOmuni3(600,0,0);
      break;
    case LEFT:
      moveOmuni3(600,90,0);
      break;
    case RIGHT:
      moveOmuni3(600,270,0);
      break;
    case BACK:
      moveOmuni3(600,180,0);
      break;
    case STOP:
      moveOmuni3(0,0,0);
      break;
   case FUNC1:
      moveOmuni3(0,0,-600);
      break;
   case FUNC2:
      moveOmuni3(0,0,600);
      break;
  }
}

void setup() {
  //Init robot lib
  initLib();
  delay(10);

  //SV1-3 servo mode
  servoEnable(1, 1);        //Enable SV1 PWM
  servoEnable(2, 1);        //Enable SV2 PWM
  servoEnable(3, 1);        //Enable SV3 PWM

  //SV4-7 LED mode
  setLedMode(4, 1);         //Set SV4 LED mode
  setLedMode(5, 1);         //Set SV5 LED mode
  setLedMode(6, 1);         //Set SV6 LED mode
  setLedMode(7, 1);         //Set SV7 LED mode

  //Offset
  setServoOffset(1,0);   //Offset range:-500 to 500
  setServoOffset(2,0);   //モータに合わせて設定してください
  setServoOffset(3,0);

  delay(2000);
  
}

void loop() {
  int motion_num=0;
  for(motion_num=0;motion_num<5;motion_num++){
    setMotionNumber(motion_num);
    selectMotion();
    delay(2000);
  }
}
