#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <math.h>
#include "HC_SR04.h"
//#include <stdio.h>

#define Rear_TRIG_PIN 37
#define Rear_ECHO_PIN 26
#define Rear_ECHO_INT 27

#define Right_TRIG_PIN 36
#define Right_ECHO_PIN 28
#define Right_ECHO_INT 29

#define Encoder_Y_INT 22
#define Encoder_Y_PIN 23
#define Encoder_X_INT 30
#define Encoder_X_PIN 31

#define LeftHand      34
#define Shooter       38
#define ManualLoad    40
#define servo         9

#define CornerIR      44

#define ServoBallPick   2450
#define ServoBallReload 750

#define T1_button         45
#define T2_button         41
#define T3_button         39

HC_SR04 Right_Sensor(Right_TRIG_PIN, Right_ECHO_PIN, Right_ECHO_INT);
HC_SR05 Rear_Sensor(Rear_TRIG_PIN, Rear_ECHO_PIN, Rear_ECHO_INT);

#define Buzzer 46

#define AllShink  0

#define T1        1
#define T2        2

Adafruit_BNO055 bno = Adafruit_BNO055(55);

bool GripperFlag = false;
int rx_1_pointer = 0;

bool rx_0_complete = false;
bool rx_1_complete = false;

String rx_0_buffer = "";
String rx_1_buffer = "";


int rx_2_pointer = 0;
bool rx_2_complete = false;
String rx_2_buffer = "";
unsigned int Range_1 = 0;
unsigned int Current_Range_1 = 0;
uint8_t check_sum_2 = 0;

String RelayStatus = "00000000";
int32_t Encoder_X_Val = 10000;
int32_t Encoder_Y_Val = 10000;
float Encoder_Y_Dis = 0;
float Encoder_X_Dis = 0;
float X_SetPoint = 0;
float Y_SetPoint = 0;

float GoalDistance = 0;

float RotationSetPoint = 0;
float RotationError = 0;
float LastRotationError = 0;
float RotationDerative = 0;
float RotationVelocity = 0;
float CurrentRotation = 0;
float HeadingDirectionSetPoint = 270;
float HeadingDirection = 270;
float SpeedMission = 0;
bool BNO_Reset = true;

int32_t Rear_Range = 0;
int32_t Right_Range = 0;
int32_t Rear_Range_SetPoint = 30;
int32_t Rear_Range_Error = 0;
int32_t Rear_Range_LastError = 0;
int32_t Rear_Range_Derative = 0;

int32_t Goal_Error = 0;
int32_t Goal_LastError = 0;
int32_t Goal_Derative = 0;

int32_t Heading_Range = 0;

uint8_t MISSION_STATE = 0;

void RotationErrorCalc(void);
void DirectionCalc(void);
void SendMissionToMotorControl(void);
void BNO_Calibration(void);
void CheckRange(void);
void SpeedPID();
void Move(float _X_SetPoint, float _Y_SetPoint, uint8_t _maxSpeed, float _range, float _rotation);
void ResetSpeedPID(void);
void ResetRotationPID(void);
void EncoderDisCalc(void);
void Shoot(uint8_t pole);
void serialFlush2(void);
void wait(uint8_t time_ms);
void RotationCorrect(float rot);
void RotationCorrect1(float rot);
void GoldenShoot(uint8_t pole);
void GoldenMove(float _X_SetPoint, float _Y_SetPoint, uint8_t _maxSpeed, float _range, float _rotation);
void GoldenNavigationCalc(void);
void SideSensorsSetup(void);
void BallShoot(void);
void T1_T2_Shoot(uint8_t zone);
void GoldenPush(void);
void GoldenShoot_test(uint8_t zone);
void Golden_Again_Shoot(void);
void PickHand(void);
void T1_zone(void);
void T2_zone(void);
void T3_zone(void);
void T2_tactic(void);
void T3_tactic(void);
Servo ArmServo;
sensors_event_t event;
void setup()
{
  Serial1.begin(19200);
  Serial.begin(19200);
  Serial2.begin(115200);
  ArmServo.attach(servo);
  ArmServo.writeMicroseconds(1100);
  pinMode( Buzzer, OUTPUT);
  pinMode( Encoder_X_INT, INPUT_PULLUP);
  pinMode( Encoder_X_PIN, INPUT_PULLUP);
  pinMode( Encoder_Y_INT, INPUT_PULLUP);
  pinMode( Encoder_Y_PIN, INPUT_PULLUP);

  pinMode( LeftHand,   INPUT_PULLUP);
  pinMode( Shooter,    INPUT_PULLUP);
  pinMode( ManualLoad, INPUT_PULLUP);
  pinMode( CornerIR,   INPUT_PULLUP);

  pinMode( T1_button,  INPUT_PULLUP);
  pinMode( T2_button,  INPUT_PULLUP);
  pinMode( T3_button,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder_X_INT), Encoder_X, FALLING);
  attachInterrupt(digitalPinToInterrupt(Encoder_Y_INT), Encoder_Y, FALLING);
  digitalWrite(Buzzer, LOW);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  Rear_Sensor.begin5();

  Right_Sensor.begin();
  delay(100);
  Right_Sensor.start();

  Rear_Sensor.start5();
  digitalWrite(Buzzer, HIGH);
  /* Initialise the sensor */
  delay(1000);
  
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    {
      Serial.println("Orientation Sensor Test Retry....");
    }
  }
  digitalWrite(Buzzer, LOW);
  bno.setExtCrystalUse(true);
  BNO_Calibration();
  GripperFlag = false; 
}

void loop()
{
 bool t1 = true; 
 bool t2 = true; 
 bool t3 = true;
 while(t1 & t2 & t3){
  t1 = digitalRead(T1_button);
  t2 = digitalRead(T2_button);
  t3 = digitalRead(T3_button);
  }
  if(!t1 & t2 & t3){
    T1_zone();
  }
  else if(t1 & !t2 & t3){
    T2_tactic();
  }
  else if(t1 & t2 & !t3){
    T3_tactic();
  }
 Serial.println("_____________________________________________________________________________________________");
 while (1)
    {
      RotationErrorCalc();
      SpeedMission = 0;
      SendMissionToMotorControl();
      delay(100);
    }
}
void T1_zone(void){
     Move(420, 100, 140,   30,   0);
    Move(410, 220,  60,   10,   0);
    T1_T2_Shoot(T1);
    T2_zone();
}
void T2_zone(void){
    GoldenMove(590, 360,  50,  20,   0);
    T1_T2_Shoot(T2);
    Move(520, 20,    90,  30,   0);
    Move(610, 150,   70,   80,   0);
    Move(590, 220,   50,   20,   0);
    SideSensorsSetup();
    T3_zone();
}
void T3_zone(void){
  digitalWrite(Buzzer, HIGH);
    GoldenMove(600, 640,  100,  40,   0);
    GoldenMove(595, 660,  100,  20,   0);
    digitalWrite(Buzzer, LOW);
    RotationSetPoint = 0;
    GoldenPush();
    GoldenShoot_test(1);
    GoldenMove(630, 220,  130,  10,   0);
    Golden_Again_Shoot();
    T3_zone();
}
void T2_tactic(void){
    Move(520, 20,    90,  30,   0);
    Move(610, 150,   70,   80,   0);
    Move(590, 220,   50,   20,   0);
    T1_T2_Shoot(T2);
    SideSensorsSetup();
    T3_zone();
}
void T3_tactic(void){
    Move(520, 20,    90,  30,   0);
    Move(610, 150,   70,   80,   0);
    Move(590, 220,   50,   20,   0);
    SideSensorsSetup();
    GoldenMove(590, 360,  50,  20,   0);
    digitalWrite(Buzzer, HIGH);
    GoldenMove(600, 640,  100,  40,   0);
    GoldenMove(595, 660,  100,  20,   0);
    digitalWrite(Buzzer, LOW);
    RotationSetPoint = 0;
    GoldenPush();
    GoldenShoot_test(1);
    GoldenMove(630, 220,  130,  10,   0);
    Golden_Again_Shoot();
    T3_zone();
}
void RotationErrorCalc(void)
{
  bno.getEvent(&event);
  CurrentRotation = event.orientation.x;
 // if (abs(tempCurrentRotation - CurrentRotation) > 3){;}
 // else CurrentRotation = tempCurrentRotation;
  if (CurrentRotation == 360) CurrentRotation = 0;
  if (RotationSetPoint < CurrentRotation)
  {
    RotationError = RotationSetPoint - CurrentRotation;
    if (RotationError < (-180))
    {
      RotationError = (360 + RotationError);
      RotationDerative = 6 * (RotationError - LastRotationError);
      LastRotationError = RotationError;
      RotationVelocity = (4 * RotationError + RotationDerative) / 5;
      
    }
    else  
    {
      RotationError *= (-1);
      RotationDerative = 6 * (RotationError - LastRotationError);
      LastRotationError = RotationError;
      RotationVelocity = ((4 * RotationError + RotationDerative) / 5) * (-1);
    }
      
  }
  else if (RotationSetPoint > CurrentRotation)
  {
    RotationError = RotationSetPoint - CurrentRotation;
    if (RotationError > 180)
    {
      RotationError = (360 - RotationError);
      RotationDerative = 6 * ( RotationError - LastRotationError);
      LastRotationError = RotationError;
      RotationVelocity = ((4 * RotationError + RotationDerative) / 5) * (-1);
      //RotationError = (360 - RotationError) * (-1);
    }
    else
    {
      RotationDerative = 6 * (RotationError - LastRotationError);
      LastRotationError = RotationError;
      RotationVelocity = ((4 * RotationError + RotationDerative) / 5);
    }
  
  }
  else {
    ResetRotationPID();
  }
  //if (RotationError > SpeedMission)
  if (RotationVelocity > 50) {
    RotationVelocity = 50;
  }
  else if (RotationVelocity < (-50)) {
    RotationVelocity = -50;
  }
}

void DirectionCalc(void)
{
  HeadingDirection = HeadingDirection - CurrentRotation;
  if (HeadingDirection < 0) {
    HeadingDirection = 360 + HeadingDirection;
  }
}

void BNO_Calibration(void)
{
    adafruit_bno055_offsets_t calibData;
/*    calibData.accel_offset_x = 65528;
    calibData.accel_offset_y = 65487;
    calibData.accel_offset_z = 45;
    
    calibData.gyro_offset_x = 65534;
    calibData.gyro_offset_y = 2;
    calibData.gyro_offset_z = 65534;
    
    calibData.mag_offset_x = 163;
    calibData.mag_offset_y = 65482;
    calibData.mag_offset_z = 65043;

    calibData.accel_radius = 1000;

    calibData.mag_radius = 459;*/
    calibData.accel_offset_x = 0;
    calibData.accel_offset_y = 0;
    calibData.accel_offset_z = 0;
    
    calibData.gyro_offset_x = 0;
    calibData.gyro_offset_y = 0;
    calibData.gyro_offset_z = 0;
    
    calibData.mag_offset_x = 0;
    calibData.mag_offset_y = 0;
    calibData.mag_offset_z = 0;

    calibData.accel_radius = 0;

    calibData.mag_radius = 0;

    bno.setSensorOffsets(calibData);
}

void CheckRange(void)
{
  unsigned int tempRange = 0;
  if (abs(RotationError) < 30)
  {
      if (Rear_Sensor.isFinished5())
      {
        tempRange = Rear_Sensor.getRange5();
        if (tempRange < 400) {Rear_Range = tempRange;}
        else { Rear_Range = 401;}
         Rear_Sensor.start5();
      }
      if (Right_Sensor.isFinished())
      {
        tempRange = Right_Sensor.getRange();
        if (tempRange < 400) {Right_Range = tempRange;}
        else {Right_Range = 401; }
         Right_Sensor.start();
      }
   } 
}


void SpeedPID()
{
  if (GoalDistance >  1) { Goal_Error = GoalDistance; }
  else {Goal_Derative = 0; Goal_LastError = 0; Goal_Error = 0; }
  Goal_Derative =  Goal_Error - Goal_LastError;
  Goal_LastError = Goal_Error;
  SpeedMission = ((3 * Goal_Error) + (9 * Goal_Derative))/ 5;
  if (SpeedMission < 0) SpeedMission = 0;
  if (SpeedMission > 150) SpeedMission = 150;
}
void ResetSpeedPID(void) { Goal_Derative = 0; Goal_LastError = 0; Goal_Error = 0; }
void ResetRotationPID(void)
{
    RotationError = 0;
    LastRotationError = 0;
    RotationDerative = 0;
    RotationVelocity = 0;
}

void Move(float _X_SetPoint, float _Y_SetPoint, uint8_t _maxSpeed, float _range, float _rotation)
{
    X_SetPoint = _X_SetPoint;
    Y_SetPoint = _Y_SetPoint;
    RotationSetPoint = _rotation;
    GoalDistance = _range + 1;
    ResetSpeedPID();
    Right_Sensor.start();
    while (GoalDistance > _range)
    {
      CheckRange();
      RotationErrorCalc();
      EncoderDisCalc();
      NavigationCalc();
      DirectionCalc();
      SpeedPID();
      if (SpeedMission > _maxSpeed) {SpeedMission = _maxSpeed;}
      SendMissionToMotorControl();
    } 
//  return 0;
}
void GoldenMove(float _X_SetPoint, float _Y_SetPoint, uint8_t _maxSpeed, float _range, float _rotation)
{
    X_SetPoint = _X_SetPoint;
    Y_SetPoint = _Y_SetPoint;
    RotationSetPoint = _rotation;
    GoalDistance = _range + 1;
    ResetSpeedPID();
    Rear_Sensor.start5();
    while (GoalDistance > _range)
    {
      CheckRange();
      RotationErrorCalc();
      EncoderDisCalc();
      GoldenNavigationCalc();
      DirectionCalc();
      SpeedPID();
      if (SpeedMission > _maxSpeed) {SpeedMission = _maxSpeed;}
      SendMissionToMotorControl();
    } 
//  return 0;
}
void RotationCorrect(float rot)
{
  RotationSetPoint = rot;
  do 
    {
      RotationErrorCalc();
      SendMissionToMotorControl();
      delay(10);
    } while (abs(RotationError) > 2);
    ResetRotationPID();
}
void RotationCorrect1(float rot)
{
  RotationSetPoint = rot;
  do 
    {
      RotationErrorCalc();
      SendMissionToMotorControl();
      delay(10);
    } while (abs(RotationError) > 8);
    ResetRotationPID();
}
void checkBallHand(void)
{
  uint8_t wait_time = 0;
  bool temp = digitalRead(LeftHand);
  if(temp == true) { ArmServo.writeMicroseconds(ServoBallPick); }
  if (temp == true)
  {
    do 
      { 
        if (wait_time < 20) {wait_time ++;}
        else { RelayStatus = "00000000";}
        RotationCorrect(RotationSetPoint);
        temp = digitalRead(LeftHand);
      } while (temp == true);
      RelayStatus = "00000000";
      SendMissionToMotorControl();
      delay(800);
  }
  else {RotationCorrect(RotationSetPoint);}
}

void ReloadBall(void)
{
  RotationCorrect(RotationSetPoint);
  ArmServo.writeMicroseconds(ServoBallReload);
  bool temp = digitalRead(Shooter);
  uint8_t wait_time = 0;
  while ((temp == true) && (wait_time < 40))
  {
    wait_time ++; 
    temp = digitalRead(Shooter);
    
  }
  RelayStatus = "00010001";
  SendMissionToMotorControl();
  delay(200);
}

void PickHand(void)
{
  for(int i = ServoBallReload ; i < ServoBallPick ;  i = i + 50 )
  {
      ArmServo.writeMicroseconds(i);
      delay(1);
  }
  bool temp = digitalRead(ManualLoad);
  uint8_t wait_time = 0;
  while ((temp == true) && (wait_time < 10))
  {
    wait_time ++; 
    temp = digitalRead(ManualLoad);
    RotationCorrect1(RotationSetPoint);
  }
  RelayStatus = "00000001";
  SendMissionToMotorControl();
  delay(100);
}

void BallShoot(void)
{
  bool temp = digitalRead(ManualLoad);
  uint8_t wait_time = 0;
  ArmServo.writeMicroseconds(ServoBallPick);
    while ((temp == true) && (wait_time < 500))
    {
      wait_time ++; 
      temp = digitalRead(ManualLoad);
      delay(1);
    }
    RelayStatus = "00000101";
    SendMissionToMotorControl();
    delay(1000);
    RelayStatus = "00000111";
    SendMissionToMotorControl();
}

void NavigationCalc(void)
{
  float X_Error = X_SetPoint - Encoder_X_Dis;
  float Y_Error = Y_SetPoint - Right_Range;
  float tempHead = atan2(X_Error , Y_Error);
  tempHead = ((tempHead * 180) / PI);
  if (tempHead < 0) { tempHead = 360 + tempHead; }
  HeadingDirection = tempHead;
  GoalDistance = sqrt((X_Error * X_Error) + (Y_Error * Y_Error));
  
}

void GoldenNavigationCalc(void)
{
  float X_Error = X_SetPoint - (720 - Rear_Range);
  float Y_Error = Y_SetPoint - Encoder_Y_Dis;
  float tempHead = atan2(X_Error , Y_Error);
  tempHead = ((tempHead * 180) / PI);
  if (tempHead < 0) { tempHead = 360 + tempHead; }
  HeadingDirection = tempHead;
  GoalDistance = sqrt((X_Error * X_Error) + (Y_Error * Y_Error));
  
}
void T1_T2_Shoot(uint8_t zone)
{
    uint16_t wait_time = 0;
    bool tempLeft;
    SpeedMission = 0; ResetRotationPID();
    SendMissionToMotorControl();  
    if (zone == T1)
    {
      Right_Sensor.start();
      NavigationCalc();
      CheckRange();
      float tempHead;
      tempHead = atan2((379 - (Right_Range + 40)),(Encoder_X_Dis - 34));
      tempHead = ((tempHead * 180) / PI);
      if (tempHead < 0) { tempHead = 360 + tempHead; }
      RotationSetPoint = round(tempHead);
      checkBallHand();
      //RotationCorrect(RotationSetPoint);
      //ReloadBall();
    }
    ReloadBall();   
shoot_again:
    BallShoot();
    if(zone == T2)
    {
      GoldenMove(595, 210,  80,  20,   0);
    }
    checkBallHand();
    if (zone == T1)
    {
      RotationCorrect(0);
      CheckRange();
      while(Right_Range < 150)
      {
        CheckRange();
      }
    }
    else
    {
//ReloadBall();
//PickHand();
      tempLeft = digitalRead(CornerIR);
      if(tempLeft == true) { goto shoot_again; }
      else 
      {
        //checkBallHand(); 
        //RotationCorrect1(5);
      }
    }
}

void Golden_Again_Shoot(void)
{
    SpeedMission = 0; ResetRotationPID();
    SendMissionToMotorControl();
    RotationSetPoint = 0;
    checkBallHand();  
    ReloadBall();
    ArmServo.writeMicroseconds(ServoBallPick);
    RotationCorrect(0);
}

void GoldenShoot_test(uint8_t zone)
{
    
  golden_shoot:
    RotationSetPoint = 0 ;
    SpeedMission = 0;
    delay(300);
    RelayStatus = "00000011";
    digitalWrite(Buzzer, HIGH);
    SendMissionToMotorControl();
    delay(500);
    RelayStatus = "00000000";
    SendMissionToMotorControl();
    delay(400);
    RotationCorrect(0);
    digitalWrite(Buzzer, LOW);
    bool tempLeft;
    uint16_t wait_time = 0;
    ResetRotationPID();
    tempLeft = digitalRead(LeftHand);
    if (tempLeft == false)
    {
      ReloadBall();
      delay(100);
      PickHand();
    goto golden_shoot;
    }
}
void GoldenPush(void)
{
  uint16_t wait_time = 0;
  ReloadBall();
  PickHand();
  while (wait_time < 5)
      {
        wait_time ++; 
        RotationCorrect(0);
        HeadingDirection = 270;
        DirectionCalc();
        SpeedMission = 13;
        SendMissionToMotorControl();
        delay(1);
      }
}
void Encoder_X(void)
{
  bool pinVal = digitalRead(Encoder_X_PIN);
  if ( pinVal == false) {Encoder_X_Val ++;}
  else {Encoder_X_Val --;}
}
void Encoder_Y(void)
{
  bool pinVal = digitalRead(Encoder_Y_PIN);
  if ( pinVal == false) {Encoder_Y_Val ++;}
  else {Encoder_Y_Val --;}
}
void EncoderDisCalc(void)
{
  float tempEncoder_X = Encoder_X_Val;
  Encoder_X_Dis = ((tempEncoder_X - 10000) / 25.4); //720 -
  float tempEncoder_Y = Encoder_Y_Val;
  Encoder_Y_Dis = ((tempEncoder_Y - 10000) / 25.4); // 865 -  
}
void SideSensorsSetup(void)
{
  float tempRange = (Right_Range * 25.4) + 10000;
  Encoder_Y_Val = tempRange;
}
void wait(uint8_t time_ms)
{
  uint16_t ttt = time_ms/100;
  for(uint16_t i = 0; i < (ttt); i ++)
  {
    RotationErrorCalc();
    SendMissionToMotorControl();
    delay(100);
  }
}
// gyroscope
// gar dooshluulah (shaarig harah)
// servonii ariin tulaas
// shidegchind hyzgaarlagch hiih 
// buh engiin bumbug shideh
// leftHand ynzlah
// gar dood tsenher hzgaarlah 
// shuud 590 ywah tgd 5sm uragshaa
void SendMissionToMotorControl(void)
{
  Serial1.print("<speed="); Serial1.print(SpeedMission, 0); Serial1.print(';');
  Serial1.print("dir="); Serial1.print(HeadingDirection , 1); Serial1.print(";");
  Serial1.print("rot="); Serial1.print(RotationVelocity, 0); Serial1.print(';');
  Serial1.print("valve="); Serial1.print(RelayStatus); Serial1.print(";>");
  Serial1.println("");

  //********************************************************************************************
  
 Serial.print("<speed="); Serial.print(SpeedMission, 0); Serial.print(';');
  Serial.print("dir="); Serial.print(HeadingDirection , 1); Serial.print(";");
  Serial.print("rot="); Serial.print(RotationVelocity, 0); Serial.print(';');
  Serial.print("valve="); Serial.print(RelayStatus); Serial.print(";>");
  Serial.print('\t');
  Serial.print('\t');
  Serial.print("RotSP="); Serial.print(RotationSetPoint , 1); Serial.print(";");
  Serial.print('\t');
  Serial.print("CurrRot="); Serial.print(CurrentRotation, 0); Serial.print(';');
  Serial.print('\t');
  Serial.print("Rear Dis="); Serial.print(Rear_Range); Serial.print(';');
  Serial.print('\t');
  Serial.print("Right Dis="); Serial.print(Right_Range); Serial.print(';');
  Serial.print('\t');
  Serial.print("Encoder Dis="); Serial.print(Encoder_Y_Dis); Serial.print(';');
/*  Serial.print("Rear Dis="); Serial.print(Rear_Range); Serial.print(';');
  Serial.print('\t');
  Serial.print("Front Dis="); Serial.print(EncoderDis); Serial.print(';');
  Serial.print('\t');
  Serial.print("Laser Range=");Serial.print(Range_1);
  //Serial.print(EncoderVal);
  //Serial.print('\t');
 // Serial.print(Right_Range); Serial.print('\t'); Serial.println(EncoderDis);// Serial.print(';');
 // Serial.print('\t');
 // Serial.print("Btn="); Serial.print(JoyStick_Btn,BIN);
  Serial.println("");*/
  // Serial.print(Range_1); 
   //Serial.print('\t');
   //Serial.print(Encoder_X_Dis); 
   Serial.println("");
}
void serialEvent2()
{
  while (Serial2.available())
  {
    char _UDR = (char)Serial2.read();
    if ((rx_2_complete == false) && (rx_2_pointer == 0))
    {
      if (_UDR == 0x59) {
        rx_2_pointer = 1;
        check_sum_2 = _UDR;
      }
    }
    else if ((rx_2_pointer == 1) && (rx_2_complete == false))
    {
      if (_UDR == 0x59) {
        rx_2_pointer = 2;
        check_sum_2 += _UDR;
      }
      else {
        rx_2_pointer = 0;
      }
    }
    else if ((rx_2_pointer >= 2) && (rx_2_complete == false))
    {
      rx_2_buffer += _UDR; rx_2_pointer ++;
      if (rx_2_pointer == 9)
      {
        if (check_sum_2 == _UDR) {
          rx_2_complete = true;
          rx_2_pointer = 0;
        }
        else {
          rx_2_pointer = 0;
        }
      }
      else {
        check_sum_2 += _UDR;
      }
    }
  }
}
void PoleLocation(void)
{
  if (rx_2_complete)
  {
    Range_1 = rx_2_buffer[1]; Range_1 <<= 8; Range_1 += rx_2_buffer[0];
    rx_2_complete = false; rx_2_buffer = ""; rx_2_pointer = 0;
  }

}
void serialFlush2(void)
{
  char _UDR;
  while (Serial2.available())
  {
    _UDR = (char)Serial2.read();
  }
}
