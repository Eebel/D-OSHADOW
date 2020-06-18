// =======================================================================================
//        SHADOW_MD:  Small Handheld Arduino Droid Operating Wand + MarcDuino
// =======================================================================================
//
//                Inspired by the PADAWAN / KnightShade SHADOW effort
//                Built for Version 2 of Mr. Baddeley's D-O
//                Hacked by Eebel:  15 Aug 18 -  Jun 2020
//                
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it for
//         your personal use and the personal use of other astromech club members.  
//
//         This program is distributed in the hope that it will be useful 
//         as a courtesy to fellow astromech club members wanting to develop
//         their own droid control system.
//
//         IT IS OFFERED WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//         You are using this software at your own risk and, as a fellow club member, it is
//         expected you will have the proper experience / background to handle and manage that 
//         risk appropriately.  It is completely up to you to insure the safe operation of
//         your droid and to validate and test all aspects of your droid control system.
//
// =======================================================================================
//   Note: You will need a Arduino Mega 2560 rev3 to run this sketch,
//   as a normal Arduino (Uno, Duemilanove etc.) doesn't have enough SRAM and FLASH
//
//   This is written to be a SPECIFIC Sketch - supporting only one type of controller
//      - PS3 Move Navigation 
//
//   PS3 Bluetooth library - developed by Kristian Lauszus (kristianl@tkjelectronics.com)
//   For more information visit my blog: http://blog.tkjelectronics.dk/ or
//
//  - Arduino Mega 2560 rev3
//  - USB Host Shield
//  - USB Class I Bluetooth Dongle
//  - Cytron MDD10A Motor driver
//  - DFPlayer Mini
//  - MPU6050 IMU (Inertial Measurement Unit)
//  - Pololu 20.4:1 HP 12V 25mm DC Motors (No encoder)
//  - PCA 9685 Servo Expander
//  - HiTech HS-645MG (MainArmServo)
//  - HiTech HS-85MG  (HeadNodServo)
//  - HiTech HS-65HB  (HeadTiltServo
//  - HiTech HS-65HB  (HeadTrun Servo)
// ***************************************************************************************************

#include <PS3BT.h>
#include <usbhub.h>
#include <Arduino.h>

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!! Comment out line 40 "#define USE_PCA9685_SERVO_EXPANDER" in ServoEasing.h to make the expander example work !!!
 * Otherwise you will see errors like: "PCA9685_Expander:44:46: error: 'Wire' was not declared in this scope"
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
#include "ServoEasing.h"
#include <Wire.h>
#include "DFRobotDFPlayerMini.h" //For audio
//#include "elapsedMillis.h"

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define LOG_INPUT


//#define PCA9685  //uncomment if using PCA9685

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

//**************MPU VARIABLES*********************************************
const int MPU6050_addr=0x68;
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;
double compAngleX,compAngleY,/*gzangle,*/compAngleZ,timer;
  double accXangle ,accYangle,acczangle ,gyroXrate ,gyroYrate,gyroZrate;
  double gyroXAngle, gyroYAngle, gyroZAngle;
  // float rgyro,w;
  int ap=0.955;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

float elapsedTime, thisTime, timePrev;
//*******END MPU VAIRABLES************************************************

//******PID CONSANTS******************************************************
float PID, pwmLeft, pwmRight, error, previous_error;
//Tweak the PID values here
float kp=20;  //was 25
float ki=20; //was 0
float kd=0; //was.8
float pid_p=kp;
float pid_i=ki;
float pid_d=kd;
float desired_angle = 0;//////////////TARGET ANGLE/////////////
//******END PID CONSTANTS************************************************

//******PWM MOTOR CONTOL*************************************************
const int pinSTBY = 9;   //On/Off switch?
const int pinAIN1 = 8;  //Motor Direction pin (goes to DIR1) :dir1pin
const int pinPWMA = 11;  //Motor Speed pin (goes to PWM1) :speed1pin
const int pinBIN1 = 12;  //Motor Direction pin (goes to DIR2) :dir2pin
const int pinPWMB = 13;  //Motor Speed pin (goes to PWM2) :speed2pin
int mspeed = 10;         //Motor Speed Scale Factor?
int motorDeadZone = 20;  //Set this value higher to elimintate chatter when nearly balanced  
//int turnspeed=50;        //Turn Speed Scale Factor?

//int motorspeed1 = 0;
int motordirection1 = HIGH;
//int motorspeed2 = 0 ;
int motordirection2 = HIGH;
bool RobotOn = false;     //Robot on or off
//******END PWM MOTOR CONTROL********************************************

//*******GLOBAL PROCEDURE DECLARATIONS***********************************
void setupMPU();  //A procedure to initialize and calibrate MPU
void readMPU();   //A procedure to raw MPU information
void getYPR();    //Turn raw MPU data to Yaw/Pitch/Roll values
void getPid();    //Calculate PID values
void setupMotors();//Setup Moto pins
void moveMotors(int motorspeed1, int motorspeed2);//Move Motors based on PID calcs
void setupServos();                         //Setup Servos
void onInitPS3();
void ReadBattery();                         //Get battery level
void AdjustVolume(bool volumeUp);           //Send True to increase/False to decrease
//******END GLOBAL PROCEDURE DECLARATIONS*******************************

//*****************************************************************************
//*************************GLOBAL VARIABLES************************************
//*****************************************************************************
String PS3_MAC = "00:07:04:BA:6F:DF";  //Set Controller MAC Address here!

int PSMotor1Cmd=0;  //Commanded speed generated by PS3 controller routine
int PSMotor2Cmd=0;
int vMotor1 = 0;
int vMotor2 = 0;
int vSpeed = 0;     //motor speed in PS3 calc
//int Speed = 0;    
byte X_Axis=0;      //PS3 hat values
byte Y_Axis=0;
byte L2BUtton=0;
bool L1Pressed=false;
bool HeadMode=false;          // With the central button of the joystickk (L3) we change between head and motors
bool AutoHeadMoveOn=false;    // Enable random head movement of the head 
bool AutoHeadMove = false;    // Auto-Head
bool PS3_CONNECTED = false;   //flag for PS3 Satus
//Other Variables
byte joystick=0;
int deadZone = 30;        //joystick DeadZone
bool T1 = false;
bool FlT1=false;
int cntT1 = 0;
int cntT2 = 0;
int cntT3 = 0;
int cntTBatt = 0;
int cntAutMov = 0;            // Automatic head movement
int cntAutSon = 0;            // Automatic sound
int soundVolume = 25;                  //0 to 30 range
//******BATTERY VARIABLES**************************************************************************
//#define ShowBatteryInfo             //Send debug info to console
#define battPin0 A0                   //Plug battery into this Pin for measurment
const float LowBatt = 3.6;         //Low Battery voltage setting 5K1 and 2K resistor
//loat battWeighting = 0.1; 
int ADCValue0 = 0; 
float Cell1 = 0.00; 
bool LowBattery = false;              //Low battery state flag
bool BatteryWarning = true;           //Do or dont' look for battery state
bool Battery_LED = false;             //Low Battery LED 

//******Servo Pins************************************************************
const int pinArmServo = 46;   //Arm Angle is driven from 2560 due to location in the body
#ifdef PCA9685
//------Rest of the servos are in the head and on the PCA 9685 Expander------
  const int pinTiltServo = 5;     //Tilt Side-To-Side
  const int pinNeckTurn = 6;   //Turn Neck
  const int pinHeadNod = 7;   //Nod Up-And-Down
#else
  const int pinTiltServo = 44;     //Tilt Side-To-Side
  const int pinNeckTurn = 45;   //Turn Neck
  const int pinHeadNod = 47;   //Nod Up-And-Down
#endif


//******GLOBAL CLASS INSTANCE DECLARATIONS******************************
USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT *PS3Nav=new PS3BT(&Btd);// This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - 
                                                       // this can be obtained from the dongle when running the sketch
DFRobotDFPlayerMini myDFPlayer;
//******END GLOBAL CLASS INSTANCE DECLARATIONS**************************

#ifdef PCA9685
  ServoEasing HeadTiltServo(PCA9685_DEFAULT_ADDRESS, &Wire);
  ServoEasing NeckTurnServo(PCA9685_DEFAULT_ADDRESS, &Wire);
  ServoEasing HeadNodServo(PCA9685_DEFAULT_ADDRESS, &Wire);
#else
  ServoEasing HeadTiltServo;
  ServoEasing NeckTurnServo;
  ServoEasing HeadNodServo;
#endif
ServoEasing ArmServo; //Not going to the PCA9685

struct ServoControlStruct {
    uint16_t minDegree;
    uint16_t maxDegree;
    uint16_t center;
};

//-=-=-=-Structure for easier to read code-=-=-
ServoControlStruct ArmServoControl;
ServoControlStruct NeckTurnServoControl;
ServoControlStruct HeadNodServoControl;
ServoControlStruct HeadTiltServoControl;

//-=-=- Head servo targets-=-=- main loop will set these according to IMU and user inputs
uint8_t tNewArmPosition = 90;
uint8_t tNewHorizontal = 90;
uint8_t tNewVertical = 90;
uint8_t tNewTilt = 90;
uint16_t armCenter = 90;
uint16_t neckCenter = 90;
uint16_t nodCenter = 90;
uint16_t tiltCenter = 90;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//extra stuff



void setup() {
  
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    //******DF MINIPLAYER SETUP*******************************
    Serial1.begin(9600);//DFMiniPlayer Card
    Serial.println();
    Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  
  if (!myDFPlayer.begin(Serial1)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  
  myDFPlayer.volume(soundVolume);  //Set volume value. From 0 to 30
  myDFPlayer.play(1);  //Play the first sound
  

  setupServos(); 
  setupMPU();  
  setupMotors();
  moveMotors(0, 0);


  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.println(F("\r\nPS3 Bluetooth Library Started"));
  PS3Nav->attachOnInit(onInitPS3);
  // configure LED for output - NOT NEEDED
  pinMode(LED_PIN, OUTPUT);

  Serial.print("gyro_pitch_calibration_value: ");
  Serial.println(gyro_pitch_calibration_value);
  Serial.print("gyro_yaw_calibration_value: ");
  Serial.println(gyro_yaw_calibration_value); 
}

void loop() {

  Usb.Task();
  if (PS3_CONNECTED == false) {
    PSMotor1Cmd = 0;
    PSMotor2Cmd = 0;
    return;//give time for PS3 to connect befor MPU calcs
  }

    readMPU();
    getYPR();
    getPid();
    ReadPS3Controller(); 
//    Serial.print("PSMotor1Cmd: ");Serial.println(PSMotor1Cmd);
//    Serial.print("PSMotor2Cmd: ");Serial.println(PSMotor2Cmd);    
    moveMotors(PSMotor1Cmd, PSMotor2Cmd);

    if(HeadMode==false){
      if(T1==true) {
        if (joystick==0){
          tNewArmPosition = ArmServoControl.center;//Servo Ease to this location
        }else{
          tNewArmPosition = ArmServoControl.center + (compAngleX*0.6);
        }
        //digitalWrite(HeadLED1, HIGH);
      }else{
        //digitalWrite(HeadLED1, LOW);
        tNewArmPosition = ArmServoControl.center;//Servo Easto this location   
      }
    }
    
    if(compAngleX>3){tNewVertical = HeadNodServoControl.center - max(compAngleX, 15);}
    if(compAngleX<-3){
      tNewVertical = HeadNodServoControl.center + max(compAngleX, 15);
    }else{
      tNewVertical - HeadNodServoControl.center;
    }
//    Serial.print("compAngleX:" );Serial.println(compAngleX);
    moveServos(); 

}//end Main Loop

void setupServos(){ 

   // Initialize wire before checkI2CConnection()
    Wire.begin();  // Starts with 100 kHz. Clock will eventually be increased at first attach() except for ESP32.
    Serial.println(F("Try to communicate with PCA9685 Expander by TWI / I2C"));
    Serial.flush();
    Wire.beginTransmission(PCA9685_DEFAULT_ADDRESS);
    uint8_t tWireReturnCode = Wire.endTransmission(true);
    if (tWireReturnCode == 0) {
        Serial.print(F("Found"));
    } else {
        Serial.print(F("Error code="));
        Serial.print(tWireReturnCode);
        Serial.print(F(". Communication with I2C was successful, but found no"));
    }
    Serial.print(F(" I2C device attached at address: 0x"));
    Serial.println(PCA9685_DEFAULT_ADDRESS, HEX);

    //Serial.println(F("Attach servo to port 9 of PCA9685 expander"));
    /**************************************
     * Attach the expander servos first!
     **************************************/
    if (NeckTurnServo.attach(pinNeckTurn) == INVALID_SERVO) {
        Serial.println(
                F("Error attaching NeckTurnServo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
    }else{
      Serial.print(F("Attached NeckTurnServo to PCA9685 expander port: "));
      Serial.println(pinNeckTurn);
    }
    if (HeadNodServo.attach(pinHeadNod) == INVALID_SERVO) {
        Serial.println(
                F("Error attaching HeadNodServo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
    }else{
      Serial.print(F("Attached HeadNodServo to PCA9685 expander port: "));
      Serial.println(pinHeadNod);
    }
    if (HeadTiltServo.attach(pinTiltServo) == INVALID_SERVO) {
        Serial.println(
                F("Error attaching HeadTiltServo - maybe MAX_EASING_SERVOS=" STR(MAX_EASING_SERVOS) " is to small to hold all servos"));
    }else{
      Serial.print(F("Attached HeadTiltServo to PCA9685 expander port: "));
      Serial.println(pinTiltServo);
    }

    // Attach servo to pin
    Serial.println(F("Attach Arduino servos last."));
    if (ArmServo.attach(pinArmServo) == INVALID_SERVO) {
        Serial.print(F("Error attaching ArmServo to pin: "));
        Serial.println(pinArmServo);
    }else{
      Serial.print(F("Attached ArmServo to Arduino port: "));
      Serial.println(pinArmServo);
    }

    
    /**************************************************
     * Set servos to start position.
     * This is the position where the movement starts.
     *************************************************/
  

    setSpeedForAllServos(360);

    ArmServoControl.minDegree = 70;
    ArmServoControl.maxDegree = 130;
    ArmServoControl.center = armCenter;
    NeckTurnServoControl.minDegree = 10;   
    NeckTurnServoControl.maxDegree = 170;  
    NeckTurnServoControl.center = neckCenter;
    HeadNodServoControl.minDegree = 50;     
    HeadNodServoControl.maxDegree = 140;   
    HeadNodServoControl.center = nodCenter;
    HeadTiltServoControl.minDegree = 35;  
    HeadTiltServoControl.maxDegree = 155; 
    HeadTiltServoControl.center = tiltCenter;


    NeckTurnServo.write(NeckTurnServoControl.center);
    HeadNodServo.write(HeadNodServoControl.center);
    HeadTiltServo.write(HeadTiltServoControl.center);
    ArmServo.write(ArmServoControl.center);
    
    // Wait for servos to reach start position.
    delay(500);
}



//Move Servos with ServoEasing Libray
void moveServos(){
  if (!ArmServo.isMoving()) {
        ArmServo.setEaseTo(tNewArmPosition);//, tSpeed);
        synchronizeAllServosAndStartInterrupt();
    }

  if (!NeckTurnServo.isMoving()) {
        NeckTurnServo.setEaseTo(tNewHorizontal);//, tSpeed);
        synchronizeAllServosAndStartInterrupt();
    } 
    
  if (!HeadNodServo.isMoving()) {     
      HeadNodServo.setEaseTo(tNewVertical);//, tSpeed);
      synchronizeAllServosAndStartInterrupt();
  } 

    if (!HeadTiltServo.isMoving()) {   
        HeadTiltServo.setEaseTo(tNewTilt);
        synchronizeAllServosAndStartInterrupt();
    }   
}//End moveServos


void ReadPS3Controller()
{
  //int Speed = 0; //Speed (Y axis) and vSteer (X axis) 
  int vSteer = 0; 
  
  // Motor Variables
  int leftMotorCmd = 0; 
  int rightMotorCmd = 0;                 
  int leftMotorScaled = 1; 
  int rightMotorScaled = 1;
  float leftMotorScale = 1;
  float rightMotorScale = 1;
  float maxMotorScale = 1;
  
 
  if (PS3Nav->PS3NavigationConnected || PS3Nav->PS3Connected){
    //Get Joystick Valuesk
    Y_Axis = PS3Nav->getAnalogHat(LeftHatY);  //0 to 255 128 is middle
    X_Axis = PS3Nav->getAnalogHat(LeftHatX);  //0 to 255 128 is middle

    
    // Cursor L2
    L2BUtton = PS3Nav->getAnalogButton(L2);   //0 to 255, Not pressed is 0

    // Hmm, we are here, so robot is on!
    RobotOn = true;
    digitalWrite(pinSTBY, HIGH);

    if(HeadMode==false){
      
      //aquire the analog input for Y  and rescale the 0..255 range to -255..255 range
      vSpeed = -(127-Y_Axis)*2; //Mapped Range of input
      vSteer = (127-X_Axis)/2; //Steering -64 to 64

      joystick=0;
      // Detect Y_Axis      
      if(vSpeed>deadZone){
        joystick=2; //Move Forward?
      }else{
        if(vSpeed<-deadZone){
          joystick=1; //Move Backward?
        }else{
          // detect X_Axis
          if(vSteer>deadZone){
            joystick=3; //Turn Right?
          }else{
            if(vSteer<-deadZone){
              joystick=4; //Turn Left
            }
          }
        }
      }
      
      if(joystick>0){
//          Serial.print("Battery Level: ");
//          Serial.println(ADCValue0);
        T1=true;
        cntT1=0;
        if(LowBattery==true && BatteryWarning==true){ 
          BatteryWarning=false;
          //playMp3(100); //Low battery Warning
          myDFPlayer.play(9);  //Sad
          delay(500);
          myDFPlayer.play(9);  //Sad
          delay(500);
          myDFPlayer.play(15);  //I have a squeaky wheel 
//          Serial.print(Battery Level: ");
//          Serial.println(ADCValue0);
        } 
      }

//      Speed=vSpeed;

      #ifdef DebugVerbose
        Serial.print("Y_Axis: "); Serial.print( Y_Axis, DEC);
        Serial.print(", Y: "); Serial.print( Y_Axis, DEC);
        Serial.println();
      #endif
  
      //mix Speed and vSteer
      //mix Y and X Axis
            //-255-64 to 255+64 or -319 to 319
      //max is 255 so we need to scale it down to -255 to 255
      leftMotorCmd = map(vSpeed+vSteer, -319,319, -255,255);
      rightMotorCmd = map(vSpeed-vSteer, -319, 319, -255, 255); 
      leftMotorCmd = constrain(leftMotorCmd, -255, 255);
      rightMotorCmd = constrain(rightMotorCmd, -255, 255);

 

      if(abs(leftMotorCmd)>deadZone) {
        PSMotor1Cmd=leftMotorCmd;
      }else{
        PSMotor1Cmd=0;
      } 
      if(abs(rightMotorCmd)>deadZone) {
        PSMotor2Cmd=rightMotorCmd;
      }else{
        PSMotor2Cmd=0;
      } 

      #ifdef DebugVerbose
      //print the initial mix results
        Serial.print("LIN:"); Serial.println( leftMotorCmd, DEC);
        Serial.print(", RIN:"); Serial.println( rightMotorCmd, DEC);
      #endif
      
//      //calculate the scale of the results in comparision base 8 bit PWM resolution
//      leftMotorScale =  leftMotorCmd/255.0;
//      leftMotorScale = abs(leftMotorScale);
//      rightMotorScale =  rightMotorCmd/255.0;
//      rightMotorScale = abs(rightMotorScale);
//
//      #ifdef DebugVerbose
//        Serial.print("| LSCALE:"); Serial.println( leftMotorScale,2);
//        Serial.print(", RSCALE:"); Serial.println( rightMotorScale,2);
//      #endif
//    
//      //choose the max scale value if it is above 1
//      maxMotorScale = max(leftMotorScale,rightMotorScale);
//      maxMotorScale = max(1,maxMotorScale);
//    
//      //and apply it to the mixed values
//      leftMotorScaled = constrain(leftMotorCmd/maxMotorScale,-255,255);
//      rightMotorScaled = constrain(rightMotorCmd/maxMotorScale,-255,255);
//
//      #ifdef DebugVerbose
//        Serial.print("| LOUT:"); Serial.print( leftMotorScaled);
//        Serial.print(", ROUT:"); Serial.println( rightMotorScaled);
//      #endif
//    
//      //apply the results to appropriate PWM outputs for the LEFT motor:
//      if(abs(leftMotorScaled)>deadZone){ 
//        PSMotor1Cmd=leftMotorScaled;
//      }else{
//        PSMotor1Cmd=0;
//      } 
//    
//      //apply the results to appropriate PWM outputs for the RIGHT motor:  
//      if(abs(rightMotorScaled)>deadZone) {
//        PSMotor2Cmd=rightMotorScaled;
//      }else{
//        PSMotor2Cmd=0;
//      } 

      #ifdef DebugVerbose
        Serial.print ("PSMotor1Cmd: ");
        Serial.print (PSMotor1Cmd);
        Serial.print (" ");
      //  Serial.print ("motordirection1: ");
      //  Serial.print (motordirection1);
      //  Serial.println (" ");
        Serial.print ("PSMotor2Cmd: ");
        Serial.print (PSMotor2Cmd);
      //  Serial.print ("motordirection2: ");
      //  Serial.print (motordirection2);
        Serial.println (" ");
      #endif
  
      
      // *******************************************************************
      //Correct the head according to the turn
      // *******************************************************************
      if(AutoHeadMove==false){
        if(vSteer>deadZone){
          //NeckDestination=NeckCenter+50;//old servo code delete
          tNewHorizontal = NeckTurnServoControl.maxDegree;
        }else{
          if(vSteer<-deadZone){
            //NeckDestination=NeckCenter-50;//old servo code delete
            tNewHorizontal = NeckTurnServoControl.minDegree;
          }else{
            //NeckDestination=NeckCenter;
            tNewHorizontal = NeckTurnServoControl.center;
          }
        }
      }

  
    }else{
      //HeadMode = true
      PSMotor1Cmd=0;
      PSMotor2Cmd=0;
      AutoHeadMove=false;
      // ---------------------------------------------------------------------
      // Neck Control
      // ---------------------------------------------------------------------
      if(X_Axis==0){ X_Axis=1; }
  
      
      if(X_Axis>(127+deadZone)){
        tNewHorizontal = NeckTurnServoControl.maxDegree;
      }else if ( X_Axis<(127-deadZone)){
          tNewHorizontal = NeckTurnServoControl.minDegree;
        //}
      }else{
        tNewHorizontal = NeckTurnServoControl.center;//might be wrong to have this last "else"
      }
      
      
      // ---------------------------------------------------------------------
      // ARM CONTROL
      // ---------------------------------------------------------------------
      if(Y_Axis==0){ Y_Axis=1; }

      // Control Arn Angle
      if(Y_Axis<(127-deadZone) || Y_Axis>(127+deadZone)){
        tNewArmPosition = map(Y_Axis, 0, 255, ArmServoControl.minDegree, ArmServoControl.maxDegree);
        joystick=1;
      }else{
        tNewArmPosition = ArmServoControl.center;
        joystick=0;
      }

        #ifdef DebugVerbose
          Serial.print("ptmp2: "); Serial.print( ptmp2, DEC);
          Serial.print("    HeadNodPosition: "); Serial.print( HeadNodPosition, DEC);
          Serial.print("    HeadNodDestination: "); Serial.print( HeadNodDestination, DEC);
          Serial.print("    HeadNodTravelMaxDegrees: "); Serial.print( HeadNodTravelMaxDegrees, DEC);
          Serial.println();
        #endif
    }
    

    // ---------------------------------------------------------------------
    // Tilt Side-To-Side CONTROL
    // ---------------------------------------------------------------------
    
    if(L2BUtton>deadZone){
      if(L1Pressed==false){
        tNewTilt = HeadTiltServoControl.minDegree;
      }else{
        tNewTilt = HeadTiltServoControl.maxDegree;
      }
    }else{
      //if(ang_x <= 5){
        tNewTilt = HeadTiltServoControl.center;
     // }
      //else{
       // TiltPosition=TiltCenter+(ang_x*0.6);
      //}      
    }
    
    // Power Button
    if (PS3Nav->getButtonClick(PS)) {
      //Serial.print(F("\r\nPS"));
      //PS3.disconnect();
      PS3Nav->disconnect();
      PS3_CONNECTED = false;
      RobotOn = false;
      digitalWrite(pinSTBY, LOW);
    } else {
      /* if (PS3.getButtonClick(TRIANGLE)) {
        Serial.print(F("\r\nTraingle"));
      }*/
      // Circle Button  
      if (PS3Nav->getButtonClick(CIRCLE)){
        //Serial.print(F("\r\nCircle"));
        // Autorizacion de la cabeza
        //AutoHeadMoveOn=not AutoHeadMoveOn;
        AutoHeadMove=not AutoHeadMove;
        if(AutoHeadMove==false){
          tNewHorizontal = NeckTurnServoControl.center;
        }
        AdjustVolume(LOW);
        myDFPlayer.play(6); //Happy
        //PlaySound();
      }
  
      if (PS3Nav->getButtonClick(CROSS)){
        //Serial.print(F("\r\nCross"));
//        digitalWrite(HeadLED2, HIGH);
//        digitalWrite(HeadLED3, HIGH);
//        cntT3=LEDTIme;
        AdjustVolume(HIGH);
        myDFPlayer.play(10);//N_N_No Thankyou
        //playMp3(10); 
      }
    }

    // Boton L1 para los sonidos con los botones
    // L1 button for button sounds
    if (PS3Nav->getButtonPress(L1)) {
      if (PS3Nav->getButtonClick(UP)){
        //playMp3(3); 
        myDFPlayer.play(2);  //He_Hello2
      }else{
        if (PS3Nav->getButtonClick(DOWN)){
          //playMp3(21); 
          myDFPlayer.play(15);  //I have a squeaky Wheel
        }else{ 
          if (PS3Nav->getButtonClick(RIGHT)){
            //playMp3(22); 
            myDFPlayer.play(11);  //What's that
          }else{
            if (PS3Nav->getButtonClick(LEFT)){
              //playMp3(23); 
              myDFPlayer.play(8);  //Very Kind
            }
          }
        }
      }
    }
    
    // Button L1
    if (PS3Nav->getButtonClick(L1)) { L1Pressed = not L1Pressed;  }
    // L3 is pushing the hat in
    if (PS3Nav->getButtonClick(L3)) { HeadMode= not HeadMode; }


    // ---------------------------------------------------------------------
    // Nod CONTROL
    // ---------------------------------------------------------------------

    // Right Cursor Button
    if (PS3Nav->getButtonClick(RIGHT)) { 
      tNewVertical = HeadNodServoControl.minDegree;
      }
    // Boton Cursor Izquierda
    if (PS3Nav->getButtonClick(LEFT)) { 
      tNewVertical = HeadNodServoControl.maxDegree;
      }
    // Boton Cursor Arriba
    if (PS3Nav->getButtonClick(UP)) { 
      tNewHorizontal = NeckTurnServoControl.center;
    }
    // Boton Cursor Abajo
    if (PS3Nav->getButtonClick(DOWN)) { 
      tNewVertical = HeadNodServoControl.center;
      }

  }
    #ifdef DebugVerbose
      Serial.print("X: "); Serial.print( X_Axis, DEC);
      Serial.print(", Y: "); Serial.print( Y_Axis, DEC);
      Serial.print("     ");
    #endif
}


void onInitPS3()
{
    String btAddress = getLastConnectedBtMAC();
    PS3Nav->setLedOn(LED1);

    PS3_CONNECTED = true;
    //Serial.print ("\r\nBT Address of Last connected Device when Primary PS3 Connected: ");
    //Serial.print(btAddress);
    //Serial.println();
    
    if (btAddress != PS3_MAC){
        //Serial.print ("\r\nWe have our primary controller connected.\r\n");
        //delay(1000);
//        PS3Nav->disconnect();
    //}else{
    //    Serial.print ("\r\nWe have a controller connected, but it is not designated as \"primary\".\r\n");
    }
}

String getLastConnectedBtMAC()
{
    String btAddress = "";
    for(int8_t i = 5; i > 0; i--)
    {
        if (Btd.disc_bdaddr[i]<0x10)
        {
            btAddress +="0";
        }
        btAddress += String(Btd.disc_bdaddr[i], HEX);
        btAddress +=(":");
    }
    btAddress += String(Btd.disc_bdaddr[0], HEX);
    btAddress.toUpperCase();
    return btAddress; 
}


void setupMPU(){
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B); // Wake up the MPU6050
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6A); // Disable the FIFO buffer
  Wire.write(0);
  Wire.endTransmission(true);
 
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x1B);                 //0x1B (GYRO_CONFIG) ± 250 °/s   
  Wire.write(0);                    //000000   
  Wire.endTransmission(true);         
  
  Wire.beginTransmission(MPU6050_addr);  
  Wire.write(0x1C);                 //0x1C (ACCEL_CONFIG) ± 4g 
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)  
  //Wire.write(0);                    //000000 this is 2G
  Wire.endTransmission(true);     

  //Set some filtering to improve the raw data.
  Wire.beginTransmission(MPU6050_addr);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                   //End the transmission with the gyro

  for(int receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(MPU6050_addr);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(MPU6050_addr, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }

  //TODO:  USe these values somewhere to finetune gyro outputs.
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
  
}
void readMPU(){
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
}

void getYPR(){
 accXangle = (atan2(AccY, AccZ) * RAD_TO_DEG);
 accYangle = (atan2(AccX, AccZ) * RAD_TO_DEG);
 acczangle = (atan2(AccX,AccY) * RAD_TO_DEG);/* my attempt to calculate yaw but not correct*/
 gyroXrate = GyroX / 16.5;
 gyroYrate = GyroY / 16.5;
 gyroZrate = GyroZ/ 16.5;
 timer = millis();
 //angular position
 gyroXAngle += gyroXrate * (millis()-timer)/1000;
 gyroYAngle += gyroYrate * (millis()-timer)/1000;
 gyroZAngle += gyroZrate * (millis()-timer)/1000;/* my attempt to calculate yaw but not correct*/
//---------------------------\\
 //COMPLIMENTRY FILTER STARTED\\
 //---------------------------\\

 compAngleX = ap * (compAngleX + gyroXAngle) + (1-ap) * accXangle;
 compAngleY = ap * (compAngleY + gyroYAngle) + (1-ap) * accYangle;
 compAngleZ = ap * (compAngleZ + gyroZAngle) + (1-ap) * acczangle; /*yaw but not correct*/
 //---------------------------\\
 //COMPLIMENTRY FILTER ENDED  \\
 //---------------------------\\

//   Serial.print("ypr\t");
//            Serial.print(compAngleZ);
//            Serial.print("\t");
//            Serial.print(compAngleX);
//            Serial.print("\t");
//            Serial.println(compAngleY);
  
}
void getPid(){
  
//    timePrev = time;  
    thisTime = millis();  //get current time
    elapsedTime = (thisTime - timePrev) / 1000; //time since last loop

    error = compAngleX - desired_angle; /////////////////ERROR CALCULATION////////////////////
    ///////////////////////PROPORTIONAL ERROR//////////////
    pid_p = kp*error;
    ///////////////////////INTERGRAL ERROR/////////////////
    pid_i = pid_i+(ki*error);  
    ///////////////////////DIFFERENTIAL ERROR//////////////
    pid_d = kd*((error - previous_error)/elapsedTime);
    timePrev = millis();  //Time for next loop
    ///////////////////////TOTAL PID VALUE/////////////////
    PID = pid_p + pid_d;
    ///////////////////////UPDATING THE ERROR VALUE////////
    previous_error = error;
//    Serial.print("PID: ");
//    Serial.println(PID);                     //////////UNCOMMENT FOR DDEBUGGING//////////////
////    delay(60);                               //////////UNCOMMENT FOR DDEBUGGING//////////////
//    Serial.println(compAngleX);          //////////UNCOMMENT FOR DDEBUGGING//////////////
}


void moveMotors(int motorspeed1, int motorspeed2){
    /////////////////CONVERTING PID VALUES TO ABSOLUTE VALUES//////////////////////////////////
    
    mspeed = abs(PID);
    if (mspeed >255) {mspeed = 255;}
    //mspeed=map(mspeed,-0,500,0,255);
    
    //Serial.println(mspeed);                  //////////UNCOMMENT FOR DDEBUGGING//////////////
    ///////////////SELF EXPLANATORY///////////////
   
    if(compAngleX<0)
      {
       mspeed=-mspeed;
      }
    if(compAngleX>0)
      {
      mspeed = mspeed;
      }
    
motorspeed1=motorspeed1+mspeed; //This add the RC drive to the correction drive, motorspeed is the rc, mspeed from the IMU
motorspeed2=motorspeed2-mspeed; //This add the RC drive to the correction drive, motorspeed is the rc, mspeed from the IMU

//Serial.print (mspeed);
//Serial.print (" ");
//Serial.print (mspeed);
//Serial.print (" ");

if (motorspeed1<0) {
  motordirection1 = LOW;
  motorspeed1=-motorspeed1;
}

else if (motorspeed1>0) {
 motordirection1 = HIGH;  
}


if (motorspeed2>-motorDeadZone && motorspeed2<motorDeadZone){
  motorspeed2=0;

}
  
if (motorspeed1>-motorDeadZone && motorspeed1<motorDeadZone){
  motorspeed2=0;
}


if (motorspeed2<0) {
  motordirection2 = LOW;
  motorspeed2=-motorspeed2;
}

else if (motorspeed2>0) {
 motordirection2 = HIGH;  
}

if (motorspeed1 >254){
  motorspeed1=255;
}

if (motorspeed2 >254){
  motorspeed2=255;
}

if (motorspeed1 == 0 || motorspeed2 ==0){
  motorspeed1 = 0;
  motorspeed2 = 0;
}

//Serial.print (motorspeed1);
//Serial.print (" ");
//Serial.print (motorspeed2);
//Serial.println (" ");

digitalWrite(pinAIN1,motordirection1);
analogWrite(pinPWMA,motorspeed1); //increase the speed of the motor from 0 to 255
digitalWrite(pinBIN1,motordirection2);
analogWrite(pinPWMB,motorspeed2); //increase the speed of the motor from 0 to 255

}
void setupMotors(){  
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinPWMB, OUTPUT);

  RobotOn=true;//was commented
  digitalWrite(pinSTBY, HIGH);//was commented out
}

void AdjustVolume(bool volumeUp){
//  Serial.print("volumeUp = ");
//  Serial.println(volumeUp);
  
  if (volumeUp == 1){
    soundVolume += 3;
    if (soundVolume > 30) {
      soundVolume = 30;
    }
  }else{
    soundVolume -= 3;
    if (soundVolume < 0) {soundVolume =0;
    }
  }
//  Serial.print("Sound Volume set to ");
//  Serial.println(soundVolume);
  myDFPlayer.volume(soundVolume);
}
// Battery Check
void ReadBattery()
{
  //const float adcVolt = 0.003222656; 
  
  //pinMode(battPin0, INPUT);
  // read battery 50 times... not sure why not just once
  //for (int ii=0; ii<50; ii++){
    ADCValue0 = analogRead(battPin0);
    ADCValue0 = ADCValue0 * 5.0/1023;
    //Using 5K1 and 2K resistors for voltage divider to send less the 5V to input pin
    //Cell1 = (adcVolt * ADCValue0 * 4.73);
    
  //}  
  LowBattery=false;
  //if(Cell1<LowBatt){ LowBattery=true; }
  if(ADCValue0<LowBatt){ LowBattery=true; }
  
  // audio de bateria baja
  //if(LowBattery==true){ playMp3(100); }  
  if(LowBattery==true){ myDFPlayer.play(9); }  
}
