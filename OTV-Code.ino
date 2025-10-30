#include <Servo.h>
#include "Enes100.h"
#include <math.h>
#include "TLx493D_inc.hpp"
#include <Wire.h>
#include <Tank.h>

//Defining Pins here
#define FORWARD 0
#define RIGHT 1
#define LEFT 2
#define CUT -1

#define F_ORI -90
#define L_ORI 90
#define R_ORI -180

//Ultrasonic Pins 1 = FORWARD, 2 = Right, 3 = Left
#define echoPin1 1
#define trigPin1 4
#define echoPin2 5
#define trigPin2 6
#define echoPin3 99
#define trigPin3 100

#define teamMarker 13

//ESP8826 Pins
#define espRX 2
#define expTX 3

//L298N Motor Pins: 
#define motor1pin1 5
#define motor1pin2 6
#define motor1en 4
#define motor2pin1 10
#define motor2pin2 11
#define motor2en 12

//Duty Cycle Reader
#define CYCLE_PIN 23

//Servo Pins
#define servoPin 35

//Limit Switch
#define LIM_SWITCH_PIN 11

#define TANK_MODE true
#define TANK_L 1
#define TANK_R 2
#define TANK_F 3


#define BIAS_DEG 10


/*
Content of Code will be divided into: 
- Initial Variables defined
- setup()
- helper functions()
- visionSystem functions
- action
  - Motors
  - Ultrasonic Sensors
  - Magnetic Sensors
*/
//Servo Motor Pin: Continuous 
using namespace ifx::tlx493d;
TLx493D_A1B6 mag(Wire, TLx493D_IIC_ADDR_A0_e);
Servo rackServo;

struct CoordinatePacket {
  float x_coord;
  float y_coord;
  int theta;
  bool isVisible;
};
struct MagnetoPacket {
  float x_mag;//Milli Gauss
  float y_mag;//Milli Gauss
  float z_mag;//Milli Gauss 
};
CoordinatePacket c_pack;
MagnetoPacket m_pack;
int USPinArr[6] = {echoPin1,trigPin1,echoPin2,trigPin2,echoPin3,trigPin3};//<-- Ignore for now... we might need mega
int motorPinArr[6] = {motor1pin1,motor1pin2,motor2pin1,motor2pin2,motor1en,motor2en};
int initAngle = 0;

//ZoneCounter
int zoneCounter = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  /*
  if(!mag.begin())
  {
    Serial.println("Magnetometer not starting");
    while(1);
  }
  */
  if(TANK_MODE)
  {
    Tank.begin();
    Serial.println("TANK_MODE Activated");
  }
  else
  {
    //Motor Setup
    for(int i=0;i<sizeof(motorPinArr)/sizeof(motorPinArr[0]);i++)
    {
      pinMode(motorPinArr[i],OUTPUT);//Every Pin is OUTPUT. En, pin1, pin2
      if(i<4)
        digitalWrite(motorPinArr[i],LOW);//<-- All Motors are turned off initially
    }
  }
  Serial.println("Running:");
  //pwm.begin();
  //pwm.setOscillatorFrequency();//Complete PWM  when done and also import laneZone() one. 
  visionSetup();
  Serial.println("Vision system successful");
  /*
  //Declaring Ultrasonic pins here
  for(int i=0;i<3;i++)
  {
    pinMode(USPinArr[2*i],INPUT);//Echo -> Input
    pinMode(USPinArr[2*i+1],OUTPUT);//Trigger -> Output
  }
  */
  rackServo.attach(servoPin);
  //Set up the magnetometer
  //dPin, Dir
  
}
//Vision Function
void visionSetup()
{
  char teamName[] = "Wall-E";
  Enes100.begin("Wall-E",DATA,teamMarker,1120,3,2);//; Vision System down...
  Enes100.println("Ambautakaum");
  //If (isConnected() + isVisible()) -> True? 
  if(Enes100.isConnected())
  {
    Serial.println("Initiated Connection: ");
    delay(100);
    Enes100.println("Wall-E Connected ");
    pingPacketData();
    delay(300);
    printPacketData();
    //This is honestly a bunch of garbage I made for our people to visualize the outputs. 
  }
}
//Helper Function /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Converts Radians to Degrees
int radToDeg(float radAng)
{
  return (int)((radAng*180)/M_PI); // angle * 180(deg) / PI
}
bool isMagnetic()
{
  return(m_pack.x_mag > 5||m_pack.y_mag > 5||m_pack.z_mag > 5);
}
//Vision System Functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Pings the Enes100 Server.
void pingPacketData()
{
  c_pack.x_coord = Enes100.getX();
  c_pack.y_coord = Enes100.getY();
  c_pack.theta = radToDeg(Enes100.getTheta());
  c_pack.isVisible = Enes100.isVisible();
}
//Prints the outputs of the server
void printPacketData()
{
  Serial.print("Initial Data recorded is as follows (x:");
  Serial.print(c_pack.x_coord);
  Serial.print(", y:");
  Serial.print(c_pack.y_coord);
  Serial.print(", θ(deg):");
  Serial.print(c_pack.theta);
  Serial.print(", isVisible:");
  Serial.print(c_pack.isVisible);
  Serial.print(")");
  Serial.println();
}
//Determines which direction OTV is facing
int currentDIR()
{
  //I want to define the directions. 
  int DIR = 0;
  if(c_pack.theta == 90)//I will enforce biases soon
  {
    DIR = FORWARD;
  }
  else if(c_pack.theta == 180)
  {
    DIR = LEFT;
  }
  else if(c_pack.theta == 0)
  {
    DIR = RIGHT;
  }
  return DIR;
} 
//Determines direction of the Goal Zone
int goalZoneDir()
{
  //At every point, the Orientation will use SMARTMAPPING
  //I wanna argue like, based on orientation
  if(c_pack.theta == 0)
    return FORWARD;
  else if(c_pack.theta == -90)
    return LEFT;
  else if(c_pack.theta == 90)
    return RIGHT;
}
///Motor Action Functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Overarching Motor Controller. 
void motorRun(int DIR, int speed)
{
  //DIR = Direction we're going
  //speed = At what speed?

  //Setting speeds of Motors
  analogWrite(motorPinArr[(sizeof(motorPinArr)/sizeof(motorPinArr[0]))-2],speed);
  analogWrite(motorPinArr[(sizeof(motorPinArr)/sizeof(motorPinArr[0]))-1],speed);
  if(TANK_MODE)
  {
    switch(DIR)
    {
      case FORWARD:
      {
        Tank.setMotorPWM(1, 255);
        Tank.setMotorPWM(2, 255);
        Tank.setMotorPWM(3, 255);
        Tank.setMotorPWM(4, 255);
        Tank.setRightMotorPWM(speed);
        Tank.setLeftMotorPWM(speed);
      }
      case LEFT:
      {
        Tank.setLeftMotorPWM(speed);
      }
      case RIGHT:
      {
        Tank.setRightMotorPWM(speed);
      }
    }
    return;
  }
  switch(DIR)
  {
    case FORWARD: {
      //configure Motor1
      //analogWrite(motorPinArr[(sizeof(motorPinArr)/sizeof(motorPinArr[0]))-2],speed);

      digitalWrite(motorPinArr[0],LOW);
      digitalWrite(motorPinArr[1],HIGH);//Switch if this is wheel is going reverse
      //Configure motor2
      //analogWrite(motorPinArr[(sizeof(motorPinArr)/sizeof(motorPinArr[0]))-1],speed);
      digitalWrite(motorPinArr[2],HIGH);//Also this is switched from other motor because 2 motors ==> normal|flipped
      digitalWrite(motorPinArr[3],LOW);//Switch if this is wheel is going reverse
      break;
    }
    case LEFT:
    {
      //Motor 1
      digitalWrite(motorPinArr[0],LOW);
      digitalWrite(motorPinArr[1],HIGH);

      digitalWrite(motorPinArr[2],LOW);
      digitalWrite(motorPinArr[3],HIGH);
    }
    case RIGHT:
    {
      digitalWrite(motorPinArr[0],HIGH);
      digitalWrite(motorPinArr[1],LOW);

      digitalWrite(motorPinArr[2],HIGH);
      digitalWrite(motorPinArr[3],LOW);
    }
  }
}
//Extra Functions for Motor turning: 

//Turns OTV until it reaches a set angle. 
void turnSet(int DIR, int angle, int speed = 100)
{
  //SMARTMAPPING to be added soon -> Guess which side is faster based on math and yields direction. 
  //Bias Range = ? 1-2 Deg?
  while(c_pack.theta != angle)//Has to be some kind of bias within a range or else it will have trouble stopping
  {
    motorRun(DIR,speed);
    pingPacketData();//Updates the Angle data
  }
}
//Turns 90 degrees in a direction L/R
void turnDirection(int DIR,int speed = 100)//Assumes 90 degrees
{
  //LEFT = I would turn left 90 deg
  //RIGHT = I would turn right 90 deg
  int initDeg = c_pack.theta;
  int degAddVal = 0;
  //Determines which way to turn. 
  if(DIR==LEFT)
    degAddVal = -90;
  else
    degAddVal = 90;
  int goalDeg = degAddVal+initDeg;//Initial Degree Count + 90|-90
  while(c_pack.theta!=goalDeg)
  {
    motorRun(DIR,speed);
  }
}
//Cuts power to all wheels.
void motorBreak()
{
  if(TANK_MODE)
  {
    Tank.turnOffMotors();
    return;
  }
  else
  {
    for(int i=0;i<4;i++)
    {
      digitalWrite(motorPinArr[i],LOW);
    }
  }
}
//Aruco should indicate mod 90 deg. Corrects OTV if not. 
void adjustAngle()
{

}
//Reads distance between the Ultrasonics
//DIR = Direction of Ultrasonic
//1 = FORWARD
//2 = RIGHT
//3 = LEFT

//Ultrasonic Sensor Function /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Reads the distance between the Ultrasonic in the LEFT/RIGHT/FORWARD direction
double readDistance(int DIR)
{ 
  if(TANK_MODE)
  {
    Enes100.print("Ultrasonic Reading: ");
    Enes100.println(Tank.readDistanceSensor(1));
    return;
  }
  //Trigger Pin: Direction * 2 - 1
    int trigPin = (DIR*2)+1;
    //Low to reset pin
    digitalWrite(USPinArr[trigPin],LOW);
    delayMicroseconds(2);
    //Reading it... 
    digitalWrite(USPinArr[trigPin],HIGH);
    delayMicroseconds(10);
    digitalWrite(USPinArr[trigPin],LOW);
    
    //Speed of sound 343m/s
    long duration = pulseIn(USPinArr[trigPin-1],HIGH);
    //2 is for back and forth reading
    return (double)(duration*0.0343/2);//Distance in cm
}
//Magnet Functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Detects magnetic fields. 
void readMagnet()
{
  double xMag,yMag,zMag;
  mag.getMagneticField(&xMag, &yMag, &zMag);
  //Magnetometer in X,Y,Z Coordinates
  m_pack.x_mag = xMag;
  m_pack.y_mag = yMag;
  m_pack.z_mag = zMag;
}
//Measures the duty Cycle
float readDutyCycle()
{
  //We could get the averages of the amount measured
  //
  int validCounts = 0,dutyCycle=0;
  for(int i=0;i<20;i++)
  {
    float highDur = pulseIn(CYCLE_PIN,HIGH);
    float lowDur = pulseIn(CYCLE_PIN,LOW);
    if(highDur!=0&&lowDur!=0)
    { 
      validCounts++;
      dutyCycle+= 100*(highDur/(highDur + lowDur));
    }
  }
  if(validCounts==0)
    return 0;
  return dutyCycle/validCounts;
}
//Picks up the Puck and measures Duty Cycle and Puck Magnetism 
void pickUp()
{
  //Continuously Rotate to lower -->
  //180 -> Rotate One way Forward
  //90 -> Stop
  //0 -> Rotate The other way
  while(digitalRead(LIM_SWITCH_PIN)==LOW)//Lim switch hasnt been hit. 
  {
    rackServo.write(180);
  }

  //Item gets picked up and is close enough for readings.
  //If at any point, if readMagnet() reads > (X an amount), -> break for loop
  //checks about 20 times to see if there is a magnet 
  for(int i=0;i<20;i++)
  {
    readMagnet();
    delayMicroseconds(100);
    if(isMagnetic())
      break;
  }
  if(isMagnetic())
  {
    //If anything is above 5 micro Gauss, --> There is a magnet
    
    int dutyCycle = readDutyCycle();
    Enes100.mission(MAGNETISM,MAGNETIC);
    Enes100.mission(CYCLE,dutyCycle);
  }
  else
  {
    Enes100.mission(MAGNETISM,NOT_MAGNETIC);
  }
  rackServo.write(90);
  //(Surely they were hoping to use a limit switch)
}
//Zone Functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Navigates the Landzone
void landZone()
{
  //Turn
  //Straight while within range
  //pick up
  //Turn Right
  //Rotate until The theta is correct.  
  //If above x coord, turn other way
  //else if below 
  //turnDirection()
  if(c_pack.y_coord > 1.0)//This means it's above the mid-point line
  {
    //Turn until Angle reached
    //Turn 90 deg angle
    turnSet(RIGHT,-90);
  }
  else
  {
    turnSet(LEFT,90);
  }
  //Move until the FORWARD Sensor reads 
  //10 is temporary, I need to see the size of the Rack and Pinion
  while(readDistance(FORWARD)>10)
  {
    motorRun(FORWARD,255);
  }
  //Distance from Rack and Pinion to Arduino
  //PickUp <-- I'll look into servo like rn
  pickUp();
  turnDirection(RIGHT);
  //If any m_pack coords are > biasAmount --> Enes Write IsMagnetic();
  zoneCounter+=1;
}
//Navigates the Obstacle Zone
void obsZone()
{
  while(readDistance(FORWARD)>10)
  {

  }
}
//Navigates the Open Zone
void openZone()
{

}
//test functions: /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void testCase1()
{
  pingPacketData();
  printPacketData();
}
//Testing the Duty Cycle Measurement
void testCase2()
{
  readDutyCycle();
}
//Forward Locomotion
void testCase3()
{
  motorRun(FORWARD,100);
}
//Turning 
void testCase4()
{
  motorRun(LEFT,100);
}
//Obstacle Sensing
void testCase5()
{
  readDistance(FORWARD);
}
//Mission Sensing
void testCase6()
{
  Serial.println(readDutyCycle());
}
//Avoiding one item
void testCase7()
{

}
//
//Loop /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  switch(zoneCounter)
  {
    case 0:
    {
      //landZone();
      testCase1();
      testCase3();
      testCase5();
      break;
    }
    case 1:
    {
      obsZone();
      break;
    }
    case 2:
    {
      openZone();
      break;
    }
  }
  //Functions I need: Move Forward
  //Map Update,Get,
  //Data Scan function
  //Overall it should be, move towards the Goal Zone... We can choose the Aruco Marker as the goal

  //Testing in lab goal
  //Vision System
  //Finalize Budget --> MEGA, Magnetometer Please, DC Motor(Buy 1 more)
  //Sell everything final.  
}

//Easy Route -> 2 Limit Switches.
//Hard Route -> Calculate amount of time in between