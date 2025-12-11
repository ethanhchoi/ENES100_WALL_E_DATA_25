#include <Servo.h>
#include "Enes100.h"
#include <math.h>
#include <Wire.h>

//Required downloads^^
//Main weird ones being the TLX + Enes100/Tank(these two are imported via zip downloads)

//Final Launch Protocol: {TANK_MODE:false,enVision:true,espRX:espTX:}
//Tank Protocol: {TANK_MODE:false,enVision:true,espRX:50,espTX:52}

//Protocol Defining Variables: 
#define enVision true


//How do we Turn?? You tell me because math isn't mathing. Unless we had a whole steering system, we don't know how to turn. 

//Defining Pins here
#define FORWARD 0
#define RIGHT 1
#define LEFT 2
#define NONE -1 // No Direction Specified 
#define BACKWARD 3

//These are the orientations given Defined by the Aruco Marker relative to the OTV
#define F_ORI -90
#define L_ORI 90
#define R_ORI -180

//Ultrasonic Pins 1 = FORWARD, 2 = Right, 3 = Left
#define echoPin1 8
#define trigPin1 9
#define echoPin2 10
#define trigPin2 11
#define echoPin3 12
#define trigPin3 13

#define motorSpeed 250

#define teamMarker 257

//L298N Motor Pins: 
#define motor1pin1 22
#define motor1pin2 24
#define motor1en 45
#define motor2pin1 26
#define motor2pin2 28
#define motor2en 44

//Duty Cycle Reader
#define CYCLE_PIN 7

//Servo Pins
#define servoPin 2

//Limit Switch
//#define LIM_SWITCH_PIN 11
//Bias Degree
#define BIAS_DEG 1
//Universal orientation: General Orientation for Forward
#define UNI_F 0
#define UNI_L 90
#define UNI_R -90
//Navigation Definitions
#define midpoint_y 1.0
#define bump_dist 10 //cm, the distance the otv reads before turning around

#define openPos 2.8

#define MAGNET_PIN_1 19
#define MAGNET_PIN_2 20
#define MAGNET_PIN_3 21

//Magnetic Sensor Pins: 
//POWER Pin: Pin powering the Magnetic Sensor
//Mag Documentation: https://arduino-xensiv-3d-magnetic-sensor-tlx493d.readthedocs.io/en/latest/api-ref.html

//ESP8826 Pins
#define espRX 50
#define espTX 52
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
- Navigation Sensors
- Zone Functions
*/
//Servo Motor Pin: Continuous 

Servo rackServo;

struct CoordinatePacket {
  float x_coord;
  float y_coord;
  int theta;
  bool isVisible;
};
CoordinatePacket c_pack;
int USPinArr[6] = {echoPin2,trigPin2,echoPin1,trigPin1,echoPin3,trigPin3};
int motorPinArr[6] = {motor1pin1,motor1pin2,motor2pin1,motor2pin2,motor1en,motor2en};
int initAngle = 0;
int DIR_OR[] = {FORWARD,LEFT,RIGHT};
int DEG_DIR[] = {F_ORI,L_ORI,R_ORI};
//ZoneCounter
int zoneCounter = 0;
int turnAmount = 0;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(MAGNET_PIN_1,INPUT);
  pinMode(MAGNET_PIN_2,INPUT);
  pinMode(MAGNET_PIN_3,INPUT);
  pinMode(CYCLE_PIN,INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //Motor Setup
  for(int i=0;i<sizeof(motorPinArr)/sizeof(motorPinArr[0]);i++)
  {
    pinMode(motorPinArr[i],OUTPUT);//Every Pin is OUTPUT. En, pin1, pin2
    if(i<4)
      digitalWrite(motorPinArr[i],LOW);//<-- All Motors are turned off initially
  }
  if(enVision)
  {
    visionSetup();
    Serial.println("Vision system successful");
  }
  //Declaring Ultrasonic pins here
  for(int i=0;i<3;i++)
  {
    pinMode(USPinArr[2*i],INPUT);//Echo -> Input
    pinMode(USPinArr[2*i+1],OUTPUT);//Trigger -> Output
  }
  rackServo.attach(servoPin);
  //Set up the magnetometer
  //dPin, Dir
  delay(2000);
}
//Vision Function
void visionSetup()
{
  char teamName[] = "Wall-E";
  Enes100.begin("Wall-E",DATA,teamMarker,1120,espTX,espRX);//; Vision System down...
  delay(100);
  if(Enes100.isConnected())
  {
    Enes100.println("Wall-E Connected ");
  }
}
//Helper Function /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Converts Radians to Degrees
int radToDeg(float radAng)
{
  return (int)((radAng*180)/M_PI); // angle * 180(deg) / PI
}
int cycleRange(int dutyCycle)
{
  int possibleRanges[5] = {10,30,50,70,90};
  //10 + 10||
  //Possible Duty Cycle
  int possibleCycle = 0;
  for(int i=0;i<sizeof(possibleRanges[0]);i++)
  {
    if(dutyCycle < possibleRanges[i]+ 10 || dutyCycle > possibleRanges[i] - 10)
    {
      //If it's within the range
      return possibleRanges[i];
      
    }
    //If it's 80 flat, or incrementally flat, No Bias. Just don't accept it. 
    //80 < 100 || 80 > 80
  }
  return 0;
  //42 + 10 > X
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
  for(int i=0;i<sizeof(DIR_OR)/sizeof(DIR_OR[0]);i++)
  {
    //Same Idea as if Direction + Bias > Current Angle > Direction - Bias
    //Ex: 90 + 2 > 90 > 90 - 88 
    //It's like 90 degrees
    Serial.print(c_pack.theta);Serial.print(">");Serial.print(DEG_DIR[i]);Serial.print("-");Serial.println(BIAS_DEG);
    Serial.print(c_pack.theta);Serial.print("<");Serial.print(DEG_DIR[i]);Serial.print("+");Serial.println(BIAS_DEG);    
    if(c_pack.theta > DEG_DIR[i] - BIAS_DEG && c_pack.theta < DEG_DIR[i] + BIAS_DEG)
    {
      return DIR_OR[i];
    }
  }
} 
//Determines direction of the Goal Zone
//Helpful in telling us how to orientate the OTV
int goalZoneDir()
{
  //At every point, the Orientation will use SMARTMAPPING
  //If facing forward => Forward
  //If facing left => Right
  //If facing right => Left
  if(c_pack.theta == UNI_F)
    return FORWARD;
  else if(c_pack.theta == UNI_L)
    return RIGHT;
  else if(c_pack.theta == UNI_R)
    return LEFT;
}
///Motor Action Functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Overarching Motor Controller. 
void motorRun(int DIR, int speed)
{
  //DIR = Direction we're going
  //speed = Speed we're going at
  switch(DIR)
  {
    case FORWARD: {
      //configure Motor1 to be both equal
      digitalWrite(motorPinArr[0], HIGH);
      digitalWrite(motorPinArr[1], LOW);
      digitalWrite(motorPinArr[2], HIGH);
      digitalWrite(motorPinArr[3], LOW);
      analogWrite(motorPinArr[4], speed);
      analogWrite(motorPinArr[5], speed);
      break;
    }
    case LEFT:
    {
      //Tells the motor to turn Left
      digitalWrite(motorPinArr[0],LOW);
      digitalWrite(motorPinArr[1],HIGH);
      digitalWrite(motorPinArr[2],HIGH);
      digitalWrite(motorPinArr[3],LOW);
      analogWrite(motorPinArr[4],speed);
      analogWrite(motorPinArr[5],speed);
      break;
    }
    case RIGHT:
    {
      //Tells the motor to turn right
      digitalWrite(motorPinArr[0],HIGH);
      digitalWrite(motorPinArr[1],LOW);
      digitalWrite(motorPinArr[2],LOW);
      digitalWrite(motorPinArr[3],HIGH);
      analogWrite(motorPinArr[4],speed);
      analogWrite(motorPinArr[5],speed);
      break;
    }
    case BACKWARD:
    {
      digitalWrite(motorPinArr[0], LOW);
      digitalWrite(motorPinArr[1], HIGH);
      digitalWrite(motorPinArr[2], LOW);
      digitalWrite(motorPinArr[3], HIGH);
      analogWrite(motorPinArr[4], speed);
      analogWrite(motorPinArr[5], speed);
    }
  }
}
//Extra Functions for Motor turning: 

//Turns OTV until it reaches a set angle. 
void turnSet(int DIR, int angle, int speed = 100)
{
  //SMARTMAPPING to be added soon -> Guess which side is faster based on math and yields direction. 
  //Bias Range = ? 1-2 Deg?
  //Set 0 DEgrees
  //3 >= 0 + 2 or 3 <= 0 - 2
  //c_pack.theta < angle + BIAS_DEG && c_pack.theta > angle - BIAS_DEG //Puts the Thing into range
  while(c_pack.theta > angle + BIAS_DEG || c_pack.theta < angle - BIAS_DEG)//Has to be some kind of bias within a range or else it will have trouble stopping
  {
    motorRun(DIR,speed);
    pingPacketData();
    delay(40);
  }
}
//Turns 90 degrees in a direction L/R
void turnDirection(int DIR,int speed = 100)//Assumes 90 degrees
{
  if(DIR!=LEFT||DIR!=RIGHT)
    return;//Safety mechanism ig
  //Should be 45 turn Direction
  ///backup Some amount
  //45 Again
  //Thought process: We either turn going L/R
  //wall detect
  pingPacketData();
  int currentTheta = c_pack.theta;
  int angleChange = 0;
  if(DIR==LEFT)
    angleChange = -45;
  else
    angleChange = 45;
  //Rotate until current_theta +- 45
  turnSet(DIR,currentTheta+angleChange,speed);//Turn From current position LEFT/RIGHT 45 Degrees
  delay(300);
  turnSet(DIR,currentTheta+angleChange,speed);//Turn From current position LEFT/RIGHT 45 Degrees
}
//Cuts power to all wheels.
void motorBreak()
{
  for(int i=0;i<4;i++)
  {
    digitalWrite(motorPinArr[i],LOW);
  }
  analogWrite(motorPinArr[4],0);
  analogWrite(motorPinArr[5],0);
}
//Aruco should indicate mod 90 deg. Corrects OTV if not. 
void adjustAngle()
{
  //Plan 1: Adjust to the closest value of ||-90|| ||0|| ||90||
  //Plan 2: Rotate to the desired rotation (pass in a value)
  if(c_pack.theta%90==0)
    return;  
  //Turn by this much until we hit this angle
  int offsetAngle = c_pack.theta%90;
  //-90 | 0 | 90
  //Adjust to be closest to be one of these Angles
  if(offsetAngle > 0)
  {
    //Adjust the offseted angle
    turnSet(RIGHT,offsetAngle);
  }
  //Calculate RAD/Deg // Per Sec

}

//Reads distance between the Ultrasonics
//DIR = Direction of Ultrasonic
//1 = FORWARD
//2 = RIGHT
//3 = LEFT

//Ultrasonic Sensor Function /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Reads the distance between the Ultrasonic in the LEFT/RIGHT/FORWARD direction
float readDistance(int DIR)
{ 
  //echo Pin: Direction * 2 - 1
  //
    int trigPin = (DIR*2)+1;
    //Low to reset pin
    digitalWrite(USPinArr[trigPin],LOW);
    delayMicroseconds(2);
    //Reading it... 
    digitalWrite(USPinArr[trigPin],HIGH);
    delayMicroseconds(10);
    digitalWrite(USPinArr[trigPin],LOW);
    
    //Speed of sound 343m/s
    long duration = pulseIn(USPinArr[trigPin-1],HIGH,30000);
    //2 is for back and forth reading
    float dist = (duration*0.0343*0.5);//Distance in cm
    if (dist == 0 || dist > 400) dist = 0;//Outlier distances
    return dist;
}
//Might consider making a function that tells me which side is blocked

//Magnet Functions /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Detects magnetic fields. 
bool isMagnetic()
{
  //Reads all magnets and sees if it detects a magnet. 
  return digitalRead(MAGNET_PIN_1)==LOW||digitalRead(MAGNET_PIN_2)==LOW||digitalRead(MAGNET_PIN_3)==LOW;
}
//Corrects the angle until the closest interval
void correctAngle(int angle = 90)
{
  //Turn amount should be like 90 - CurrentAngle; 
  
}
//Measures the duty Cycle
float readDutyCycle()
{
  //We could get the averages of the amount measured
  float dutyCycle=0;
  float highDur = pulseIn(CYCLE_PIN,HIGH,30000);
  float lowDur = pulseIn(CYCLE_PIN,LOW,30000);
  if(highDur!=0&&lowDur!=0)
  { 
    dutyCycle= 100*(highDur/(highDur + lowDur));
    return dutyCycle;
  }
  return 0;
}
//Tells me if the direction is cleared
bool dirIsClear(int DIR,int distance=bump_dist)
{
  //If Direction is smaller than a certain amount
  return readDistance(DIR) > distance;// distanceRead> 10cm
}
//Rack and Pinion//////////////////////////////////////////////////////////////////////////////////////////////
//Rack and Pinion go down with the servo
//Picks up the Puck and measures Duty Cycle and Puck Magnetism 
void pickUp()
{
  //Continuously Rotate to lower -->
  //180 -> Rotate Counterclockwise
  //90 -> Stop
  //0 -> Rotate Clockwise(downward)
  //Checks Magnetism
  rackServo.write(180); //Goes Down
  delay(1000);
  rackServo.write(90);//Stops
  
  for(int i=0;i<20;i++)
  {
    int readValue = readDutyCycle();
    if(readValue!=0)
    {
      int finalDutyCycleValue = cycleRange(readValue);
      //Print Duty Cycle here
      Enes100.println(finalDutyCycleValue);
    }
    delay(200); 
  }
  Enes100.println("Measuring If Puck is Magnetic");
  bool magStatus = false;
  for(int i=0;i<10;i++)
  {
    if(isMagnetic())
      magStatus = true;
      break;
    delay(400);
  }
  if(magStatus)
    Enes100.println("Puck is Magnetic");
  else
    Enes100.println("Puck is not magnetic");
  //Read Duty Cycle
  rackServo.write(40);//Goes back
  delay(2000); // Rotate for 2 seconds
}
//Nav Functions //////////////////////////////////////////////////////////////////////////////////////////////

//Moves Forward until it detects something
void forwardUntilDetect(int DIR=FORWARD,int dist=40)
{
  //Moves OTV forward until the direction indicated(most likely towards the center is cleared)
  motorRun(FORWARD,255);
  while(dirIsClear(DIR))
  {
    pingPacketData();
    //If The Ultrasonic sensor was passed in, detect if that side is opened
    delay(50);
  }
  motorBreak();
  
}
//Defaults to right if it's at the center
//Fixes the turning toward the center because our turning is ass
void turnToCenter()
{
  turnDirection(goalZoneDir());
}
void forwardSideDetect()
{
  //Move forward until the direction towards center is cleared
  bool successfulTurn = false;
  while(dirIsClear(FORWARD))//This should be changed to y position y > 0.0 OR y < 2.0
  {
    motorRun(FORWARD,150);
    delay(60);
    
    //If Direction towards goalzone is cleared up, 
    if(dirIsClear(goalZoneDir()))
    {
      //If the direction of the goalzone is blocked and front is blocked
      //180 and repeat
      turnToCenter();
      //forwardUntilDetect(goalZoneDir());
      if(turnAmount<2)
        turnAmount++;//I'm being lazy
      else
        zoneCounter++;//Switches to the next zone
      break;
    }
  }
}
void DIRUntilPosX(int DIR, int speed,float x_coord)
{
  while(c_pack.x_coord < x_coord)
  {
    pingPacketData();
    motorRun(FORWARD,speed);
    delay(60);
  }
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
  
  turnSet(RIGHT,0,30);//Faces "Straight", speed is slow
  //Backup until Go to x = 0.5
  pingPacketData();
  if(c_pack.x_coord > 0.5)
    DIRUntilPosX(BACKWARD,50,0.5);
  else if(c_pack.x_coord < 0.5)
    DIRUntilPosX(FORWARD,50,0.5);
  turnToCenter();
  
  forwardUntilDetect(FORWARD,15);
  //Distance from Rack and Pinion to Arduino
  pickUp();
  turnDirection(goalZoneDir());
  //If any m_pack coords are > biasAmount --> Enes Write IsMagnetic();
  zoneCounter+=1;
}
//Navigates the Obstacle Zone
void obsZone()
{
  //Moves forward until something is detected
  
  forwardUntilDetect(FORWARD,30);
  //Turns the direction towards the center point y > 1 or y < 1
  turnToCenter();
  //This should get the thing towards an open zone 
  forwardSideDetect();
  
}
//Navigates the Open Zone
void openZone()
{
  //If above Y Axis
  DIRUntilPosX(FORWARD,200,3.2);//Would be 2.8 but thats just math(it should be fine)
  if(c_pack.y_coord > midpoint_y)
  {
    turnDirection(RIGHT);
    while(c_pack.y_coord > midpoint_y)
    {
      motorRun(FORWARD,255);
    }
    turnDirection(LEFT);
    //Move until below y > 0.75
  }
  DIRUntilPosX(FORWARD,255,3.9);
}
//test functions: /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void testCase1()
{
  printPacketData();
  Serial.println(goalZoneDir());
}
//Testing the Duty Cycle Measurement
void testCase2()
{
  Serial.println(readDutyCycle());
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
  delay(500);
  motorRun(RIGHT,100);
}
//Obstacle Sensing
void testCase5()
{
  Serial.println(readDistance(FORWARD));
  delay(50);
}
//Magnet
void testCase6()
{
  Serial.println(isMagnetic());
}
//Avoiding one item
void testCase7()
{
  
}
//void checking which way I'm facing
void testCase8()
{
  if(currentDIR() == LEFT)
  {
    Serial.println("LEFT");
  }
  else if(currentDIR() == RIGHT)
  {
    Serial.println("RIGHT");
  }
  else if(currentDIR() == FORWARD)
  {
    Serial.println("FORWARD");
  }
}
void testCase9()
{
  
}
//
//Loop /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  switch(zoneCounter)
  {
    case 0:
    {
      //Put code into here
      //landZone();
      pickUp();
      break;
    }
    case 1:
    {
      //If at any point, the chasis reaches x > openzone -> stop
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
}
//Whoever is reading this
//Put all tests Case 0:{code here} Line 664
//Run testCase9() if you figure out the magnetometer
//testCase1(): To run this enable vision. 
//testCase2(): runDutyCycle if you wire the Dutycycle wires up
//testCase3(): The motor runs forward. 
//testCase4(): Motor turns both ways. //Only test this if testCase3 works.(If not try on your laptop and see if your wheel code works) 
//Change pins on #define if you want to modify this code
