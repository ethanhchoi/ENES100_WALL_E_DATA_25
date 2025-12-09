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
#define CYCLE_PIN 5

//Servo Pins
#define servoPin 2

//Limit Switch
#define LIM_SWITCH_PIN 11
//Bias Degree
#define BIAS_DEG 2
//Universal orientation: General Orientation for Forward
#define UNI_F 0
#define UNI_L 90
#define UNI_R -90
//Navigation Definitions
#define midpoint_y 1.0
#define bump_dist 10 //cm, the distance the otv reads before turning around

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
int USPinArr[6] = {echoPin1,trigPin1,echoPin2,trigPin2,echoPin3,trigPin3};//<-- Ignore for now... we might need mega
int motorPinArr[6] = {motor1pin1,motor1pin2,motor2pin1,motor2pin2,motor1en,motor2en};
int initAngle = 0;
int DIR_OR[] = {FORWARD,LEFT,RIGHT};
int DEG_DIR[] = {F_ORI,L_ORI,R_ORI};
//ZoneCounter
int zoneCounter = 0;
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
  Serial.println("Running:");
  if(enVision)
    visionSetup();
    Serial.println("Vision system successful");
  //Declaring Ultrasonic pins here
  for(int i=0;i<3;i++)
  {
    pinMode(USPinArr[2*i],INPUT);//Echo -> Input
    pinMode(USPinArr[2*i+1],OUTPUT);//Trigger -> Output
  }
  rackServo.attach(servoPin);
  //Set up the magnetometer
  //dPin, Dir
  
}
//Vision Function
void visionSetup()
{
  char teamName[] = "Wall-E";
  Enes100.begin("Wall-E",DATA,teamMarker,1120,espTX,espRX);//; Vision System down...
  Enes100.println("We Connected");
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
      for(int i=0;i<2;i++)
      {
        digitalWrite(motorPinArr[i],HIGH);
        digitalWrite(motorPinArr[i+1],LOW);
        analogWrite(motorPinArr[4+i],speed);
      }
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
      Serial.println("Turning Left");
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
      Serial.println("Turning Right");
      break;
    }
    case BACKWARD:
    {
      digitalWrite(motorPinArr[0], LOW);
      digitalWrite(motorPinArr[1], HIGH);
      digitalWrite(motorPinArr[2], LOW);
      digitalWrite(motorPinArr[3], HIGH);
  }
}
//Extra Functions for Motor turning: 

//Turns OTV until it reaches a set angle. 
void turnSet(int DIR, int angle, int speed = 100)
{
  //SMARTMAPPING to be added soon -> Guess which side is faster based on math and yields direction. 
  //Bias Range = ? 1-2 Deg?
  
  while(c_pack.theta >= angle + BIAS_DEG || c_pack.theta <= angle - BIAS_DEG)//Has to be some kind of bias within a range or else it will have trouble stopping
  {
    pingPacketData();
    motorRun(DIR,speed);
  }
  adjustAngle();
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
  //While the orientation is not facing goal direction
  while(c_pack.theta!=goalDeg)
  {
    motorRun(DIR,speed);
  }
}
//Cuts power to all wheels.
void motorBreak()
{
  for(int i=0;i<4;i++)
  {
    digitalWrite(motorPinArr[i],LOW);
  }
}
//Aruco should indicate mod 90 deg. Corrects OTV if not. 
void adjustAngle()
{
  //Plan 1: Adjust to the closest value of ||-90|| ||0|| ||90||
  //Plan 2: Rotate to the desired rotation (pass in a value)
  if(c_pack.theta%90==0)
    return;  
 //math.abs(-90 - c_pack.theta)
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
double readDistance(int DIR)
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
    return (double)(duration*0.0343/2);//Distance in cm
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
  int validCounts = 0,dutyCycle=0;
  float highDur = pulseIn(CYCLE_PIN,HIGH);
  float lowDur = pulseIn(CYCLE_PIN,LOW);
  if(highDur!=0&&lowDur!=0)
  { 
    dutyCycle+= 100*(highDur/(highDur + lowDur));
    return dutyCycle/validCounts;
  }
  return 0;
}
//Tells me if the direction is cleared
bool dirIsClear(int DIR)
{
  //If Direction is smaller than a certain amount
  return readDistance(DIR) > bump_dist;
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
  for(int i=0;i<4;i++)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);
    delay(100); 
  }
  
}

//Nav Functions //////////////////////////////////////////////////////////////////////////////////////////////

//Moves Forward until it detects something
void forwardUntilDetect(int DIR=FORWARD)
{
  //Moves OTV forward until the direction indicated(most likely towards the center is cleared)
  while(dirIsClear(DIR))
  {
    //If The Ultrasonic sensor was passed in, detect if that side is opened
    motorRun(FORWARD,255);
    delay(50);
  }
}
//Turns the OTV in the direction of where the center point is
//Defaults to right if it's at the center
void turnToCenter()
{
  pingPacketData();
  if(c_pack.y_coord > midpoint_y)//Above midpoint line
  {
    turnSet(RIGHT,-90);
  }
  else
  {
    turnSet(LEFT,90);
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
  pingPacketData();
  if(c_pack.y_coord > midpoint_y)//Above midpoint line
  {
    turnSet(RIGHT,-45);
    motorRun(BACKWARD,
    turnSet(RIGHT,-90);
  }
  else
  {
    turnSet(LEFT,45);
    //backward();
    turnSet(LEFT,90);
  }
  
  
  forwardUntilDetect();
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
  forwardUntilDetect();
  //Turns the direction towards the center point y > 1 or y < 1
  turnToCenter();
  //Move forward until the direction towards center is cleared
  forwardUntilDetect(goalZoneDir());
  //Either both sides are blocked or 
  /*
  if(!dirIsClear(goalZoneDir()) && !dirIsClear(FRONT))
  {
    //If the direction of the goalzone is blocked and front is blocked
    //180 and repeat
    turnSet(goalZoneDir(),180);
    forwardUntilDetect(goalZoneDir());
  }
  */
}
//Navigates the Open Zone
void openZone()
{
  //If above Y Axis
  if(c_pack.y_coord > 0.75)
  {
    turnDirection(RIGHT);
    while(c_pack.y_coord > 0.75)
    {
      motorRun(FORWARD,255);
    }
    turnDirection(LEFT);
    //Move until below y > 0.75
  }
  //MoveOverLog
  while(c_pack.x_coord < 3.76)
  {
    motorRun(FORWARD,255);
  }
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
      landZone();
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
