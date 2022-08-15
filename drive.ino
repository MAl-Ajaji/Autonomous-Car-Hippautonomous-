#include <PIDLoop.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <Pixy2I2C.h>
#include <Pixy2Line.h>
#include <Pixy2SPI_SS.h>
#include <Pixy2Video.h>
#include <TPixy2.h>

#include <Servo.h> ////////////////////////////////////////////////////////////////

#include <SPI.h>

Servo FrontServo; /////////////////////////////////////////////////////////////
Servo BackServo;////////////////////////////////////////////////////////////////
int servoAngle = 0;////////////////////////////////////////////////////////////////
int servoAngle2 = 80;///////////////////////////////////////////////////////////////

//Reference to Pixy
Pixy2 pixy;

//pins to motor driver
int myPins[4] = {6, 5, 4, 3};

//if in dead zone, car goes straight
float deadZone = 0.15;

//PWM ignore this
int baseSpeed = 30;

//global variables
int cont = 0;
int turn; ////////////////////////////////////////////////////////
int sig, x=0, y=0, w, h;
float cx, cy, area;


void setup() {
  // 5 second delay to start car after UNO receives power
  delay(5000);

  //Communication to Pixy2
  Serial.begin(19200);
  Serial.print("Start up initiated...\n");

  //initialize pixy
  pixy.init();

  //Set motor pins to output
  for(int i = 0; i < 4; i++)
  {
    pinMode(myPins[i], OUTPUT);
  }

  FrontServo.attach(9);  ///////////////////////////////////////////////////////////////
  BackServo.attach(10);  ///////////////////////////////////////////////////////////////
  FrontServo.write(servoAngle); ///////////////////////////////////////////////////////////////
  BackServo.write(servoAngle2); ///////////////////////////////////////////////////////////////
}

void loop() {
  // put your main code here, to run repeatedly:
  float turn = pixyCheck();

  //if ball is in dead zone
  if(turn > -deadZone && turn < deadZone)
  {
    turn = 0; 
    
    if(y>=200) ///////////////////////////////////////////////////////////////
    {
    captureBall(); ///////////////////////////////////////////////////////////////
    }
  }
  if(turn < 0)
  {
    moveRobot(1,-1);  //moveRobot(left speed, right speed)
  }
  else if(turn > 0)
  {
    moveRobot(-1, 1);
  }
  else if(turn == 0)
  {
    moveRobot(-1,-1);  //forwards
  }
  else if(turn == -999)
  {
    moveRobot(1,-1);
  }
  delay(1);
}

float pixyCheck()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

//get info from pixy bounding boxes
  blocks = pixy.ccc.getBlocks();

  if(blocks)
  {
    sig = pixy.ccc.blocks[0].m_signature;
    h = pixy.ccc.blocks[0].m_height; 
    w = pixy.ccc.blocks[0].m_width;
    x = pixy.ccc.blocks[0].m_x;
    y = pixy.ccc.blocks[0].m_y;
    //cx = (x + (w/2));
    //cy = (y + (h/2));
    cx = mapfloat(x, 0, 315, -1, 1);
    cy = mapfloat(y, 0, 207, 1, -1);
    area = w*h;  //does not do anything with area here
  }
  else
  {
    cont += 1;

    //does not see any balls, spin around to look for them
    if(cont == 100)
    {
      cont = 0;
      cx = -999;
    }
  }
  return cx;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return(float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void moveRobot(int leftSpeed, int rightSpeed)
{
  if(leftSpeed >= 0)
  {
   
    digitalWrite(myPins[0],1); //index myPins
    digitalWrite(myPins[1],0);
  }
  else
 
  {
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],1);
  }

  if(rightSpeed >= 0)
  
  {
    digitalWrite(myPins[2],1);
    digitalWrite(myPins[3],0);
  }
  else
  {
    digitalWrite(myPins[2],0);
    digitalWrite(myPins[3],1);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void captureBall()
{
//Raise front gate
   for(servoAngle = 0; servoAngle <= 100; servoAngle++)  //increment the angle from 0 to 180, in steps of 1
  { 
    FrontServo.write(servoAngle);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }
//drive car forward
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],1);
    digitalWrite(myPins[2],0);
    digitalWrite(myPins[3],1);
    delay(2000); //drive car for 2 seconds (can change)

//Lower Front Gate 
  for (servoAngle = 100; servoAngle >= 0; servoAngle--)  //decrement the angle from 180 to 0, in steps of 1 
  {
    FrontServo.write(servoAngle);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }
  cont = 0;
  turn == -999;                                           //return to ball searching
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void releaseBalls()
{

  //Lift Back Gate 
  for (servoAngle2 = 80; servoAngle2 >=15; servoAngle2--)   //decrement the angle from 180 to 0, in steps of 1 
  {
    BackServo.write(servoAngle2);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }
  
//drive car forward
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],1);
    digitalWrite(myPins[2],0);
    digitalWrite(myPins[3],1);
    delay(2000); //drive car for 2 seconds (can change)

//stop car
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],0);
    digitalWrite(myPins[2],0);
    digitalWrite(myPins[3],0);  

 //Lower Back Gate
   for(servoAngle2 = 15; servoAngle2 <= 80; servoAngle2++)   //increment the angle from 0 to 180, in steps of 1
  { 
    BackServo.write(servoAngle2);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }   
}
