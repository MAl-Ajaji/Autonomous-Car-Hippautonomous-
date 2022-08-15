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

Pixy2 pixy;

//6 is 
int myPins[4] = {6, 5, 4, 3};
float deadZone = 0.30;
int baseSpeed = 30;
int index = 0;

Servo FrontServo; /////////////////////////////////////////////////////////////
Servo BackServo;////////////////////////////////////////////////////////////////
int servoAngle = 0;////////////////////////////////////////////////////////////////
int servoAngle2 = 80;///////////////////////////////////////////////////////////////

int curry = 0;
int cont = 0;
int turn; ////////////////////////////////////////////////////////
int sig, x, y, w, h;
float cx, cy, area;

void setup() {
  // put your setup code here, to run once:
  delay(5000);
  Serial.begin(115200);
  Serial.print("Start up initiated...\n");
  pixy.init();
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
  Serial.print("Turn: ");
  Serial.println(turn);
    Serial.print("CX: ");
  Serial.println(cx);
  if(turn > -deadZone && turn < deadZone)
  {
    turn = 0;
  }
  if(turn < 0)
  {
    moveRobot(1,-1);
    Serial.println("Left");
  }
  else if(turn > 0)
  {
    moveRobot(-1, 1);
    Serial.println("Right");
  }
  else if(turn == 0)
  {
    moveRobot(-1,-1);
    if (pixy.ccc.blocks[index].m_y >= 200 && pixy.ccc.blocks[index].m_y <= 207)
    {
      Serial.println(pixy.ccc.blocks[index].m_y);
      Serial.print("Forward ");
      Serial.println(index);
      captureBall();
    }
  }
  delay(1);
}

float pixyCheck()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  blocks = pixy.ccc.getBlocks();

  Serial.print("blocks");
  Serial.println(blocks);
index = 0; 
float besty = pixy.ccc.blocks[0].m_y;
  if(blocks >0){
      Serial.println("blocks>0");

for (int i =0; i< blocks-1; i++)
{
  //search for largest y
  curry = pixy.ccc.blocks[i].m_y;
  if(curry > besty){
  index = i;
  besty = curry;
  }
Serial.print("Loop: ");
Serial.println(i);


}
Serial.print("For loop: ");
Serial.println(index);
Serial.println(blocks);
if (pixy.ccc.blocks[index].m_y < 60){
 cont =0;
  cx =-999;
  delay(200);
  return cx;
}
  x = pixy.ccc.blocks[index].m_x;
  cx = mapfloat(x, 0, 315, -1, 1);
return cx;

}

  else
  {
    cont += 1;
    if(cont == 100)
    {
      cont = 0;
      cx = -999;
      delay(200);
    }
  }
  return cx;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return(float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}
;
void moveRobot(int leftSpeed, int rightSpeed)
{
  if(leftSpeed >= 0)
  {
    digitalWrite(myPins[0],1);
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
//    digitalWrite(myPins[0],0);
//    digitalWrite(myPins[1],1);
//    digitalWrite(myPins[2],0);
//    digitalWrite(myPins[3],1);
    delay(200); //drive car for 2 seconds (can change)

//Lower Front Gate 
  for (servoAngle = 100; servoAngle >= 0; servoAngle--)  //decrement the angle from 180 to 0, in steps of 1 
  {
    FrontServo.write(servoAngle);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }
//  cont = 0;
//  turn == -999;  
//return to ball searching
return;
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
