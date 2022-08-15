//#include <PIDLoop.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <Pixy2I2C.h>
#include <Pixy2Line.h>
#include <Pixy2SPI_SS.h>
#include <Pixy2Video.h>
#include <TPixy2.h>


#include <SPI.h>

Pixy2 pixy;

int myPins[4] = {6, 5, 4, 3};
float deadZone = 0.15;
int baseSpeed = 30;

int cont = 0;
int sig, x, y, w, h;
float cx, cy, area;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Start up initiated...\n");
  pixy.init();
  for(int i = 0; i < 4; i++)
  {
    pinMode(myPins[i], OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  static int16_t index = -1;
  Block *block=NULL;
  
  pixy.ccc.getBlocks();
  
  if (index==-1) // search....
  {
    Serial.println("Searching for block...");
    index = findBlock();
    if (index>=0)
      Serial.println("Found block!");
 }
  // If we've found a block, find it, track it
  if (index>=0)
     block = trackBlock(index);

  
  float turn = pixyCheck(block);
  if(turn > -deadZone && turn < deadZone)
  {
    turn = 0;
  }
  if(turn < 0)
  {
    moveRobot(1,-1);
  }
  else if(turn > 0)
  {
    moveRobot(-1, 1);
  }
  else
  {
    moveRobot(-1,-1); 
  }
  delay(1);
}

float pixyCheck(Block* block)
{
  static int i = 0;
  int j;
  char buf[32];

  if(block)
  {
    sig = block->m_signature;
    h = block->m_height;
    w = block->m_width;
    x = block->m_x;
    y = block->m_y;
    cx = (x + (w/2));
    cy = (y + (h/2));
    cx = mapfloat(cx, 0, 320, -1, 1);
    cy = mapfloat(cy, 0, 200, 1, -1);
    area = w*h;
  }
  else
  {
    cont += 1;
    if(cont == 100)
    {
      cont = 0;
      cx = 0;
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

int16_t  findBlock()
{
  if(pixy.ccc.numBlocks && pixy.ccc.blocks[0].m_age>30)
    return pixy.ccc.blocks[0].m_index;
    
  return -1;
}

Block *trackBlock(uint8_t index)
{
  uint8_t i;
  
  for(i=0; i<pixy.ccc.numBlocks; i++)
  {
    if(index==pixy.ccc.blocks[i].m_index)
      return &pixy.ccc.blocks[i];
  }
  return NULL;
}
