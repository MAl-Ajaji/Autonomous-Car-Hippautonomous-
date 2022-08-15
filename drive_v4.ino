#include <PIDLoop.h>
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <Pixy2I2C.h>
#include <Pixy2Line.h>
#include <Pixy2SPI_SS.h>
#include <Pixy2Video.h>
#include <TPixy2.h>

#include <Servo.h>
#include <SPI.h>

Pixy2 pixy;

//6 is 
int myPins[4] = {7, 5, 4, 2};
int pwmPins[2] = {6, 3};
int ledPins[2] = {11, 12};

float deadZone = 0.30;
int baseSpeed = 30;
int index = 0;
int flagcont= 0;
int spinning = 0;
int notfinding = 0;

int drivecount = 0;

int gate = 0;
int gatemax = 0;

Servo FrontServo; 
Servo BackServo;

int servoAngle = 0;
int servoAngle2 = 80;

int curry = 0;
int cont = 0;
int turn; 
int sig, x, y, w, h;
float cy, area;
float cx = 100;

void setup() {
  // put your setup code here, to run once:
  
  //Start the party
  pinMode(ledPins[0], OUTPUT);
  pinMode(ledPins[1], OUTPUT);

  delay(5000);
  
  Serial.begin(115200);
  pixy.init();
  for(int i = 0; i < 4; i++)
  {
    pinMode(myPins[i], OUTPUT);
  }

  pinMode(pwmPins[0], OUTPUT);
  pinMode(pwmPins[1], OUTPUT);

  
  FrontServo.attach(9);  
  BackServo.attach(10);  
  FrontServo.write(servoAngle); 
  BackServo.write(servoAngle2); 
}

void loop() {
  //Turn LEDs Red
  digitalWrite(ledPins[0],HIGH);
  digitalWrite(ledPins[1],LOW);
  
  // put your main code here, to run repeatedly:
  float turn = pixyCheck();

  //Check if time since program start is equal to five minutes
  if(millis() >= 90000)
  {
    GameOver();
    exit(0);
  }
  
  if (notfinding > 3) {
    GameOver();
    exit(0); //Kill it "Done"
  }
  else
  
  {
  if(turn > -deadZone && turn < deadZone)
  {
    turn = 0;
  }
  if(turn < 0)
  {
    analogWrite(pwmPins[0],200);//0-255
    analogWrite(pwmPins[1],200);
    moveRobot(1,-1);
  }
  else if(turn > 0)
  {
    analogWrite(pwmPins[0],200);//0-255
    analogWrite(pwmPins[1],200);
    moveRobot(-1, 1);
  }
  else if(turn == 0)
  {
    analogWrite(pwmPins[0],180);//0-255
    analogWrite(pwmPins[1],180);
    moveRobot(-1,-1);
    drivecount++;
    
    if (pixy.ccc.blocks[index].m_y >= 190 && pixy.ccc.blocks[index].m_y <= 207){
       if(pixy.ccc.blocks[index].m_signature == 2){
            captureBall();
       }
    }
  delay(1);
}
}
}

float pixyCheck(){
  
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  blocks = pixy.ccc.getBlocks();

  index = 0; 
  if(blocks >0){
     float besty = pixy.ccc.blocks[0].m_y;


      for (int i =0; i< blocks-1; i++){
        curry = pixy.ccc.blocks[i].m_y;    //search for largest y
        if(curry > besty)
        {
          index = i;
          besty = curry;
        }
      }
      if (pixy.ccc.blocks[index].m_y < 55){  //noise
        notfinding += 1;
        cont =0;
        cx =100;
        delay(200);
        return cx;
      }
      
      else if(pixy.ccc.blocks[index].m_signature == 2)
      {
        x = pixy.ccc.blocks[index].m_x;  // no noise
        cx = mapfloat(x, 0, 315, -1, 1);
        notfinding = 0;
        return cx;
      }

       else if(pixy.ccc.blocks[index].m_signature == 7 && pixy.ccc.blocks[index].m_y >= 190 && pixy.ccc.blocks[index].m_y <= 207){
            digitalWrite(myPins[0],0);
            digitalWrite(myPins[1],1);
            digitalWrite(myPins[2],1);
            digitalWrite(myPins[3],0);
            delay(1400);
        //     drive car forward
            analogWrite(pwmPins[0],180);//0-255
            analogWrite(pwmPins[1],180);
            digitalWrite(myPins[0],0);
            digitalWrite(myPins[1],1);
            digitalWrite(myPins[2],0);
            digitalWrite(myPins[3],1);
            delay(2400);
            cx= 100;
            return cx;
       }
     }

  else{
    cont += 1;
    if(cont == 100){
      notfinding += 1;
      cont = 0;
      cx = 100;
      delay(200);
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


///////////////////////////////////////////////////////////////////////////////////////////////////////////
void captureBall()
{
	//Turn LEDs Green
  digitalWrite(ledPins[0],LOW);
  digitalWrite(ledPins[1],HIGH);
  
//Raise front gate
    analogWrite(pwmPins[0],100);//0-255
    analogWrite(pwmPins[1],100);
   for(servoAngle = 0; servoAngle <= 100; servoAngle++)  //increment the angle from 0 to 180, in steps of 1
  { 
    FrontServo.write(servoAngle);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }

    delay(200); //drive car for 2 seconds (can change)
                                                          // if 
//Lower Front Gate 
  for (servoAngle = 100; servoAngle >= 0; servoAngle--)  //decrement the angle from 180 to 0, in steps of 1 
  {
    FrontServo.write(servoAngle);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
    
//
  }
  
  analogWrite(pwmPins[0],180);//0-255
  analogWrite(pwmPins[1],180);
  moveRobot(1,1);
  delay(300);
 
//return to ball searching

  for(int i = 0; i < drivecount;i++)
  {
    analogWrite(pwmPins[0],180);//0-255
    analogWrite(pwmPins[1],180);
    moveRobot(1,1);
    delay(29);
  }
  drivecount = 0;
  notfinding = 0;
  delay(10);
cx = 100;
return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void GameOver()
{
  //Turn LEDs to End Game
    digitalWrite(ledPins[0], HIGH);
    digitalWrite(ledPins[1], HIGH);

    
  //Lift Back Gate 

  
    analogWrite(pwmPins[0],220);//0-255
    analogWrite(pwmPins[1],220);
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],1);
    digitalWrite(myPins[2],1);
    digitalWrite(myPins[3],0);
    delay(2000);
    for (servoAngle2 = 80; servoAngle2 >=15; servoAngle2--)   //decrement the angle from 180 to 0, in steps of 1 
  {
    BackServo.write(servoAngle2);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }
//     drive car forward
    analogWrite(pwmPins[0],180);//0-255
    analogWrite(pwmPins[1],180);
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],1);
    digitalWrite(myPins[2],0);
    digitalWrite(myPins[3],1);
    delay(1500);
    analogWrite(pwmPins[0],220);//0-255
    analogWrite(pwmPins[1],220);
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],1);
    digitalWrite(myPins[2],1);
    digitalWrite(myPins[3],0);
    delay(1000);
    analogWrite(pwmPins[0],180);//0-255
    analogWrite(pwmPins[1],180);
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],1);
    digitalWrite(myPins[2],0);
    digitalWrite(myPins[3],1);
    delay(1500);

 //Lower Back Gate
   for(servoAngle2 = 15; servoAngle2 <= 80; servoAngle2++)   //increment the angle from 0 to 180, in steps of 1
  { 
    BackServo.write(servoAngle2);                        //set the servo angle and it will move to it
    delay(6);                                            //wait 6ms before moving to the next position
  }  
    digitalWrite(myPins[0],0);
    digitalWrite(myPins[1],0);
    digitalWrite(myPins[2],0);
    digitalWrite(myPins[3],0); 
}
