// Code Originator: Delon Tan Wee Keat
// Clone on: 20/06/2017 5.00PM
// Source: https://github.com/blazekyo/IRC2017.git
// Modified: Chan Poh Hong
// Reason: To make the code into classes
// About this code: This is the code for the Main class of the autonomous 
//                  robots made for IRC 2017 autonomous bot.

#include "Autobot_arm.h"
#include "Autobot_wheel.h"
#include "Autobot_settings.h"

int stateNo=0;
boolean done=false;
void setup() 
{
  // put your setup code here, to run once:
  //armPin :: lFront, rFront, lBack, rBack
  botArm->armSetup(15,14,18,17);
  
  //walkSpeed, turnSpeed, walk-distance, spin-degree
  //speed-distance ratio = 111.1 when speed = 70pwm - 1 cm
  botLeg->legSetup(35,20,120.5,32);
  /*commands:
   * forward : the bot move forward
   * backward : the bot move backward
   * left : the bot rotate anti-clockwise
   * right : the bot rotate clockwise
   * stopMove : the bot stop
   * pickUp : the bot pick up ping pong balls
   * drop : the bot drop ping pong balls
   */
  //msequence->package4("forward",60,"stopMove",1000000,"backward",20000,"stopMove",100);
  //walk in a square clockwise
  //msequence->package18("forward",80,"stopMove",1000,"left",90,"forward",230,"stopMove",1000,"left",90,"stopMove",1000,"drop",0,"forward",35.5,"stopMove",1000,"pickUp",0,"backward",35.5,"stopMove",1000,"left",90,"forward",280,"stopMove",10000,"drop",0,"pickUp",0);
  msequence->package3("stopMove",2000,"drop",0,"pickUp",0);
  
  //msequence->package8("forward",10000,"left",90,"forward",10000,"left",90,"forward",10000,"left",90,"forward",10000,"left",90);
  
  //pidController
  LSetpoint = 0;
  RSetpoint = 0;
  myPIDL.SetMode(AUTOMATIC);
  myPIDL.SetSampleTime(30);
  myPIDR.SetMode(AUTOMATIC);
  myPIDR.SetSampleTime(30);

  //encoder
  encoderInit();
  Serial.begin(9600);
}

void loop() 
{
  // put your main code here, to run repeatedly:

  /*Serial.print("Pulse:");
  Serial.println(Rwspeed);
  Rwspeed = 0;
  delay(100);*/

  //Serial.println(abs_Rwspeed);
  myPIDL.SetOutputLimits(0,255);
  myPIDL.SetOutputLimits(0,255);
  myPIDR.SetOutputLimits(0,255);
  myPIDR.SetOutputLimits(0,255);
  
  if(msequence->getState(stateNo) == "forward")
  {
    LSetpoint = botLeg->getWalkSpeed();
    RSetpoint = botLeg->getWalkSpeed();
    done = botLeg->moveForward(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));

    Serial.println("forward");
  }
  else if(msequence->getState(stateNo) == "backward")
  {
    LSetpoint = botLeg->getWalkSpeed();
    RSetpoint = botLeg->getWalkSpeed();
    done = botLeg->moveBackward(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));
    Serial.println("backward");
  }
  else if(msequence->getState(stateNo) == "left")
  {
    LSetpoint = botLeg->getTurnSpeed();
    RSetpoint = botLeg->getTurnSpeed();
    done = botLeg->rotateClockWise(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));
    Serial.println("left");
  }
  else if(msequence->getState(stateNo) == "right")
  {
    LSetpoint = botLeg->getTurnSpeed();
    RSetpoint = botLeg->getTurnSpeed();    
    done = botLeg->rotateAntiClockWise(&LpidOutput,&RpidOutput,&Lwspeed,&Rwspeed,&abs_Lwspeed,&abs_Rwspeed,myPIDL,myPIDR,msequence->getAmp(stateNo));
    Serial.println("right");
  }
  else if(msequence->getState(stateNo) == "pickUp")
  {
    done = botArm->pickThePP();
    botLeg->resetTimer();
    Serial.println("pickup");
  }
  else if(msequence->getState(stateNo) == "drop")
  {
    done = botArm->dropThePP();
    botLeg->resetTimer();
    Serial.println("drop");
  }
  else if(msequence->getState(stateNo) == "stopMove")
  {
    done = botLeg->stopMove(msequence->getAmp(stateNo));
    Serial.println("stop");
  }
  
  if(done)
  {
    Serial.println("done");
    stateNo++;
    done=false;
    myPIDL.SetOutputLimits(0.0, 1.0);
    myPIDL.SetOutputLimits(-1.0, 0.0);
    myPIDR.SetOutputLimits(0.0, 1.0);
    myPIDR.SetOutputLimits(-1.0, 0.0);
    Lwspeed=0;
    Rwspeed=0;
  }
  
  if(stateNo>=msequence->getPhases())
  {
    //repeat all over again
    stateNo=0;
  }
}
