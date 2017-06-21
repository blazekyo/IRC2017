// Code Originator: Delon Tan Wee Keat
// Clone on: 20/06/2017
// Source: https://github.com/blazekyo/IRC2017.git
// Modified: Chan Poh Hong
// Reason: To make the code into classes
// About this code: This is the code for the arm part of the autonomous 
//					robots made for IRC 2017 autonomous bot.

#include<Servo.h>

class Arms {
	private: 
		// Timer 
		long currentMilli;
		long previousMilli;
		
		// Servo initialization
		Servo leftFront;
		Servo rightFront;
		Servo leftBack;
		Servo rightBack;
		
		// Initial position
		int initialLArm;
		int initialRArm;
		int initialLHand;
		int initialRHand;
		
		// state position of the arm and hand
		// Due to robot settings calibration
		// The robot arm have three states which are 
		// 1. Lower ground state (intend to pick up ping-pong)
		// 2. Middle air state 
		// 3. Highest to the top state (prevent ping-pong from falling down)
		int maxArm = 120;
		int midArm = 90;
		int minArm = 0;
		int pos1Hand = 0;
		int pos2Hand = 70;
		int pos3Hand = 140;
		
		// Error in mechanical calibration compensation
		int error = 0;
		
		//Position
		int currentPosition;
		
	public:
		Arms(int LArm, int RArm, int LHand, int RHand);
		void armSetup(int lFront, int rFront, int lBack, int rBack);
		boolean pickThePP();
		boolean dropThePP();
};

// Global constructor
Arms::Arms(int LArm, int RArm, int LHand, int RHand){
	this->initialLArm = LArm;
	this->initialRArm = RArm;
	this->initialLHand = LHand;
	this->initialRHand = RHand;
	
	this->currentMilli = 0;
	this->previousMilli = 0;
	
	this->currentPosition = 0; // Resting
}

void Arms::armSetup(int lFront, int rFront, int lBack, int rBack)
  {
    //attach servo
    this->leftFront.attach(lFront);
    this->rightFront.attach(rFront);
    this->leftBack.attach(lBack);
    this->rightBack.attach(rBack);
    
    //initialize servo value
    this->rightFront.write(initialRHand);
    this->leftFront.write(initialLHand);
    this->rightBack.write(initialRArm);
    this->leftBack.write(initialLArm);    
  }

  //pick up pingPong Ball
  boolean Arms::pickThePP()
  {
    long period=0;
    boolean done=false;
    while(!done)
    {
    //handsUp
    if(this->currentPosition == 0)// if the arm is in resting position
    {
        //for(int i=pos2Hand; i<=pos3Hand ; i+=1)
        int i = pos2Hand;
        while(i<=pos3Hand)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - this->previousMilli;          
           
          //Before>> rightHand=40, leftHand=40
          //After >> rightHand=80, leftHand=0
    
          this->rightFront.write(i);
          this->leftFront.write(pos2Hand-(i-pos2Hand));
          
          if(period>60)
          {
            this->previousMilli = this->currentMilli;
            i+=1;            
          }
          //delay(60);
        }
        this->currentPosition++;//1st position
    }
    
    //armsUp-Mid
    else if(this->currentPosition == 1)//if the arm is in firstPosition
    {
        int i = minArm;
        //for(int i=minArm; i<=midArm ; i+=1)
        while(i<=midArm)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - this->previousMilli;   
           
          this->rightBack.write(maxArm-(i-error));
          this->leftBack.write(i);
          //delay(60);

          if(period>60)
          {
            this->previousMilli = this->currentMilli;   
            i+=1;        
          }
        }
        this->currentPosition++;             
    }
    //handsDown-mid
      if(this->currentPosition == 2)
      {
        Serial.println("handsDown");
        int i = pos3Hand;
          //for(int i=pos3Hand; i>=pos1Hand ; i-=1)
          while(i>=pos2Hand)
          {
            //time control
            this->currentMilli = millis();
            period = this->currentMilli - previousMilli;
          
            //Before>> rightHand=80, leftHand=0  
            //After >> rightHand=0, leftHand=80 
            this->rightFront.write(i);
            this->leftFront.write(pos3Hand-i);
            //delay(5);
            if(period>60)
            {
              this->previousMilli = this->currentMilli;            
              i-=1;
            }
          }
        this->currentPosition++;
      }
    //armsUp-Top
    else if(this->currentPosition == 3)//if the arm is in firstPosition
    {
        int i = midArm;
        //for(int i=minArm; i<=midArm ; i+=1)
        while(i<=maxArm)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - this->previousMilli;   
           
          this->rightBack.write(maxArm-(i-error));
          this->leftBack.write(i);
          //delay(60);

          if(period>60)
          {
            this->previousMilli = this->currentMilli;   
            i+=1;        
          }
        }
        this->currentPosition=0;
        done=true;              
    }
    }
    return done;
  }

  //drop pingPong Ball
  boolean Arms::dropThePP()
  {
    boolean done =false;
    long period =0;
    
    while(!done)
    {
      //handsDown
      if(this->currentPosition == 0)
      {
        Serial.println("handsDown");
        int i = pos2Hand;
          //for(int i=pos3Hand; i>=pos1Hand ; i-=1)
          while(i>=pos1Hand)
          {
            //time control
            this->currentMilli = millis();
            period = this->currentMilli - previousMilli;
          
            //Before>> rightHand=80, leftHand=0  
            //After >> rightHand=0, leftHand=80 
            this->rightFront.write(i);
            this->leftFront.write(pos3Hand-i);
            //delay(5);
            if(period>60)
            {
              this->previousMilli = this->currentMilli;            
              i-=1;
            }
          }
        this->currentPosition++;
      }
    
      //handsUp
      else if(this->currentPosition == 1)
      {
        Serial.println("handsUp");
        int i = pos1Hand;
          //for(int i=pos1Hand; i<=pos2Hand ; i+=1)
          while(i<=pos2Hand)
          {
            //time control
            this->currentMilli = millis();
            period = this->currentMilli - previousMilli;
                    
            //Before>> rightHand= 0, leftHand=80
            //After >> rightHand= 40, leftHand=40
              this->rightFront.write(i);
              this->leftFront.write(pos3Hand-i);
             //delay(60);

            if(period>60)
            {
              this->previousMilli = this->currentMilli;                
              i+=1;
            }
          }
          this->currentPosition++;
        }

      //arm down
      else if(this->currentPosition == 2)
      {
        Serial.println("armsDown");
        int i = maxArm;
        //for(int i=maxArm; i>=minArm ; i-=1)
        while(i>=minArm)
        {
          //time control
          this->currentMilli = millis();
          period = this->currentMilli - previousMilli;
                  
          this->rightBack.write(maxArm-(i-error));
          this->leftBack.write(i);
          //delay(60);
          if(period>60)
          {
            this->previousMilli = this->currentMilli;                
            i-=1;          
          }
        }
        this->currentPosition=0;
        done = true;
      } 
    } 
    return done; 
  }
