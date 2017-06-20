#include<Servo.h>
#include<PID_v1.h>

//Arm
class Arms
{
  private:
  //timer
    long currentMilli;
    long previousMilli;
    
  //servo
    Servo leftFront;
    Servo rightFront;
    Servo leftBack;
    Servo rightBack;

  //initialPosition
    int initialLArm;
    int initialRArm;
    int initialLHand;
    int initialRHand;

  //statePosition
    int maxArm = 120;
    int midArm = 90;
    int minArm = 0;
    int pos1Hand = 0;
    int pos2Hand = 70;
    int pos3Hand = 140;

  //Arm Error
    int error = 0;
  
  //Position
    int currentPosition;
    
  public:
    Arms(int LArm, int RArm, int LHand, int RHand);
    void armSetup(int lFront, int rFront, int lBack, int rBack);
    boolean pickThePP();
    boolean dropThePP();
};
    
  //constructor
  Arms::Arms(int LArm, int RArm, int LHand, int RHand)
  {
    this->initialLArm = LArm;
    this->initialRArm = RArm;
    this->initialLHand = LHand;
    this->initialRHand = RHand;

    this->currentMilli=0;
    this->previousMilli=0;

    this->currentPosition = 0; //resting
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
//Legs , exempted PID
class Legs
{
  //driverPins
  private:
    int d1Pwm;
    int d1Dir;
    int d2Pwm;
    int d2Dir;
    
  //timer
    long currentMilli;
    long previousMilli;

  //default
    int walkSpeed = 0;
    int turnSpeed = 0;
    double speedOdistance = 0;
    double speedOdegree = 0;
  
    long walkTime = 0;
    long spinTime = 0;  

  public:
    Legs(int pwm1, int dir1, int pwm2, int dir2);
    void legSetup(double wspeed, double tspeed, double speed_distance, double speed_degree);
    void setWalkSpeed(double wspeed);
    double getWalkSpeed();
    void setSpeedODistance();
    void setTurnSpeed(double tspeed);
    double getTurnSpeed();
    void setSpeedODegree();
    void setWalkTime(double wtime);
    void setSpinTime(double stime);
    void resetTimer();
    boolean moveForward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance);
    boolean moveBackward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance);
    boolean rotateClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree);
    boolean rotateAntiClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree);
    boolean stopMove(long stopTime); 
};
  //constructor
  Legs::Legs(int pwm1, int dir1, int pwm2, int dir2)
  {
    this->d1Pwm = pwm1;
    this->d1Dir = dir1;
    this->d2Pwm = pwm2;
    this->d2Dir = dir2;

    currentMilli=0;
    previousMilli=0;
  }

  void Legs::resetTimer()
  {
    this->previousMilli = millis();
  }

  void Legs::legSetup(double wspeed, double tspeed, double speed_distance, double speed_degree)
  {
    pinMode(this->d1Pwm,OUTPUT);
    pinMode(this->d1Dir,OUTPUT);
    pinMode(this->d2Pwm,OUTPUT);
    pinMode(this->d2Dir,OUTPUT);

    this->setWalkSpeed(wspeed);
    this->setTurnSpeed(tspeed);
    this->speedOdistance = speed_distance;
    this->speedOdegree = speed_degree;
  }
   
  void Legs::setWalkSpeed(double wspeed)
  {
    this->walkSpeed = wspeed;
  }

  double Legs::getWalkSpeed()
  {
    return this->walkSpeed;
  }

  void Legs::setSpeedODistance()
  {
    switch(this->walkSpeed)
    {
      case 0:
        this->speedOdistance = 0;
        break;
      //other options, for further implementation
      default:
        break;
    }
  }
  
  void Legs::setTurnSpeed(double tspeed)
  {
    this->turnSpeed = tspeed;
  }

  double Legs::getTurnSpeed()
  {
    return this->turnSpeed;
  }

  void Legs::setSpeedODegree()
  {
    switch(this->turnSpeed)
    {
      case 0 :
        this->speedOdegree = 0;
        break;
      //other options, for further implementation
      default:
        break;
    }
  }

  void Legs::setWalkTime(double wtime)
  {
    this->walkTime = wtime;
  }

  void Legs::setSpinTime(double stime)
  {
    this->spinTime = stime;
  }
  
  boolean Legs::moveForward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult=false;
    boolean RconversionResult=false;
    boolean done = false;
    
    //computeTimeTravel
    timeTravel = distance*this->speedOdistance;
    this->currentMilli = millis();
    if(this->currentMilli - this-> previousMilli<=timeTravel)
    {
         //left driver
         analogWrite(d1Pwm,*LpidOutput);
         digitalWrite(d1Dir,LOW);
         //right driver
         analogWrite(d2Pwm,*RpidOutput);
         digitalWrite(d2Dir,LOW);
              
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);
        
        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      }
       
    }
    else
    {
        this->previousMilli = this->currentMilli;
        //stop
        analogWrite(d1Pwm,0);
        analogWrite(d2Pwm,0);
        
        Serial.println("reset");
        done=true;
    }

     return done;
  }

  boolean Legs::moveBackward(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long distance)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult=false;
    boolean RconversionResult=false;
    boolean done=false;
    
    //computeTimeTravel
    timeTravel = distance*this->speedOdistance;
    
    this->currentMilli = millis();
    period = this->currentMilli - this->previousMilli;
    
      if(this->currentMilli - this->previousMilli<=timeTravel)
      {
         //left driver
         analogWrite(d1Pwm,*LpidOutput);
         //analogWrite(d1Pwm,80);
         digitalWrite(d1Dir,HIGH);
         //right driver
         analogWrite(d2Pwm,*RpidOutput);
         //analogWrite(d2Pwm,80);
         digitalWrite(d2Dir,HIGH);
        
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);

        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      }
      }
      else
      {
         this->previousMilli = this->currentMilli;
        //stop
        analogWrite(d1Pwm,0);
        analogWrite(d2Pwm,0);
                 
         done = true;
      }

      return done;
  }

  boolean Legs::rotateClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult;
    boolean RconversionResult;
    boolean done=false;
    
    //computeTimeTravel
    timeTravel = degree*this->speedOdegree;

    this->currentMilli = millis();
    period = this->currentMilli - previousMilli;
    
      if(period<=timeTravel)
      {
         //left driver
         analogWrite(this->d1Pwm,*LpidOutput);
         digitalWrite(this->d1Dir,HIGH);
         //right driver
         analogWrite(this->d2Pwm,*RpidOutput);
         digitalWrite(this->d2Dir,LOW);
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);

        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      } 
      }
      else
      {
         this->previousMilli = this->currentMilli;
         //stop
         analogWrite(d1Pwm,0);
         analogWrite(d2Pwm,0);
                 
         done = true;
      }   

      return done;
  }

  boolean Legs::rotateAntiClockWise(double *LpidOutput, double *RpidOutput, double *Lwspeed, double *Rwspeed, double *abs_Lwspeed, double *abs_Rwspeed, PID &myPIDL, PID &myPIDR,  long degree)
  {
    //initialize time control
    long timeTravel = 0;
    long period = 0;
    boolean LconversionResult;
    boolean RconversionResult;
    boolean done = false;
    //computeTimeTravel
    timeTravel = degree*this->speedOdegree;

    this->currentMilli = millis();
    period = this->currentMilli - previousMilli;
    
      if(period<=timeTravel)
      {
         //left driver
         analogWrite(this->d1Pwm,*LpidOutput);
         digitalWrite(this->d1Dir,LOW);
         //right driver
         analogWrite(this->d2Pwm,*RpidOutput);
         digitalWrite(this->d2Dir,HIGH);
        *abs_Lwspeed = abs(*Lwspeed);
        *abs_Rwspeed = abs(*Rwspeed);

        LconversionResult = myPIDL.Compute(); //pid.Compute() starts the conversion
        RconversionResult = myPIDR.Compute();

      if(LconversionResult && RconversionResult)
      {
        *Lwspeed = 0;
        *Rwspeed = 0;
      } 
     }
     else
     {
         this->previousMilli = this->currentMilli;
         //stop
         analogWrite(d1Pwm,0);
         analogWrite(d2Pwm,0);
                 
         done=true;
     }

     return done;
  }

  boolean Legs::stopMove(long stopTime)
  {
    boolean done=false;
    this->currentMilli = millis();
    if(this->currentMilli - this->previousMilli<=stopTime)
    {
      //left driver
      analogWrite(this->d1Pwm,0);
      //right driver
      analogWrite(this->d2Pwm,0); 
    }
    else
    {
      this->previousMilli = this->currentMilli;
      done=true;
    }

    return done;
  }

class Sequencer
{
  private:
    String state2[2];
    String state3[3];
    String state4[4];
    String state5[5];
    String state6[6];
    String state7[7];
    String state8[8];
    String state9[9];
    String state10[10];
    String state11[11];
    String state12[12];
    String state13[13];
    String state14[14];
    String state15[15];
    String state16[16];
    String state17[17];
    String state18[18];
    
    int phases;
    
    int amp2[2]; //amplitute
    int amp3[3];
    int amp4[4];
    int amp5[5];
    int amp6[6];
    int amp7[7];
    int amp8[8];
    int amp9[9];
    int amp10[10];
    int amp11[11];
    int amp12[12];
    int amp13[13];
    int amp14[14];
    int amp15[15];
    int amp16[16];
    int amp17[17];
    int amp18[18];

  public:
    void package18(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift, String stateSixt, int ampSixt, String stateSevent, int ampSevent, String stateEightt, int ampEightt);  
    void package17(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift, String stateSixt, int ampSixt, String stateSevent, int ampSevent);
    void package16(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift, String stateSixt, int ampSixt);  
    void package15(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift);
    void package14(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt);
    void package13(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt);
    void package12(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve);
    void package11(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven);
    void package10(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen);
    void package9(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine);
    void package8(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight);
    void package7(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven);
    void package6(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix);
    void package5(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive);
    void package4(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour);
    void package3(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree);
    void package2(String stateOne, int ampOne, String stateTwo, int ampTwo);
    void setPhases(int _phases);
    String getState(int stateNumber);
    int getAmp(int stateNumber);
    int getPhases();
};

  void Sequencer::package18(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift, String stateSixt, int ampSixt, String stateSevent, int ampSevent, String stateEightt, int ampEightt)
  {
    this->state18[0] = stateOne;
    this->state18[1] = stateTwo;
    this->state18[2] = stateThree;
    this->state18[3] = stateFour;
    this->state18[4] = stateFive;
    this->state18[5] = stateSix;
    this->state18[6] = stateSeven;
    this->state18[7] = stateEight;
    this->state18[8] = stateNine;
    this->state18[9] = stateTen;
    this->state18[10] = stateEleven;
    this->state18[11] = stateTwelve;
    this->state18[12] = stateThirt;
    this->state18[13] = stateFourt;
    this->state18[14] = stateFift;
    this->state18[15] = stateSixt;
    this->state18[16] = stateSevent;
    this->state18[17] = stateEightt;
    
    this->phases = 18;

    this->amp18[0] = ampOne;
    this->amp18[1] = ampTwo;
    this->amp18[2] = ampThree;
    this->amp18[3] = ampFour;
    this->amp18[4] = ampFive;
    this->amp18[5] = ampSix;
    this->amp18[6] = ampSeven;
    this->amp18[7] = ampEight;
    this->amp18[8] = ampNine;
    this->amp18[9] = ampTen;
    this->amp18[10] = ampEleven;
    this->amp18[11] = ampTwelve;
    this->amp18[12] = ampThirt;
    this->amp18[13] = ampFourt;
    this->amp18[14] = ampFift;
    this->amp18[15] = ampSixt;
    this->amp18[16] = ampSevent;
    this->amp18[17] = ampEightt;
  }
  

  void Sequencer::package17(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift, String stateSixt, int ampSixt, String stateSevent, int ampSevent)
  {
    this->state17[0] = stateOne;
    this->state17[1] = stateTwo;
    this->state17[2] = stateThree;
    this->state17[3] = stateFour;
    this->state17[4] = stateFive;
    this->state17[5] = stateSix;
    this->state17[6] = stateSeven;
    this->state17[7] = stateEight;
    this->state17[8] = stateNine;
    this->state17[9] = stateTen;
    this->state17[10] = stateEleven;
    this->state17[11] = stateTwelve;
    this->state17[12] = stateThirt;
    this->state17[13] = stateFourt;
    this->state17[14] = stateFift;
    this->state17[15] = stateSixt;
    this->state17[16] = stateSevent;
    
    this->phases = 17;

    this->amp17[0] = ampOne;
    this->amp17[1] = ampTwo;
    this->amp17[2] = ampThree;
    this->amp17[3] = ampFour;
    this->amp17[4] = ampFive;
    this->amp17[5] = ampSix;
    this->amp17[6] = ampSeven;
    this->amp17[7] = ampEight;
    this->amp17[8] = ampNine;
    this->amp17[9] = ampTen;
    this->amp17[10] = ampEleven;
    this->amp17[11] = ampTwelve;
    this->amp17[12] = ampThirt;
    this->amp17[13] = ampFourt;
    this->amp17[14] = ampFift;
    this->amp17[15] = ampSixt;
    this->amp17[16] = ampSevent;
  }

  void Sequencer::package16(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift, String stateSixt, int ampSixt)
  {
    this->state16[0] = stateOne;
    this->state16[1] = stateTwo;
    this->state16[2] = stateThree;
    this->state16[3] = stateFour;
    this->state16[4] = stateFive;
    this->state16[5] = stateSix;
    this->state16[6] = stateSeven;
    this->state16[7] = stateEight;
    this->state16[8] = stateNine;
    this->state16[9] = stateTen;
    this->state16[10] = stateEleven;
    this->state16[11] = stateTwelve;
    this->state16[12] = stateThirt;
    this->state16[13] = stateFourt;
    this->state16[14] = stateFift;
    this->state16[15] = stateSixt;
    
    this->phases = 16;

    this->amp16[0] = ampOne;
    this->amp16[1] = ampTwo;
    this->amp16[2] = ampThree;
    this->amp16[3] = ampFour;
    this->amp16[4] = ampFive;
    this->amp16[5] = ampSix;
    this->amp16[6] = ampSeven;
    this->amp16[7] = ampEight;
    this->amp16[8] = ampNine;
    this->amp16[9] = ampTen;
    this->amp16[10] = ampEleven;
    this->amp16[11] = ampTwelve;
    this->amp16[12] = ampThirt;
    this->amp16[13] = ampFourt;
    this->amp16[14] = ampFift;
    this->amp16[15] = ampSixt;
  }

  void Sequencer::package15(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt, String stateFift, int ampFift)
  {
    this->state15[0] = stateOne;
    this->state15[1] = stateTwo;
    this->state15[2] = stateThree;
    this->state15[3] = stateFour;
    this->state15[4] = stateFive;
    this->state15[5] = stateSix;
    this->state15[6] = stateSeven;
    this->state15[7] = stateEight;
    this->state15[8] = stateNine;
    this->state15[9] = stateTen;
    this->state15[10] = stateEleven;
    this->state15[11] = stateTwelve;
    this->state15[12] = stateThirt;
    this->state15[13] = stateFourt;
    this->state15[14] = stateFift;
    
    this->phases = 15;

    this->amp15[0] = ampOne;
    this->amp15[1] = ampTwo;
    this->amp15[2] = ampThree;
    this->amp15[3] = ampFour;
    this->amp15[4] = ampFive;
    this->amp15[5] = ampSix;
    this->amp15[6] = ampSeven;
    this->amp15[7] = ampEight;
    this->amp15[8] = ampNine;
    this->amp15[9] = ampTen;
    this->amp15[10] = ampEleven;
    this->amp15[11] = ampTwelve;
    this->amp15[12] = ampThirt;
    this->amp15[13] = ampFourt;
    this->amp15[14] = ampFift;
  }

  void Sequencer::package14(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt, String stateFourt, int ampFourt)
  {
    this->state14[0] = stateOne;
    this->state14[1] = stateTwo;
    this->state14[2] = stateThree;
    this->state14[3] = stateFour;
    this->state14[4] = stateFive;
    this->state14[5] = stateSix;
    this->state14[6] = stateSeven;
    this->state14[7] = stateEight;
    this->state14[8] = stateNine;
    this->state14[9] = stateTen;
    this->state14[10] = stateEleven;
    this->state14[11] = stateTwelve;
    this->state14[12] = stateThirt;
    this->state14[13] = stateFourt;
    
    this->phases = 14;

    this->amp14[0] = ampOne;
    this->amp14[1] = ampTwo;
    this->amp14[2] = ampThree;
    this->amp14[3] = ampFour;
    this->amp14[4] = ampFive;
    this->amp14[5] = ampSix;
    this->amp14[6] = ampSeven;
    this->amp14[7] = ampEight;
    this->amp14[8] = ampNine;
    this->amp14[9] = ampTen;
    this->amp14[10] = ampEleven;
    this->amp14[11] = ampTwelve;
    this->amp14[12] = ampThirt;
    this->amp15[13] = ampFourt;
  }

  void Sequencer::package13(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve, String stateThirt, int ampThirt)
  {
    this->state13[0] = stateOne;
    this->state13[1] = stateTwo;
    this->state13[2] = stateThree;
    this->state13[3] = stateFour;
    this->state13[4] = stateFive;
    this->state13[5] = stateSix;
    this->state13[6] = stateSeven;
    this->state13[7] = stateEight;
    this->state13[8] = stateNine;
    this->state13[9] = stateTen;
    this->state13[10] = stateEleven;
    this->state13[11] = stateTwelve;
    this->state13[12] = stateThirt;

    this->phases = 13;

    this->amp13[0] = ampOne;
    this->amp13[1] = ampTwo;
    this->amp13[2] = ampThree;
    this->amp13[3] = ampFour;
    this->amp13[4] = ampFive;
    this->amp13[5] = ampSix;
    this->amp13[6] = ampSeven;
    this->amp13[7] = ampEight;
    this->amp13[8] = ampNine;
    this->amp13[9] = ampTen;
    this->amp13[10] = ampEleven;
    this->amp13[11] = ampTwelve;
    this->amp13[12] = ampThirt;
  }

  void Sequencer::package12(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven, String stateTwelve, int ampTwelve)
  {
    this->state12[0] = stateOne;
    this->state12[1] = stateTwo;
    this->state12[2] = stateThree;
    this->state12[3] = stateFour;
    this->state12[4] = stateFive;
    this->state12[5] = stateSix;
    this->state12[6] = stateSeven;
    this->state12[7] = stateEight;
    this->state12[8] = stateNine;
    this->state12[9] = stateTen;
    this->state12[10] = stateEleven;
    this->state12[11] = stateTwelve;

    this->phases = 12;

    this->amp12[0] = ampOne;
    this->amp12[1] = ampTwo;
    this->amp12[2] = ampThree;
    this->amp12[3] = ampFour;
    this->amp12[4] = ampFive;
    this->amp12[5] = ampSix;
    this->amp12[6] = ampSeven;
    this->amp12[7] = ampEight;
    this->amp12[8] = ampNine;
    this->amp12[9] = ampTen;
    this->amp12[10] = ampEleven;
    this->amp12[11] = ampTwelve;
  }

  void Sequencer::package11(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen, String stateEleven, int ampEleven)
  {
    this->state11[0] = stateOne;
    this->state11[1] = stateTwo;
    this->state11[2] = stateThree;
    this->state11[3] = stateFour;
    this->state11[4] = stateFive;
    this->state11[5] = stateSix;
    this->state11[6] = stateSeven;
    this->state11[7] = stateEight;
    this->state11[8] = stateNine;
    this->state11[9] = stateTen;
    this->state11[10] = stateEleven;

    this->phases = 11;

    this->amp11[0] = ampOne;
    this->amp11[1] = ampTwo;
    this->amp11[2] = ampThree;
    this->amp11[3] = ampFour;
    this->amp11[4] = ampFive;
    this->amp11[5] = ampSix;
    this->amp11[6] = ampSeven;
    this->amp11[7] = ampEight;
    this->amp11[8] = ampNine;
    this->amp11[9] = ampTen;
    this->amp11[10] = ampEleven;
  }

  void Sequencer::package10(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine, String stateTen, int ampTen)
  {
    this->state10[0] = stateOne;
    this->state10[1] = stateTwo;
    this->state10[2] = stateThree;
    this->state10[3] = stateFour;
    this->state10[4] = stateFive;
    this->state10[5] = stateSix;
    this->state10[6] = stateSeven;
    this->state10[7] = stateEight;
    this->state10[8] = stateNine;
    this->state10[9] = stateTen;

    this->phases = 10;

    this->amp10[0] = ampOne;
    this->amp10[1] = ampTwo;
    this->amp10[2] = ampThree;
    this->amp10[3] = ampFour;
    this->amp10[4] = ampFive;
    this->amp10[5] = ampSix;
    this->amp10[6] = ampSeven;
    this->amp10[7] = ampEight;
    this->amp10[8] = ampNine;
    this->amp10[9] = ampTen;
  }

  void Sequencer::package9(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight, String stateNine, int ampNine)
  {
    this->state9[0] = stateOne;
    this->state9[1] = stateTwo;
    this->state9[2] = stateThree;
    this->state9[3] = stateFour;
    this->state9[4] = stateFive;
    this->state9[5] = stateSix;
    this->state9[6] = stateSeven;
    this->state9[7] = stateEight;
    this->state9[8] = stateNine;

    this->phases = 9;

    this->amp9[0] = ampOne;
    this->amp9[1] = ampTwo;
    this->amp9[2] = ampThree;
    this->amp9[3] = ampFour;
    this->amp9[4] = ampFive;
    this->amp9[5] = ampSix;
    this->amp9[6] = ampSeven;
    this->amp9[7] = ampEight;
    this->amp9[8] = ampNine;
  }

  void Sequencer::package8(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven, String stateEight, int ampEight)
  {
    this->state8[0] = stateOne;
    this->state8[1] = stateTwo;
    this->state8[2] = stateThree;
    this->state8[3] = stateFour;
    this->state8[4] = stateFive;
    this->state8[5] = stateSix;
    this->state8[6] = stateSeven;
    this->state8[7] = stateEight;

    this->phases = 8;

    this->amp8[0] = ampOne;
    this->amp8[1] = ampTwo;
    this->amp8[2] = ampThree;
    this->amp8[3] = ampFour;
    this->amp8[4] = ampFive;
    this->amp8[5] = ampSix;
    this->amp8[6] = ampSeven;
    this->amp8[7] = ampEight;
  }

  void Sequencer::package7(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix, String stateSeven, int ampSeven)
  {
    this->state7[0] = stateOne;
    this->state7[1] = stateTwo;
    this->state7[2] = stateThree;
    this->state7[3] = stateFour;
    this->state7[4] = stateFive;
    this->state7[5] = stateSix;
    this->state7[6] = stateSeven;

    this->phases = 7;

    this->amp7[0] = ampOne;
    this->amp7[1] = ampTwo;
    this->amp7[2] = ampThree;
    this->amp7[3] = ampFour;
    this->amp7[4] = ampFive;
    this->amp7[5] = ampSix;
    this->amp7[6] = ampSeven;
  }

  void Sequencer::package6(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive, String stateSix, int ampSix)
  {
    this->state6[0] = stateOne;
    this->state6[1] = stateTwo;
    this->state6[2] = stateThree;
    this->state6[3] = stateFour;
    this->state6[4] = stateFive;
    this->state6[5] = stateSix;

    this->phases = 6;

    this->amp6[0] = ampOne;
    this->amp6[1] = ampTwo;
    this->amp6[2] = ampThree;
    this->amp6[3] = ampFour;
    this->amp6[4] = ampFive;
    this->amp6[5] = ampSix;
  }
  
  void Sequencer::package5(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour, String stateFive, int ampFive)
  {
    this->state5[0] = stateOne;
    this->state5[1] = stateTwo;
    this->state5[2] = stateThree;
    this->state5[3] = stateFour;
    this->state5[4] = stateFive;

    this->phases = 5;

    this->amp5[0] = ampOne;
    this->amp5[1] = ampTwo;
    this->amp5[2] = ampThree;
    this->amp5[3] = ampFour;
    this->amp5[4] = ampFive;
  }

  void Sequencer::package4(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree, String stateFour, int ampFour)
  {
    this->state4[0] = stateOne;
    this->state4[1] = stateTwo;
    this->state4[2] = stateThree;
    this->state4[3] = stateFour;

    this->phases = 4;

    this->amp4[0] = ampOne;
    this->amp4[1] = ampTwo;
    this->amp4[2] = ampThree;
    this->amp4[3] = ampFour;
  }

  void Sequencer::package3(String stateOne, int ampOne, String stateTwo, int ampTwo, String stateThree, int ampThree)
  {
    this->state3[0] = stateOne;
    this->state3[1] = stateTwo;
    this->state3[2] = stateThree;
    
    this->phases = 3;

    this->amp3[0] = ampOne;
    this->amp3[1] = ampTwo;
    this->amp3[2] = ampThree;
  }

  void Sequencer::package2(String stateOne, int ampOne, String stateTwo, int ampTwo)
  {
    this->state2[0] = stateOne;
    this->state2[1] = stateTwo;

    this->phases = 2;

    this->amp2[0] = ampOne;
    this->amp2[1] = ampTwo;
  }
  
  String Sequencer::getState(int stateNumber)
  {
    switch(this->phases)
    {
      case 2:
        return this->state2[stateNumber];
        break;
      case 3:
        return this->state3[stateNumber];
        break;
      case 4:
        return this->state4[stateNumber];
        break;
      case 5:
        return this->state5[stateNumber];
        break;
      case 6:
        return this->state6[stateNumber];
        break;
      case 7:
        return this->state7[stateNumber];
        break;
      case 8:
        return this->state8[stateNumber];
        break;
      case 9:
        return this->state9[stateNumber];
        break;
      case 10:
        return this->state10[stateNumber];
        break;
      case 11:
        return this->state11[stateNumber];
        break;
     case 12:
        return this->state12[stateNumber];
        break;
     case 13:
        return this->state13[stateNumber];
        break;
     case 14:
        return this->state14[stateNumber];
        break;
     case 15:
        return this->state15[stateNumber];
        break;       
     case 16:
        return this->state16[stateNumber];
        break; 
     case 17:
        return this->state17[stateNumber];
        break; 
     case 18:
        return this->state18[stateNumber];
        break; 
     default:
        break;  
    }
  }

  int Sequencer::getAmp(int stateNumber)
  {
    switch(this->phases)
    {
      case 2:
        return this->amp2[stateNumber];
        break;
      case 3:
        return this->amp3[stateNumber];
        break;
      case 4:
        return this->amp4[stateNumber];
        break;
      case 5:
        return this->amp5[stateNumber];
        break;
      case 6:
        return this->amp6[stateNumber];
        break;
      case 7:
        return this->amp7[stateNumber];
        break;
      case 8:
        return this->amp8[stateNumber];
        break;
      case 9:
        return this->amp9[stateNumber];
        break;
      case 10:
        return this->amp10[stateNumber];
        break;
      case 11:
        return this->amp11[stateNumber];
        break;
      case 12:
        return this->amp12[stateNumber];
        break;
      case 13:
        return this->amp13[stateNumber];
        break;
      case 14:
        return this->amp14[stateNumber];
        break;
      case 15:
        return this->amp15[stateNumber];
        break;
      case 16:
        return this->amp16[stateNumber];
        break;
      case 17:
        return this->amp17[stateNumber];
        break;
      case 18:
        return this->amp18[stateNumber];
        break;
      default:
        break;  
    }
  }

  int Sequencer::getPhases()
  {
    return this->phases;
  }

//Arms(initial positions :: LeftArm, RightArm, LeftHand, RightHand)
Arms *botArm = new Arms(120,0,70,70);

//Legs(pins :: leftPwm , leftDir, rightPw, rightDir)
Legs *botLeg = new Legs(5,22,4,24);
Sequencer *msequence = new Sequencer();

//pid controller
boolean LconversionResult;
boolean RconversionResult;

double LpidOutput;
double RpidOutput;
double LSetpoint;
double RSetpoint;
double Kp=5, Ki=1, Kd=0;

double Lwspeed, abs_Lwspeed;
double Rwspeed, abs_Rwspeed;

PID myPIDL(&abs_Lwspeed,&LpidOutput,&LSetpoint,Kp,Ki,Kd,DIRECT);
PID myPIDR(&abs_Rwspeed,&RpidOutput,&RSetpoint,Kp,Ki,Kd,DIRECT);

//encoder
//Encoder
const byte encoderLInterrupt = 20;// interrupt 3
const byte encoderLDigital = 8;
const byte encoderRInterrupt = 21;// interrupt 2
const byte encoderRDigital = 9;

byte encoderLInterruptLast;
byte encoderRInterruptLast;

boolean Ldirection;
boolean Rdirection;

void encoderInit()
{
  Ldirection = true;
  Rdirection = true;

  //Left Encoder
  pinMode(encoderLDigital,INPUT);
  attachInterrupt(3,LwheelSpeed,CHANGE);
  //Right Encoder
  pinMode(encoderRDigital,INPUT);
  attachInterrupt(2,RwheelSpeed,CHANGE);  
}

void LwheelSpeed()
{
  int Lstate = digitalRead(encoderLInterrupt);
  if((encoderLInterruptLast == LOW) && Lstate==HIGH)//Lstate will be current state
  {
    int val = digitalRead(encoderLDigital);
    if(val == LOW && Ldirection)
    {
      Serial.println("Lpulse");
      Ldirection = false;
    }
    else if(val == HIGH && !Ldirection)
    {
      Ldirection = true;
    }
  }
  encoderLInterruptLast = Lstate;

  if(!Ldirection)
  {
    Lwspeed++;
  }
  else 
  {
    Lwspeed--;
  }
}

void RwheelSpeed()
{
  int Rstate = digitalRead(encoderRInterrupt);
  if((encoderRInterruptLast == LOW) && Rstate==HIGH)//Rstate will be current state
  {
    int val = digitalRead(encoderRDigital);
    if(val == LOW && Rdirection)
    {
      Serial.println("Rpulse");
      Rdirection = false;
    }
    else if(val == HIGH && !Rdirection)
    {
      Rdirection = true;
    }
  }
  encoderRInterruptLast = Rstate;

  if(Rdirection)
  {
    Rwspeed++;
  }
  else 
  {
    Rwspeed--;  
  }
}

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
  
  //msequence->package8("forward",10000,"left",90,"forward",10000,"left",90,"forward",10000,"left",90,"forward",10000,"left",90);

  msequence->package17("forward",80,"left",90,"forward",230,"left",90,"drop",0,"forward",50,"pickUp",0,"backward",40,"left",90,"forward",230,"right",90,"forward",80,"left",90,"stopMove",10000,"drop",0,"pickUp",0,"left",90);
  
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
