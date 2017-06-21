// Code Originator: Delon Tan Wee Keat
// Clone on: 20/06/2017 5.00PM
// Source: https://github.com/blazekyo/IRC2017.git
// Modified: Chan Poh Hong
// Reason: To make the code into classes
// About this code: This is the code for the wheel part of the autonomous 
//					        robots made for IRC 2017 autonomous bot.

#include<PID_v1.h>

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
