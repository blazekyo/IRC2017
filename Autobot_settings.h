// Code Originator: Delon Tan Wee Keat
// Clone on: 20/06/2017 5.00PM
// Source: https://github.com/blazekyo/IRC2017.git
// Modified: Chan Poh Hong
// Reason: To make the code into classes
// About this code: This is the code for the settings of the autonomous 
//                  robots made for IRC 2017 autonomous bot.
//                  *** Changes made here will affect the sequence ***
//                  *** of movement of the autonomous bot.         ***

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
