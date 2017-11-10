

#ifndef PotentiostatLibrary_h // TL
#define PotentiostatLibrary_h // TL

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif




class PotentiostatLibrary
{
  public:

    PotentiostatLibrary(); // Constructor

    void init(double userMode, double delayTimeHigh, double delayTimeLow,
      double quietTime, double squareWaveAmpltudeVoltage,
      double squareWaveStepVoltage, double startVoltage, double stopVoltage);

    void setPinsToOutput();
    void setPinsToInput();
    void setWritePinToOutput();

    double determineDigitalPinToSet(double currentStep, double stepSize);
    void stepAlgorithm(double userInputStepSize);
    void writeToDigitalPin( int digitalPin, int value);
    void outputDigitalValues( int digitalEquivalentValue);
    void executeLinearSweep();

    // square wave functions
    void squareWaveIncreasing( int delayTimeHigh, int delayTimeLow,
      int startValue, int stopValue, int squareWaveAmplitude,
      int squareWaveStep);
    void squareWaveDecreasing( int delayTimeHigh, int delayTimeLow,
      int startValue, int stopValue, int squareWaveAmplitude,
      int squareWaveStep);
    void executePulse(); 


    // Pin declarations
    int mDigitalPinZero = 24;
    int mDigitalPinOne = 25;
    int mDigitalPinTwo = 26;
    int mDigitalPinThree = 27;
    int mDigitalPinFour = 28;
    int mDigitalPinFive = 29;
    int mDigitalPinSix = 30;
    int mDigitalPinSeven = 31;
    int mDigitalPinEight = 32;
    int mDigitalPinNine = 33;
    int mDigitalPinTen = 34;
    int mDigitalPinEleven = 35;
    // Output pin to control writing to the DAC
    int mDigitalPinWR = 23;

    // Square wave parameters
    int mQuietTime = 0;
    int mDelayTimeHigh = 0;
    int mDelayTimeLow = 0;
    double mSquareWaveAmplitudeVoltage = 0;
    double mSquareWaveStepVoltage = 0;
    double mStartVoltage = 0;
    double mStopVoltage = 0;

    int mStartValue = (int)((mStartVoltage + 2500) * .819);
    int mStopValue = (int)((mStopVoltage + 2500) * .819);

    int mSquareWaveAmplitude = (int)(mSquareWaveAmplitudeVoltage * .819);  // Val between 0 and 4095
    int mSquareWaveStep = (int)(mSquareWaveStepVoltage * .819);

    int mSquareWaveFlag = 0 ;
    int mNumberOfFields = 8;
    int mFieldIndex = 0;
    double mValues[8];

};

















#endif
