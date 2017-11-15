

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

    void init( double delayTimeHigh, double delayTimeLow,
      double quietTime, double squareWaveAmpltudeVoltage,
      double squareWaveStepVoltage, double startVoltage, double stopVoltage);

    void setPinsToOutput();
    void setPinsToInput();
    void setWritePinToOutput();

    double determineDigitalPinToSet(double currentStep, double stepSize);
    void linearSweepAlgorithm(double userInputStepSize);
    void setLinearSweepRepititions(double repititions) {mLinearSweepRepititions = repititions;}
    //void stepAlgorithmReverse(double userInputStepSize);
    void writeToDigitalPin( int digitalPin, int value);
    void outputDigitalValues( int digitalEquivalentValue);
    int executeLinearSweep();

    // square wave functions
    void squareWaveAlgorithm( double delayTimeHigh, double delayTimeLow,
      double startValue, double stopValue, double squareWaveAmplitude,
      double squareWaveStep);
    void executePulse();

    int convertVoltageForDAC(int desiredVoltage);
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

    double mLinearSweepRepititions = 0;

};

#endif
