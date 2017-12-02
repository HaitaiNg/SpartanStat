/**
 * \file PotentiostatLibrary.h
 *
 * \author Haitai Ng & Justin Opperman
 *
 * This file contains all the function declarations and member variables
 * required to execute the portable potentiostat created by MSU
 * ECE 480 Team 11 Fall 2017.
 *
 * This file is to be used with PotentiostatLibrary.cpp
 */


#ifndef PotentiostatLibrary_h // TL
#define PotentiostatLibrary_h // TL

#if (ARDUINO >= 100)
  #include "Arduino.h"
  #include "Adafruit_ADS1015.h"
  #include "Wire.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
  #include "WConstants.h"
#endif


/**
 * Potentiostat Library is an object that is to be used with the developed
 * PCB / Circuit made by the ECE 480 Team 11.
 */
class PotentiostatLibrary
{
  public:

    /// Constructor
    PotentiostatLibrary();
    /// Initialize and pass square wave parameters to private member variables
    void init( double delayTimeHigh, double delayTimeLow,
      double quietTime, double squareWaveAmpltudeVoltage,
      double squareWaveStepVoltage, double startVoltage, double stopVoltage);

    /// Set GPIO pins to output
    void setPinsToOutput();
    /// Set GPIO pins to input
    void setPinsToInput();
    /// Set 'Write Pin' to output a digital high signal
    void setWritePinToOutput();
    /// Write value to digital GPIO pins (logic low or logic high)
    void writeToDigitalPin( int digitalPin, int value);
    /// Output voltage from GPIO pin
    void outputDigitalValues( int digitalEquivalentValue);
    /// Calibrate a desired voltage to be compatible with the external DAC
    int convertVoltageForDAC(int desiredVoltage);
    /// Read current
    int readCurrent();
    /// Convert a digital signal from the DAC to a voltage
    double DACvalToVoltage(int DACval);


    /// Linear sweep functions
    void linearSweepAlgorithm();
    /// Set member variables relatives to linear sweep
    void setLinearParameters(double startVoltage, double stopVoltage) {mStartVoltage = startVoltage; mStopVoltage = stopVoltage;}
    /// Set the slope of the initial linear sweep
    void setLinearSlope(int slope) {mLinearSweepType = slope;}
    /// Set the number of repitions for linear sweep
    void setLinearSweepRepititions(double repititions) {mLinearSweepRepititions = repititions;}
    /// Execute the linear sweep
    int executeLinearSweep();

    /// Square wave functions
    void squareWaveAlgorithm( double delayTimeHigh, double delayTimeLow,
      double startValue, double stopValue, double squareWaveAmplitude,
      double squareWaveStep);
    /// Execute the square wave
    void executePulse();
    /// Read the current from the square wave
    double readCurrentForSquareWave();
    /// Print the current onto the console
    void printCurrentForSquareWave(double current);

    /// Pin declarations. These pins output a voltage to the external DAC
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

    /// Linear Sweep
    double mLinearSweepRepititions = 0;
    double mLinearSweepMinVoltage = -2.0;
    double mLinearSweepMaxVoltage = 2.0;
    double mLinearSweepStep = 0.001;
    int mLinearSweepType = 0;

    /// Arduino Mega Pins
    int mAnalogPinOne = A15;
    int mAnalogPinTwo = A14;
    double mADCValueToVoltageRatio = 4878.0 / 1024.0;
    double mADCValue = 0;
    double mCurrent = 0;
};
#endif
