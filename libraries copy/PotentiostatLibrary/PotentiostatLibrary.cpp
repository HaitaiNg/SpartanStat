/**
 * \file PotentiostatLibrary.cpp
 *
 * \author Haitai Ng & Justin Opperman
 *
 * This file contains all the function definitions and source code
 * required to execute the portable potentiostat created by MSU
 * ECE 480 Team 11 Fall 2017.
 *
 * This file is to be used with PotentiostatLibrary.h
 */


#include "PotentiostatLibrary.h"
#include "Arduino.h"

Adafruit_ADS1115 ads;

/*
 * Name: PotententioStatLibrary
 * Description: Constructor. Initiate and create a Potentiostat object
 */
PotentiostatLibrary::PotentiostatLibrary()
{
  /// Set the gain and configuration for the external ADC
  ads.setGain(GAIN_TWOTHIRDS);
  ads.begin();
}

/*
 * Name: convertVoltageForDAC
 * Description: Calbirate the desired voltage so it can be received by DAC
 */
int PotentiostatLibrary::convertVoltageForDAC(int desiredVoltage)
{
  return (desiredVoltage + 2346) * 0.681;
}

/*
*  Name: init
*  Description: initialize and set private member variables associated with
*   the square wave
*
* \param Square Wave input parameters
* \return None
*/

void PotentiostatLibrary::init( double delayTimeHigh,
  double delayTimeLow, double quietTime, double squareWaveAmpltudeVoltage,
  double squareWaveStepVoltage, double startVoltage, double stopVoltage)
{
  mDelayTimeHigh = delayTimeHigh;
  mDelayTimeLow = delayTimeLow;
  mQuietTime = quietTime;
  mSquareWaveAmplitudeVoltage = 0.681 * squareWaveAmpltudeVoltage;
  mSquareWaveStepVoltage = 0.681 * squareWaveStepVoltage;
  mStartVoltage = convertVoltageForDAC(startVoltage);
  mStopVoltage = convertVoltageForDAC(stopVoltage);
  executePulse();
}

/*
*  Name: setPinToOutput
*  Descripton: set the GPIO pins to output mode
* \param None
* \return None
*/

void PotentiostatLibrary::setPinsToOutput()
{
  pinMode(mDigitalPinEleven, OUTPUT);
  pinMode(mDigitalPinTen, OUTPUT);
  pinMode(mDigitalPinNine, OUTPUT);
  pinMode(mDigitalPinEight, OUTPUT);
  pinMode(mDigitalPinSeven, OUTPUT);
  pinMode(mDigitalPinSix, OUTPUT);
  pinMode(mDigitalPinFive, OUTPUT);
  pinMode(mDigitalPinFour, OUTPUT);
  pinMode(mDigitalPinThree, OUTPUT);
  pinMode(mDigitalPinTwo, OUTPUT);
  pinMode(mDigitalPinOne, OUTPUT);
  pinMode(mDigitalPinZero, OUTPUT);
}

/*
*  Name: setWritePinToOutput
*  Description: set the write pin to output mode
* \param None
* \return None
*/
void PotentiostatLibrary::setWritePinToOutput()
{
  pinMode(mDigitalPinWR, OUTPUT);
}

/*
*  Name: WriteToDigitalPins
*  Description: Set the value to the GPIO pin.
*   Logic high = a voltage > 0
    Logic low = a voltage < 0
* \param integer GPIO pin , integer Logic high (1) or Logic low (0)
* \return None
*/
void PotentiostatLibrary::writeToDigitalPin( int digitalPin, int value)
{
  if(value == 1)
  {
    digitalWrite(digitalPin, HIGH);
  //  Serial.print("1");

  }
  else
  {
    digitalWrite(digitalPin, LOW);
  //  Serial.print("0");
  }
}

/*
*  Name: OutputDigitalPins
*  Description: Convert an integer value into a 12 bit binary equivalent
*  Creates an array data structure (size 12) and converts a 32 bit integer value
*  into a (12 bit) binary equivalent
*  This is completed by extensive bit shifting.
*  Each bit of the 12 bit array will then be outputted to the DAC by writing
*  Initialize and write once the digital write pin has been set
*
*
* \param integer DigitalEquivalentValue
* \return None
*/

void PotentiostatLibrary::outputDigitalValues( int digitalEquivalentValue)
{
  int a = 31; // 32 bit number
  int referenceIndex = 0;
  int splitArray[12]; // integers are 32 bit, create a 12 bit array
  while( a > 0)
  {
    int digitalOutputPinArray[32]; // initalize the array
    int n = digitalEquivalentValue >> a; // bit shifting to signed
    if (!(n % 2)) digitalOutputPinArray[referenceIndex] = 0 ;
    else digitalOutputPinArray[referenceIndex] = 1;
    a--;
    referenceIndex++;
    //copy over the 12 bits of interest into the smaller array
    memcpy(splitArray, digitalOutputPinArray + 20, sizeof(splitArray));
  }

  /// set the GPIO pins to output its associated value
  writeToDigitalPin(mDigitalPinEleven, splitArray[0]); // 2^11
  writeToDigitalPin(mDigitalPinTen, splitArray[1]); // 2^10
  writeToDigitalPin(mDigitalPinNine, splitArray[2]); // 2^9
  writeToDigitalPin(mDigitalPinEight, splitArray[3]); // 2^8
  writeToDigitalPin(mDigitalPinSeven, splitArray[4]); //2^7
  writeToDigitalPin(mDigitalPinSix, splitArray[5]); // 2^6
  writeToDigitalPin(mDigitalPinFive, splitArray[6]); // 2^5
  writeToDigitalPin(mDigitalPinFour, splitArray[7]); // 2^4
  writeToDigitalPin(mDigitalPinThree, splitArray[8]); // 2^3
  writeToDigitalPin(mDigitalPinTwo, splitArray[9]); // 2^2
  writeToDigitalPin(mDigitalPinOne, splitArray[10]); // 2^1
  writeToDigitalPin(mDigitalPinZero, splitArray[11]); // 2^0

  delay(1);
  digitalWrite(mDigitalPinWR, LOW); //< Disable all writing to DAC
  delay(1); //< wait until all GPIO pins have all their values set
  digitalWrite(mDigitalPinWR, HIGH); //< write to DAC
  delay(1);

}

/*
*  Name: linearSweepAlgorithm
*  Description: This program executes the linear sweep 
* \param integer DigitalEquivalentValue
* \return None
*/
void PotentiostatLibrary::linearSweepAlgorithm()
{
  double delayTimeHigh = 0; double delayTimeLow = 0;
  double startValue = 0; double stopValue = 0;
  double squareWaveAmplitude = 1;
  double squareWaveStep = 1;


  if(mLinearSweepType == 2)
  {
    startValue = convertVoltageForDAC(mStartVoltage);
    stopValue = convertVoltageForDAC(mStopVoltage);
    // Increasing slope

    for(int val = startValue ; val >= stopValue; val -= squareWaveStep)
     {
      //Serial.print(determineDigitalPinToSet(val), 4);
      Serial.print(DACvalToVoltage(val), 4);
        Serial.print("     "); // this has to be 5 spaces
      //Serial.print(val);
        Serial.print("     ");
      outputDigitalValues((int) val);
      readCurrent();
    //  delay(100);
    }

    // Decreasing slope
    for(int val = stopValue; val <= startValue; val += squareWaveStep)
    {
      //Serial.print(determineDigitalPinToSet(val), 4);
      Serial.print(DACvalToVoltage(val), 4);
         Serial.print("     ");
      //Serial.print(val);
        Serial.print("     ");
      outputDigitalValues((int) val);
      readCurrent();
    //  delay(100);
    }
  }
  else if (mLinearSweepType == 1)
  {
    startValue = convertVoltageForDAC(mStopVoltage);
    stopValue = convertVoltageForDAC(mStartVoltage);

    // Increasing slope
    for(int val = stopValue; val <= startValue; val += squareWaveStep)
    {
      Serial.print(DACvalToVoltage(val), 4);
         Serial.print("     ");
    //  Serial.print(val);
        Serial.print("     ");
      outputDigitalValues((int) val);
      readCurrent();
      delay(100);
    }

    for(int val = startValue ; val >= stopValue; val -= squareWaveStep)
     {
      //Serial.print(determineDigitalPinToSet(val), 4);
      Serial.print(DACvalToVoltage(val), 4);
        Serial.print("     "); // this has to be 5 spaces
    //  Serial.print(val);
        Serial.print("     ");
      outputDigitalValues((int) val);
      readCurrent();
      delay(100);
    }
   }
   else
   {
     Serial.println("Error :: Invalid input. Please try again \n");
   }
}


int PotentiostatLibrary::executeLinearSweep()
{
  while(mLinearSweepRepititions > 0)
  {
    linearSweepAlgorithm();
    mLinearSweepRepititions--;
  }
  return 1;
}


//// ALL code relative to the pulse waveform

void PotentiostatLibrary::executePulse()
{
  delay(mQuietTime);
  while(true)
    {
    squareWaveAlgorithm( mDelayTimeHigh, mDelayTimeLow, mStartVoltage,
    mStopVoltage, mSquareWaveAmplitudeVoltage, mSquareWaveStepVoltage);
    delay(5000);
    break;
    }

}

void PotentiostatLibrary::squareWaveAlgorithm(double delayTimeHigh, double delayTimeLow,
  double startValue, double stopValue, double squareWaveAmplitude, double squareWaveStep)
{
   /*
   * This function creates the increasing part of the square wave. Each iteration of the for loop generates a pulse with an amplitude specified by the user.
   * After each iteration, the step is incremented, and the next pulse will start at that step.
   */

  for(int val = startValue ; val <= (stopValue - squareWaveAmplitude); val += squareWaveStep)
   {

    /*
    Serial.print(DACvalToVoltage(val));
    Serial.print("     ");
    Serial.print("     ");
    */

    // give low part of square wave
    outputDigitalValues((int) val);
    delay(delayTimeLow);

    // ** Here we collect sample current
    double current_high = readCurrentForSquareWave();

    // give high part of square wave
    val += squareWaveAmplitude;

    /*
    Serial.print(DACvalToVoltage(val));
    Serial.print("     ");
    Serial.print("     ");
    */

    outputDigitalValues((int) val);
    delay(delayTimeHigh);

    //  ** Here we collect sample current
    double current_low = readCurrentForSquareWave();

    // Reset squarewave back to low, let the loop increment
    val -= squareWaveAmplitude;

    // Print difference of two currents and the starting Voltage
    Serial.print(DACvalToVoltage(val));
    Serial.print("          ");
    double differenceCurrent = current_high - current_low;
    printCurrentForSquareWave(differenceCurrent);
   }

   // Next section commented out as sweep only needs to go in one direction
   /*
   for(int val = stopValue ; val >= (startValue + squareWaveAmplitude); val -= squareWaveStep) {

     // give high part of square wave
     Serial.print(val);
     Serial.print("     ");
     outputDigitalValues(val);
     delay(delayTimeHigh);
     // read current Here
     readCurrent();
     // give low part of square wave
     val -= squareWaveAmplitude;

     Serial.print(val);
     Serial.print("     ");
     outputDigitalValues(val);
     delay(delayTimeLow);
     readCurrent();
     // Reset back to high, let the loop decrement
     val += squareWaveAmplitude;
   }
   */
}

int PotentiostatLibrary::readCurrent()
{
  //mADCValue = analogRead(mAnalogPinOne) * mADCValueToVoltageRatio;
  //mCurrent = 0.0372 * mADCValue - 86.031;
  //Serial.print( mCurrent);
  //Serial.println("  ");
  //return 1;

  int16_t adc1;
  adc1 = ads.readADC_SingleEnded(1); //* 0.1875;
  Serial.print(adc1);
  Serial.println("  ");
  return adc1;

}

/*
This function reads the raw ADC value from the Arduino pin, and converts
it into a current using a calibration equation.
*/
double PotentiostatLibrary::readCurrentForSquareWave()
{
  //int16_t adc1;
  //adc1 = ads.readADC_SingleEnded(1);
  //return adc1;

  double ADCValue = analogRead(mAnalogPinOne) * mADCValueToVoltageRatio;
  double current = .0372 * ADCValue - 86.031;
  return current;
}

/*
This function prints the current in the correct format
*/
void PotentiostatLibrary::printCurrentForSquareWave(double current)
{
    Serial.print(current);
    Serial.println("  ");
}

/*
Takes a DAC value as input, and returns the corresponding voltage that should
appear at the output of the circuit
*/
double PotentiostatLibrary::DACvalToVoltage(int DACval)
{
  double convertedVoltage = (DACval * 1.468) - 2346;
  return -convertedVoltage;
}
