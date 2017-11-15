
#include "PotentiostatLibrary.h"
#include "Arduino.h"


PotentiostatLibrary::PotentiostatLibrary()
{
}


void PotentiostatLibrary::init( double delayTimeHigh,
  double delayTimeLow, double quietTime, double squareWaveAmpltudeVoltage,
  double squareWaveStepVoltage, double startVoltage, double stopVoltage)
{
  mDelayTimeHigh = delayTimeHigh;
  mDelayTimeLow = delayTimeLow;
  mQuietTime = quietTime;
  mSquareWaveAmplitudeVoltage = squareWaveAmpltudeVoltage;
  mSquareWaveStepVoltage = squareWaveStepVoltage;
  mStartVoltage = startVoltage;
  mStopVoltage = stopVoltage;

  mStartValue = (int)((mStartVoltage + 2500) * .819);
  mStopValue = (int)((mStopVoltage + 2500) * .819);
  mSquareWaveAmplitude = (int)(mSquareWaveAmplitudeVoltage * .819);  // Val between 0 and 4095
  mSquareWaveStep = (int)(mSquareWaveStepVoltage * .819);

  executePulse();
}


// Configure pins to output mode
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

// Set write pin to output
void PotentiostatLibrary::setWritePinToOutput()
{
  pinMode(mDigitalPinWR, OUTPUT);
}


void PotentiostatLibrary::writeToDigitalPin( int digitalPin, int value)
{
  if(value == 1)
  {
    digitalWrite(digitalPin, HIGH);
    Serial.print("1");

  }
  else
  {
    digitalWrite(digitalPin, LOW);
    Serial.print("0");
  }
}


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
    memcpy(splitArray, digitalOutputPinArray + 19, sizeof(splitArray));
  }

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

  Serial.println();
  delay(1);
  //digitalWrite(mDigitalPinWR, LOW); //activating on active low
  delay(1);
  //digitalWrite(mDigitalPinWR, HIGH);
  delay(1);

}

double PotentiostatLibrary::determineDigitalPinToSet(double currentStep, double stepSize)
{
  return currentStep / stepSize;
}

void PotentiostatLibrary::linearSweepAlgorithm(double userInputStepSize)
{
  double startVoltage = 5.00;
  double sampleSize = 4096.00;
  double stepSize = startVoltage / sampleSize;

  //Increasing (positive slope)
  for( double currentStep = 0; currentStep <= 5; currentStep += stepSize) // i == currentStep
  {
    int digitalEquivalentValue = determineDigitalPinToSet( currentStep, stepSize);
    outputDigitalValues( digitalEquivalentValue);
    Serial.println(digitalEquivalentValue);
  }
  //Decreasing (negative slope)
  for( double currentStepReverse = 5; currentStepReverse >= 0; currentStepReverse -= stepSize) // i == currentStep
  {
    int digitalEquivalentValue = determineDigitalPinToSet( currentStepReverse, stepSize);
    outputDigitalValues( digitalEquivalentValue);
    Serial.println(digitalEquivalentValue);
  }
}


int PotentiostatLibrary::executeLinearSweep()
{
  while(mLinearSweepRepititions > 0)
  {
    linearSweepAlgorithm(100);
    mLinearSweepRepititions--;
  }//stepAlgorithmReverse(100);
  return 1; 
}


//// ALL code relative to the pulse waveform

void PotentiostatLibrary::executePulse()
{
  delay(mQuietTime);
  squareWaveIncreasing( mDelayTimeHigh, mDelayTimeLow, mStartValue,
    mStopValue, mSquareWaveAmplitude, mSquareWaveStep);
  squareWaveDecreasing( mDelayTimeHigh, mDelayTimeLow, mStartValue,
    mStopValue, mSquareWaveAmplitude, mSquareWaveStep);
}


void PotentiostatLibrary::squareWaveIncreasing(int delayTimeHigh, int delayTimeLow,
  int startValue, int stopValue, int squareWaveAmplitude, int squareWaveStep)
{
  /*
   * This function creates the increasing part of the square wave. Each iteration of the for loop generates a pulse with an amplitude specified by the user.
   * After each iteration, the step is incremented, and the next pulse will start at that step.
   */
  for(int val = startValue ; val <= (stopValue - squareWaveAmplitude); val += squareWaveStep) {

    // give low part of square wave
    outputDigitalValues(val);
    delay(delayTimeLow);

    // give high part of square wave
    val += squareWaveAmplitude;
    outputDigitalValues(val);
    delay(delayTimeHigh);

    // Reset squarewave back to low, let the loop increment
    val -= squareWaveAmplitude;
   }
}

void PotentiostatLibrary::squareWaveDecreasing(int delayTimeHigh, int delayTimeLow,
   int startValue, int stopValue, int squareWaveAmplitude, int squareWaveStep)
{
 /*
 * This function creates the decreasing part of the square wave. Each iteration of the for loop generates a pulse with an amplitude specified by the user.
   * After each iteration, the step is decremented, and the next pulse will start at that step.
 */
  for(int val = stopValue ; val >= (startValue + squareWaveAmplitude); val -= squareWaveStep) {

    // give high part of square wave
    outputDigitalValues(val);
    delay(delayTimeHigh);

    // give low part of square wave
    val -= squareWaveAmplitude;
    outputDigitalValues(val);
    delay(delayTimeLow);

    // Reset back to high, let the loop decrement
    val += squareWaveAmplitude;
  }

}
