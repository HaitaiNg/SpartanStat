
#include "PotentiostatLibrary.h"
#include "Arduino.h"


PotentiostatLibrary::PotentiostatLibrary()
{
}

int PotentiostatLibrary::convertVoltageForDAC(int desiredVoltage)
{
  return (desiredVoltage + 2346) * 0.681;
}

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

/*
  mStartValue = (int)((mStartVoltage + 2500) * .819);
  mStopValue = (int)((mStopVoltage + 2500) * .819);
  mSquareWaveAmplitude = (int)(mSquareWaveAmplitudeVoltage * .819);  // Val between 0 and 4095
  mSquareWaveStep = (int)(mSquareWaveStepVoltage * .819);
*/

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
    memcpy(splitArray, digitalOutputPinArray + 20, sizeof(splitArray));
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
  digitalWrite(mDigitalPinWR, LOW); //activating on active low
  delay(1);
  digitalWrite(mDigitalPinWR, HIGH);
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
    Serial.print(digitalEquivalentValue);
    readCurrent();
  }
  //Decreasing (negative slope)

  for( double currentStepReverse = 5; currentStepReverse >= 0; currentStepReverse -= stepSize) // i == currentStep
  {
    int digitalEquivalentValue = determineDigitalPinToSet( currentStepReverse, stepSize);
    outputDigitalValues( digitalEquivalentValue);
    Serial.print(digitalEquivalentValue);
    readCurrent();
  }

  delay(1000);
  //1598 sets DAC to zer0
  outputDigitalValues(0);
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
  int x = 3;
  while(x > 0 )
    {
    squareWaveAlgorithm( mDelayTimeHigh, mDelayTimeLow, mStartVoltage,
    mStopVoltage, mSquareWaveAmplitudeVoltage, mSquareWaveStepVoltage);
    delay(5000);
    x--;
    }

}


void PotentiostatLibrary::squareWaveAlgorithm(double delayTimeHigh, double delayTimeLow,
  double startValue, double stopValue, double squareWaveAmplitude, double squareWaveStep)
{
   /*
   * This function creates the increasing part of the square wave. Each iteration of the for loop generates a pulse with an amplitude specified by the user.
   * After each iteration, the step is incremented, and the next pulse will start at that step.
   */
   Serial.println();
   Serial.print("Delay time HIGH : ");
   Serial.println(delayTimeHigh);
   Serial.print("Delay time LOW : ");
   Serial.println(delayTimeLow);
   Serial.print("Start valaue : ");
   Serial.println(startValue);
   Serial.print("Stop value : ");
   Serial.println(stopValue);
   Serial.print("Square Wave Amplitude :");
   Serial.println(squareWaveAmplitude);
   Serial.print("Square wave step :");
   Serial.println(squareWaveStep);



  for(int val = startValue ; val <= (stopValue - squareWaveAmplitude); val += squareWaveStep)
   {
    // give low part of square wave
    //outputDigitalValues(val);
    Serial.print(val);
    Serial.print("     ");
    outputDigitalValues((int) val);
    delay(delayTimeLow);
    // ** Here we collect sample current
    // give high part of square wave
    val += squareWaveAmplitude;
    Serial.print(val);
    Serial.print("     ");
    outputDigitalValues((int) val);
    delay(delayTimeHigh);
    //  ** Here we collect sample current
    // Reset squarewave back to low, let the loop increment
    val -= squareWaveAmplitude;
   }

   Serial.println();
   Serial.print("Delay time HIGH : ");
   Serial.println(delayTimeHigh);
   Serial.print("Delay time LOW : ");
   Serial.println(delayTimeLow);
   Serial.print("Start valaue : ");
   Serial.println(startValue);
   Serial.print("Stop value : ");
   Serial.println(stopValue);
   Serial.print("Square Wave Amplitude :");
   Serial.println(squareWaveAmplitude);
   Serial.print("Square wave step :");
   Serial.println(squareWaveStep);
   Serial.println();


   for(int val = stopValue ; val >= (startValue + squareWaveAmplitude); val -= squareWaveStep) {

     // give high part of square wave
     Serial.print(val);
     Serial.print("     ");
     outputDigitalValues(val);
     delay(delayTimeHigh);
     // read current Here

     // give low part of square wave
     val -= squareWaveAmplitude;
     Serial.print(val);
     Serial.print("     ");
     outputDigitalValues(val);
     delay(delayTimeLow);

     // Reset back to high, let the loop decrement
     val += squareWaveAmplitude;
   }
   delay(1000);
   //1598 sets DAC to zer0
   outputDigitalValues(1598);
   delay(3000);
}

int PotentiostatLibrary::readCurrent()
{

  double ADCValue = analogRead(mAnalogPinOne);
  double ADCValueConverted = ADCValue * mADCValueToVoltageRatio;
  double current = 0.0372 * ADCValueConverted - 86.031;
//  Serial.print("ADC Converted: ");
  //Serial.print(ADCValueConverted);
  //Serial.print("   analogReadValue: ");
  //Serial.println(ADCValue);
  Serial.print("  Current: ");
  Serial.println(current);
  //delay(1000);
  return 1;

}
