/* **************************
 *  This software will generate a signal that will be read by the 12-bit DAC and turned into an analog signal
 *  with the desired characteristics. 
 *  The parameters of the square wave will be changed using information sent over USB, and a sweep will be run.
 *  The sweep will output the correct voltage waveform, and be capable of reading back a response at a different pin.
 *  This information will then be packaged together and transmitted back over USB to the host device.
 *  *************************
 *  It is worth noting that the external circuitry as of 11/1/17 inverts the output from the MCU.
 *  (e.g. Square wave that is high will appear low when measured with an oscilloscope)
 *  When measuring the output in lab, an increasing waveform will appear on the oscilloscopes as decreasing. 
 *  *************************
 *  To properly set this up using the circuit in lab, +-5V sources are needed. 
 *  
 *  Note: I apologize in advance for overdoing it with the comments. Hopefully I don't comment anything obvious.
 */

// Pin Declarations
int digitalPinOne = 24;
int digitalPinTwo = 25;
int digitalPinThree = 26;
int digitalPinFour = 27;
int digitalPinFive = 28;
int digitalPinSix = 29;
int digitalPinSeven = 30;
int digitalPinEight = 31;
int digitalPinNine = 32;
int digitalPinTen = 33;
int digitalPinEleven = 34;
int digitalPinTwelve = 35;

// Output pin to control writing to the DAC
int digitalPinWR = 23;  

//-----------------------------------------------------------------------------------------------------------
// Information sent over USB will be stored in the following variables:

// IMPORTANT NOTE ABOUT DELAYS: There is an additional 2ms delay added to each delay time due to the time it takes to write to the digitalPinWR.
int delayTimeHigh = 5;                    // Time that square wave is logic 'High' (ms)
int delayTimeLow = 5;                     // Time that square wave is logic 'Low' (ms)
int quietTime = 0;                        // 'Resting Time' in between sweeps (ms)
double squareWaveAmplitudeVoltage = .5;   // Gives 'Height' of each square wave instance in Volts (V)
double squareWaveStepVoltage = .1;        // Gives increment that each square wave will start at in Volts (V)
double startVoltage = -2.5;               // Starting voltage (V)
double stopVoltage = 2.5;                 // Ending Voltage (V)
//-----------------------------------------------------------------------------------------------------------

// These values need to be converted into positive integers so that the Arduino can use them. 
// These values are first shifted up by 2.5V in order to create a positive value. 
// As a 12-bit DAC will be used, the total range will have to be filled out using 4096 discrete points (2^12 points)
// This is done by multiplying by 4096 to get 4096 points, and then dividing by 5 to fit those 4096 points between 0 and 5
// Values are rounded down due to int typecasting
// Note: 819 == 4096/5

int startValue = (int)((startVoltage + 2.5) * 819);                 // Val between 0 and 4095
int stopValue = (int)((stopVoltage + 2.5) * 819);                   // Val between 0 and 4095

// The same process is applied to the square wave amplitude and step size parameters to fit them between 4096 values
// Values are rounded down due to int typecasting
// Note: 819 == 4096/5

int squareWaveAmplitude = (int)(squareWaveAmplitudeVoltage * 819);  // Val between 0 and 4095
int squareWaveStep = (int)(squareWaveStepVoltage * 819);            // Val between 0 and 4095

//-----------------------------------------------------------------------------------------------------------

void setup()  
{ 
  Serial.begin(9600);                     // initialize serial communication at 9600 bits per second
  Serial.println("Executing Program");    // print to serial to indicate program has started

  // Setting up output pins to be used as inputs to the DAC
  pinMode(digitalPinTwelve, OUTPUT);
  pinMode(digitalPinEleven, OUTPUT);
  pinMode(digitalPinTen, OUTPUT);
  pinMode(digitalPinNine, OUTPUT);
  pinMode(digitalPinEight, OUTPUT);
  pinMode(digitalPinSeven, OUTPUT);
  pinMode(digitalPinSix, OUTPUT);
  pinMode(digitalPinFive, OUTPUT);
  pinMode(digitalPinFour, OUTPUT);
  pinMode(digitalPinThree, OUTPUT);
  pinMode(digitalPinTwo, OUTPUT);
  pinMode(digitalPinOne, OUTPUT); 
  
  // Setting up output pin to control writing to the DAC
  pinMode(digitalPinWR, OUTPUT);

} 

// Looping through the square waveform
void loop() 
{
  // Quiet Time to let analyte rest in between sweeps
  delay(quietTime);
  
  // Sweep Up 
  squareWaveIncreasing(delayTimeHigh, delayTimeLow, startValue, stopValue, squareWaveAmplitude, squareWaveStep);
  
  // Sweep Down
  squareWaveDecreasing(delayTimeHigh, delayTimeLow, startValue, stopValue, squareWaveAmplitude, squareWaveStep);
}

//---------------------------------------------------------------------------------------------------------------------------------------
// End of main function. Functions used are below.
//---------------------------------------------------------------------------------------------------------------------------------------

void squareWaveIncreasing(int delayTimeHigh, int delayTimeLow, int startValue, int stopValue, int squareWaveAmplitude, int squareWaveStep)
{
  /*
   * This function creates the increasing part of the square wave. Each iteration of the for loop generates a pulse with an amplitude specified by the user.
   * After each iteration, the step is incremented, and the next pulse will start at that step. 
   */
  for(int val = startValue ; val <= (stopValue - squareWaveAmplitude); val += squareWaveStep) {
  
    // give low part of square wave
    setDigitalOutputPins(val);
    delay(delayTimeLow);

    // give high part of square wave
    val += squareWaveAmplitude;
    setDigitalOutputPins(val);
    delay(delayTimeHigh);

    // Reset squarewave back to low, let the loop increment
    val -= squareWaveAmplitude;
   }
}

void squareWaveDecreasing(int delayTimeHigh, int delayTimeLow, int startValue, int stopValue, int squareWaveAmplitude, int squareWaveStep)
{
 /*
 * This function creates the decreasing part of the square wave. Each iteration of the for loop generates a pulse with an amplitude specified by the user.
   * After each iteration, the step is decremented, and the next pulse will start at that step. 
 */
  for(int val = stopValue ; val >= (startValue + squareWaveAmplitude); val -= squareWaveStep) {
  
    // give high part of square wave
    setDigitalOutputPins(val);
    delay(delayTimeHigh);

    // give low part of square wave
    val -= squareWaveAmplitude;
    setDigitalOutputPins(val);
    delay(delayTimeLow);

    // Reset back to high, let the loop decrement
    val += squareWaveAmplitude;
  }
  
}

void setDigitalOutputPins( int val)
{
  /*
   * This function takes a value between 0 and 4095, and sets 12 pins to be output to the DAC. The 12 pins that are output
   * contain the binary encoded representation of the input value. 
   */
  
  int a = 31; // 32 bit number int
  int referenceIndex = 0; 
  while ( a >= 0)
  {
    int digitalOutputPinArray[32]; // initalize the array 
    int n = val >> a; // bit shifting to signed
    if (!(n % 2)) digitalOutputPinArray[referenceIndex] = 0 ;
    else digitalOutputPinArray[referenceIndex] = 1;
    a--;
    referenceIndex++;

    int splitArray[12]; // integers are 32 bit, create a 12 bit array 
    memcpy(splitArray, digitalOutputPinArray + 20,sizeof(splitArray)); //copy over the 12 bits of interest into the smaller array   
  
    setDigitalOutputPinsII(digitalPinTwelve, splitArray[0]); // 2^11 
    setDigitalOutputPinsII(digitalPinEleven, splitArray[1]); // 2^10
    setDigitalOutputPinsII(digitalPinTen, splitArray[2]); // 2^9
    setDigitalOutputPinsII(digitalPinNine, splitArray[3]); // 2^8
    setDigitalOutputPinsII(digitalPinEight, splitArray[4]); //2^7
    setDigitalOutputPinsII(digitalPinSeven, splitArray[5]); // 2^6
    setDigitalOutputPinsII(digitalPinSix, splitArray[6]); // 2^5 
    setDigitalOutputPinsII(digitalPinFive, splitArray[7]); // 2^4
    setDigitalOutputPinsII(digitalPinFour, splitArray[8]); // 2^3
    setDigitalOutputPinsII(digitalPinThree, splitArray[9]); // 2^2
    setDigitalOutputPinsII(digitalPinTwo, splitArray[10]); // 2^1
    setDigitalOutputPinsII(digitalPinOne, splitArray[11]); // 2^0  
    Serial.println();  
  } 
  
  // Toggling the write pin on and off, so that the DAC is only written to after all DAC bit inputs have been changed and finalized  
  delay(1); 
  digitalWrite(digitalPinWR, LOW);    // activating write on active low 
  delay(1); 
  digitalWrite(digitalPinWR, HIGH);   // deactivating write
}

void setDigitalOutputPinsII( int digitalPin, int value)
{
  if( value == 1)
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

