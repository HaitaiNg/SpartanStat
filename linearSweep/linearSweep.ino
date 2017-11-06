
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

int digitalPinWR = 37;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);  // initialize serial communication at 9600 bits per second
  Serial.println("Executing Program"); // print to serial to indicate program has started

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

  pinMode(digitalPinWR, OUTPUT); // Set the write pin 
}

void loop()
{
    linearSweepAlgorithm(); // this function is continously called 
}


void linearSweepAlgorithm()
{
  double startVoltage = 5.00; // Voltage range is [-2.5 , 2.5] == 5V
  double stepSize = 4096.00; // This is currently hardcoded. Sample Size
  double incrementSize = startVoltage / stepSize; // Step Size (1.2 mv)

  for (double i = 0; i < 5; i += incrementSize)
  {
    double currentStep = i;
    int digitalEquivalentValue = determineDigitalPinToSet(currentStep, incrementSize);
    //Serial.println(digitalEquivalentValue);
    setDigitalOutputPins(digitalEquivalentValue);
  }
}

double determineDigitalPinToSet(double currentStep, double incrementSize)
{
  return currentStep / incrementSize;
}

void setDigitalOutputPins( int  digitalEquivalentValue)
{
  int a = 31; // 32 bit number int
  int referenceIndex = 0; 
  int splitArray[12]; // integers are 32 bit, create a 12 bit array 
  while ( a >= 0)
  {
    int digitalOutputPinArray[32]; // initalize the array 
    int n = digitalEquivalentValue >> a; // bit shifting to signed
    if (!(n % 2)) digitalOutputPinArray[referenceIndex] = 0 ;
    else digitalOutputPinArray[referenceIndex] = 1;
    a--;
    referenceIndex++;
    memcpy(splitArray, digitalOutputPinArray + 20,sizeof(splitArray)); //copy over the 12 bits of interest into the smaller array   
  }

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
  delay(1); 
  digitalWrite(digitalPinWR, LOW); //activating on active low 
  delay(1); 
  digitalWrite(digitalPinWR, HIGH); 
  delay(1);  
   
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




