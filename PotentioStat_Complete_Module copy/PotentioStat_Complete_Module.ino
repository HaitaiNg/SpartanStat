#include "PotentiostatLibrary.h" 

float delayTimeHigh;
float delayTimeLow;
float quietTime;
float squareWaveAmplitudeVoltage;
float squareWaveStepVoltage;
float startVoltage;
float stopVoltage;

int linearSweepCount;
int linearSweepAlgorithmSlope; 

PotentiostatLibrary potentiostat = PotentiostatLibrary();
void setup()
{
  Serial.begin(9600);
  potentiostat.setPinsToOutput();
  potentiostat.setWritePinToOutput();
}

void loop()
{

  PotentiostatLibrary potentiostat = PotentiostatLibrary();

  Serial.println("---------- Program Executed ------------");
  Serial.println("Please enter P for pulse // L for linear // C for current read");
  while(Serial.available() == 0){}


  if(Serial.available())
  {
    String userInput = Serial.readString(); // Serial read converts characters to ASCII equivalent
    int operationCompleted = 0;

    if(userInput == "P") // If user equals pulse: "P" execute pulse wave
    {
      operationCompleted = executeSquareWave();
      Serial.print("Completed Operation Return Value: ");
      Serial.println(); Serial.println();
    }
    else if (userInput == "L") // If user enters linear: "L" execute linear wave
    {
     Serial.println("Executing linear sweep");
     Serial.println("Enter number of repititions");
     while(Serial.available() == 0) {} // wait for user inpiut
     linearSweepCount = Serial.parseInt();
     potentiostat.setLinearSweepRepititions(linearSweepCount);
    

     Serial.println("Enter Start Voltage (mV):"); 
     while(Serial.available() == 0) {}
     startVoltage = Serial.parseFloat(); 
     Serial.println("Enter Stop Voltage (mV):"); 
     while(Serial.available() == 0) {}
     stopVoltage = Serial.parseFloat(); 
     potentiostat.setLinearParameters(-startVoltage, -stopVoltage);   
     Serial.println("Please indicate whether linear sweep slope is increasing to decreasing");
     Serial.println("-- Options: decreasing to increasing 1 // increasing to decreasing : 2 -- "); 
     Serial.println("Enter slope (1 or 2): "); 
     while(Serial.available() == 0) {} 
     linearSweepAlgorithmSlope = Serial.parseInt(); 
     potentiostat.setLinearSlope(linearSweepAlgorithmSlope); 
     displayLinearSweepParametersToConsole(); 

      
     potentiostat.executeLinearSweep();
     Serial.print("Completed Operation Return Value: ");
     Serial.println(); Serial.println(); 
    }
    else if (userInput == "C")
    {
      int x = 1;

      Serial.println("Executing Read Current");
          while(x > 0)
          {
            operationCompleted = potentiostat.readCurrent();
          }
      Serial.print("Completed Operation Return Value: ");
      Serial.print(operationCompleted);
      Serial.println(); Serial.println();

    }
    else
    {
      Serial.println("Error :: Invalid Input. Please try again \n");
    }
  }
}

int executeSquareWave()
{
  Serial.println("Enter Square Wave Parameters: ");
  Serial.println("Enter Quiet Time (ms)");
  while(Serial.available() == 0){} // Wait for user input
  quietTime = Serial.parseFloat();
  Serial.println("Enter Delay Time High (ms)");
  while(Serial.available() == 0){} // Wait for user input
  delayTimeHigh = Serial.parseFloat();
  Serial.println("Enter Delay Time Low (ms)");
  while(Serial.available() == 0){} // Wait for user input
  delayTimeLow = Serial.parseFloat();
  Serial.println("Enter Square Wave Amplitude Voltage (mV)");
  while(Serial.available() == 0){} // Wait for user input
  squareWaveAmplitudeVoltage = Serial.parseFloat();
  Serial.println("Enter Square Wave Step Voltage (mV)");
  while(Serial.available() == 0){} // Wait for user input
  squareWaveStepVoltage = Serial.parseFloat();
  Serial.println("Enter Start Voltage (mV)");
  while(Serial.available() == 0){} // Wait for user input
  startVoltage = Serial.parseFloat();
  Serial.println("Enter Stop Voltage (mV)");
  while(Serial.available() == 0){} // Wait for user input
  stopVoltage = Serial.parseFloat();
  displaySquareWaveParametersToConsole();

  potentiostat.init(delayTimeHigh, delayTimeLow, quietTime, squareWaveAmplitudeVoltage, squareWaveStepVoltage, startVoltage, stopVoltage);
  Serial.println();
  return 1;
}

void displaySquareWaveParametersToConsole()
{

  Serial.println();
  Serial.print("Quiet Time (ms): ");
  Serial.println(quietTime);
  Serial.print("Delay Time High (ms): ");
  Serial.println(delayTimeHigh);
  Serial.print("Delay Time Low (ms): ");
  Serial.println(delayTimeLow);
  Serial.print("Square Wave Amplitude Voltage (mV): ");
  Serial.println(squareWaveAmplitudeVoltage);
  Serial.print("Square Wave Step Voltage (mV): ");
  Serial.println(squareWaveStepVoltage);
  Serial.print("Start Voltage (mV): ");
  Serial.println(startVoltage);
  Serial.print("Stop Voltage (mV): ");
  Serial.println(stopVoltage);
}

void displayLinearSweepParametersToConsole()
{
  Serial.println();
  Serial.print("Number of repititions: "); 
  Serial.println(linearSweepCount); 
  Serial.print("Stop Voltage (mV): ");
  Serial.println(startVoltage);
  Serial.print("Start Voltage (mV): ");
  Serial.println(stopVoltage); 
  Serial.println(); 
}


