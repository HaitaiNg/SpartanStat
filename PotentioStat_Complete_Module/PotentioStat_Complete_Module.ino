#include "PotentiostatLibrary.h" 

PotentiostatLibrary potentiostat = PotentiostatLibrary(); 

void setup() 
{
  Serial.begin(9600); 
  Serial.println("Program Executed"); 
  potentiostat.setPinsToOutput(); 
  potentiostat.setWritePinToOutput(); 
}

void loop() 
{
  potentiostat.executeLinearSweep(); 
  Serial.println("Execute Program");
  delay(1000000); 
}
