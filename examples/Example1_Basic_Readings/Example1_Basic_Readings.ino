
#include <Wire.h>
#include "MAX30102.h"

MAX30102 particleSensor;

#define debug Serial 

void setup()
{
  debug.begin(9600);
  debug.println("MAX30102 Basic Readings Example");

  // 初始化传感器 
  if (particleSensor.begin() == false)
  {
    debug.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }

  particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive
}

void loop()
{
  debug.print(" R[");
  debug.print(particleSensor.getRed());
  debug.print("] IR[");
  debug.print(particleSensor.getIR());
  debug.print("]");

  debug.println();
}
