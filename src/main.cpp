#include <Arduino.h>
#include <AsyncTrigger.h>

AsyncTrigger Trigger1(1000); // Trigger every 1000 milliseconds
AsyncTrigger Trigger2(314);  // Trigger every 500 milliseconds

void setup()
{

  Serial.begin(115200);
}

void loop()
{
  if (Trigger1.check())
  {
    Serial.print(" Trigger1 fired! ");
  }

  if (Trigger2.check())
  {
    Serial.print(" Trigger2 fired! ");
  }
  Serial.println();
  delay(100);
}