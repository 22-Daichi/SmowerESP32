#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17
}

void loop()
{
  if (Serial1.available())
  {
    String s = Serial1.readStringUntil('\n');
    Serial.println("Jetsonから: " + s);
  }
}