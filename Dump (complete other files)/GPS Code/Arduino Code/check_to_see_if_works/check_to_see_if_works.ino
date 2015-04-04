#include <SoftwareSerial.h>
SoftwareSerial GPS = SoftwareSerial(2,3);

void setup()
{
GPS.begin(4800);
Serial.begin(9600);
}

void loop(){
char inByte = GPS.read();
//Serial.write(inByte);
Serial.println(inByte);
}
