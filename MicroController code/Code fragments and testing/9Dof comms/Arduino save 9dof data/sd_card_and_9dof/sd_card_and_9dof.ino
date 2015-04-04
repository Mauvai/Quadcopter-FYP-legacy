/*
  SD card read/write
 
 This example shows how to read and write data to and from an SD card file 	
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 
 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 	 
 */
 
#include <SD.h>
#include <Wire.h>
 
#define GYROADDR 0x68
#define COMPASSADDR 0x1e
#define ACCELADDR 0x53
 
union XYZBuffer {
  struct {
    short x,y,z;
  } value;
  byte buff[6];
};
File myFile;
int fileNo = 0;
volatile int prev=  0;



void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
   pinMode(10, OUTPUT);
   
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
	// close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  
  // re-open the file for reading:
  /*
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");
    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
    	Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
  } else {
  	// if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }*/
  myFile.close();
  Wire.begin();        // join i2c bus (address optional for master)
  setupCompass(COMPASSADDR);
  setupAccel(ACCELADDR);
  setupGyro(GYROADDR);
  
  pinMode(3, OUTPUT);      
  pinMode(2, INPUT);   
  
  
  //delay(5000);
  
}

void loop()
{
  
  if (digitalRead(2) == HIGH)
  {
    digitalWrite(3, HIGH);
    
    if (prev == 0)
    {
      switch(fileNo)
      {
        case 0:
          Serial.println("Case 0");
          myFile = SD.open("file_0.txt", FILE_WRITE);
          break;
        case 1:
          myFile = SD.open("file_1.txt", FILE_WRITE);
          break;
        case 2:
          myFile = SD.open("file_2.txt", FILE_WRITE);
          break;
        case 3:
          myFile = SD.open("file_3.txt", FILE_WRITE);
          break;
        case 4:
          myFile = SD.open("file_4.txt", FILE_WRITE);
          break;
        case 5:
          myFile = SD.open("file_5.txt", FILE_WRITE);
          break;
        default:
          myFile = SD.open("file_6.txt", FILE_WRITE);
          break;
      }
      
      fileNo++;
    }
    prev = 1;
    digitalWrite(3, HIGH);
    union XYZBuffer compass,gyro,accel;
    
    readAccel(ACCELADDR,&accel);
    readCompass(COMPASSADDR,&compass);
    readGyro(GYROADDR,&gyro);
    myFile.print("A,");
    output(accel);
    myFile.print(",G,");  
    output(gyro);
    
    myFile.print("\r\n");
  }
  else if (prev == 1)
  {
    digitalWrite(3, LOW);
    myFile.close();
    prev = 0;
    Serial.println("File written");
  }
  
}



//functions


void changeEndian(union XYZBuffer *xyz) {
  for (int i=0;i<6;i+=2) {
    byte t=xyz->buff[i];
    xyz->buff[i]=xyz->buff[i+1];
    xyz->buff[i+1]=t;
  }
}
 
// Generically useful reading into a union type
void readXYZ(int device,union XYZBuffer *xyz) {    
  Wire.requestFrom(device, 6);      
  long start=millis();
  while (!Wire.available() && (millis()-start)<100);
  if (millis()-start<100) {
    for (int i=0;i<6;i++)
      xyz->buff[i]=Wire.read();
  }
}
 
void setupAccel(int device) {
  // Check ID to see if we are communicating
  Wire.beginTransmission(device);
  Wire.write(0x00); // One Reading
  Wire.endTransmission();
  Wire.requestFrom(device,1);
  while (!Wire.available());  
  byte ch=Wire.read();
  Serial.print("Accel id is 0x");
  Serial.println(ch,HEX);
  // Should output E5
 
  // https://www.sparkfun.com/datasheets/Sensors/Accelerometer/ADXL345.pdf
  // Page 16
  Wire.beginTransmission(device);
  Wire.write(0x2d);
  Wire.write(0x08);
  Wire.endTransmission();
  
  Wire.beginTransmission(device);
  Wire.write(0x38);
  Wire.write(0x00);
  Wire.endTransmission();
  
  Wire.beginTransmission(device);
  Wire.write(0x31);
  Wire.write(0x00);
  Wire.endTransmission();
 
}
void readAccel(int device,union XYZBuffer *xyz) {
  Wire.beginTransmission(device);
  Wire.write(0x32); // One Reading
  Wire.endTransmission();
  readXYZ(device,xyz);
}
 
void setupCompass(int device) {
  // Check ID to see if we are communicating
  Serial.print("Compass id is ");
  Wire.beginTransmission(device);
  Wire.write(10); // One Reading
  Wire.endTransmission();
  Wire.requestFrom(device,2);
  while (!Wire.available());
  char ch=Wire.read();
  Serial.print(ch);  
  ch=Wire.read();
  Serial.println(ch);
  // Should output H4  
 
// Page 18
// at http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf
  Wire.beginTransmission(device);
  Wire.write(0x00); Wire.write(0x70);
  Wire.endTransmission();
  Wire.beginTransmission(device);
  Wire.write(0x01); Wire.write(0xA0);
  Wire.endTransmission();
  Wire.beginTransmission(device);
  Wire.write(0x02); Wire.write(0x00); //  Reading
  Wire.endTransmission();
  delay(6);
}
void readCompass(int device,union XYZBuffer *xyz) {
  readXYZ(device,xyz);
  changeEndian(xyz);
  Wire.beginTransmission(device);
  Wire.write(0x03);
  Wire.endTransmission();
}
 
void setupGyro(int device) {
  // Check ID to see if we are communicating
  Wire.beginTransmission(device);
  Wire.write(0x00); // One Reading
  Wire.endTransmission();
  Wire.requestFrom(device,1);
  while (!Wire.available());  
  byte ch=Wire.read();
  Serial.print("Gyro id is 0x");
  Serial.println(ch,HEX);  
  // Should output 69
  
  
  Wire.beginTransmission(device);
  Wire.write(0x3E);
  Wire.write(0x01);
  Wire.endTransmission();
  
  Wire.beginTransmission(device);
  Wire.write(0x15);
  Wire.write(0x09);
  Wire.endTransmission();
  
  Wire.beginTransmission(device);
  Wire.write(0x16);
  Wire.write(0x1A);
  Wire.endTransmission();
  
}
void readGyro(int device,union XYZBuffer *xyz) {
  // https://www.sparkfun.com/datasheets/Sensors/Gyro/PS-ITG-3200-00-01.4.pdf
  // page 20
  Wire.beginTransmission(device);
  Wire.write(0x1d);
  Wire.endTransmission();
  readXYZ(device,xyz);
  changeEndian(xyz);  
}
 
void pad(int width,int number) {
  int n=abs(number);
  int w=width;
  if (number<0) w--;
  while (n>0) {
    w--;
    n/=10;
  }
  if (number==0) w--;
  for (int i=0;i<w;i++) Serial.print(' ');
}
 
void output(union XYZBuffer xyz) {
//  pad(6,xyz.value.x);
  myFile.print(xyz.value.x);
  myFile.print(',');
//  pad(6,xyz.value.y);
  myFile.print(xyz.value.y);
  myFile.print(',');
//  pad(6,xyz.value.z);
  myFile.print(xyz.value.z);
}

