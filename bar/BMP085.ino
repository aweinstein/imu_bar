/* BMP085 Extended Example Code
  by: Jim Lindblom
  SparkFun Electronics
  date: 1/18/11
  license: CC BY-SA v3.0 - http://creativecommons.org/licenses/by-sa/3.0/
  
  Get pressure and temperature from the BMP085 and calculate altitude.
  Serial.print it out at 9600 baud to serial monitor.

  Update (7/19/11): I've heard folks may be encountering issues
  with this code, who're running an Arduino at 8MHz. If you're 
  using an Arduino Pro 3.3V/8MHz, or the like, you may need to 
  increase some of the delays in the bmp085ReadUP and 
  bmp085ReadUT functions.
*/

#include <Wire.h>
#include "TimerOne.h"

#define BMP085_ADDRESS 0x77  // I2C address of BMP085

#define LED 13

const unsigned char OSS = 3;  // Oversampling Setting

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 

short temperature;
short temperature_filt;
long pressure, pressure_filt;

#define W 8
int temperatures[W];

byte Timer1_Flag = 0;

unsigned int ut;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  bmp085Calibration();
  pinMode(LED, OUTPUT);
  Timer1.initialize(200000); // time in microseconds
  Timer1.attachInterrupt(timer1_ISR);
  temperature = bmp085GetTemperature(bmp085ReadUT());
  for (byte i=0; i<W; i++) {
    temperatures[i] = bmp085GetTemperature(bmp085ReadUT());
  }
  temperature_filt = temperatures[W-1];
  ut = bmp085ReadUT();
}

void loop()
{
  static byte i=0;
  static byte temperature_count = 0;
  static byte j;
  unsigned long up;
  
  // Read temperature once per second
  if (Timer1_Flag) {
    Timer1_Flag = 0;
    if (++temperature_count == 5) { // read temperature every 1 second
      ut = bmp085ReadUT();
      temperature = bmp085GetTemperature(ut);
      //temperature_filt += (temperature - temperatures[0]) / W;
      //temperatures[W-1] = temperature;
      //bubble_sort(temperatures, W);
      //temperature_filt = temperatures[W/2];
      //for(j = 0; j < W - 1; j++) {
       //temperatures[j] = temperatures[j+1];
      //}
      //temperatures[W-1] = temperature_filt;
      
      temperature_count = 0;
      //Serial.println("reading temperature");
    }
    up = bmp085ReadUP();
    pressure = bmp085GetPressure(up, 0);
    //pressure_filt = bmp085GetPressure(up, 1);
    Serial.print('#');
    Serial.print(temperature, DEC);
    Serial.print(',');
    //Serial.print(temperature_filt, DEC);
    //Serial.print(',');
    Serial.print(pressure, DEC);
    ///Serial.print(',');
    //Serial.print(pressure_filt, DEC);
    //Serial.print(',');
    //Serial.print(up, DEC);
    //Serial.print(',');  
    //Serial.print(ut, DEC);
    Serial.print("$\n");
    digitalWrite(LED, 0);
  }
}

void bubble_sort(int *a, int n) {
	int j, t = 1;
	while (n-- && t)
		for (j = t = 0; j < n; j++) {
			if (a[j] <= a[j + 1]) continue;
			t = a[j], a[j] = a[j + 1], a[j + 1] = t;
			t=1;
		}
}
// Source: http://rosettacode.org/wiki/Sorting_algorithms/Quicksort#C
/*void quicksort (int *a, int n) {
    if (n < 2)
        return;
    int p = a[n / 2];
    int *l = a;
    int *r = a + n - 1;
    while (l <= r) {
        if (*l < p) {
            l++;
            continue;
        }
        if (*r > p) {
            r--;
            continue; // we need to check the condition (l <= r) every time we change the value of l or r
        }
        int t = *l;
        *l++ = *r;
        *r-- = t;
    }
    quick_sort(a, r - a + 1);
    quick_sort(l, a + n - l);
}*/

void timer1_ISR()
{
  digitalWrite(LED, 1);
  Timer1_Flag = 1;
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up, byte use_filtered)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  long b5_filtered;
  
  if (use_filtered) {
    b5_filtered = (temperature_filt << 4); // Substracting 8 makes the rounding error worse
    //b5_filtered = (int)( temperature_filt * 16.0);
    b6 = b5_filtered - 4000;
    /*Serial.print(b5, DEC);
    Serial.print(",");
    Serial.print(b5_filtered, DEC);
    Serial.print(",");
    Serial.print(b5 - b5_filtered, DEC);
    Serial.print("\n");*/
  } else {   
    b6 = b5 - 4000;
  }
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}

