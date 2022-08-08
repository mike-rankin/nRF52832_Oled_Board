//Install Adafruit_SSD1306.h
//Install ClosedCube_HDC1080.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ClosedCube_HDC1080.h"

Adafruit_SSD1306 display = Adafruit_SSD1306(96, 16, &Wire, 23);

ClosedCube_HDC1080 hdc1080;


  //#define BUTTON_A 31
  //#define BUTTON_B 30
  //#define BUTTON_C 27
//#else // 32u4, M0, M4, nrf52840 and 328p
//  #define BUTTON_A  9
//  #define BUTTON_B  6
//  #define BUTTON_C  5
//#endif

void setup() {
  Serial.begin(9600);
  Serial.println("Hardware Test");

  hdc1080.begin(0x40);

  Serial.print("Manufacturer ID=0x");
  Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
  Serial.print("Device ID=0x");
  Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device
  
  display.setRotation(2);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //display.display();
  //delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  Serial.println("Hardware test");

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("Hardware Test");
  display.display(); // actually display all of the above
  delay(5000);
  //display.clearDisplay();
}

void loop() 
{
  display.clearDisplay();
  Serial.print("T=");
  Serial.print(hdc1080.readTemperature());
  Serial.print("C, RH=");
  Serial.print(hdc1080.readHumidity());
  Serial.println("%");

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,5);
  display.print("T=");
  display.print(hdc1080.readTemperature());
  display.print(" H=");
  display.print(hdc1080.readHumidity());
  display.print("%");
  
  delay(10);
  yield();

  
  display.display();
}



void printSerialNumber() {
  Serial.print("Device Serial Number=");
  HDC1080_SerialNumber sernum = hdc1080.readSerialNumber();
  char format[12];
  sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
  Serial.println(format);
}
