//Install Adafruit_SSD1306.h
//Install uFire_SHT20.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "uFire_SHT20.h"

Adafruit_SSD1306 display = Adafruit_SSD1306(96, 16, &Wire, 23);

uFire_SHT20 sht20;


void setup() 
{
  Serial.begin(9600);
  Serial.println("Hardware Test");

  Wire.begin();
  sht20.begin();

  display.setRotation(2);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C 96x16 oled display

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
  Serial.println((String)sht20.temperature() + "°C");
  Serial.println((String)sht20.tempF + "°F");

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print("T=");
  display.print(sht20.temperature());

  delay(10);
  yield();

  display.display();
}
