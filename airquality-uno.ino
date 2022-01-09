/*
 * Air Quality sensor
 * Programmed by: Je'aime Powell
 * Target board: Arduino Uno
 * 
 * Sensors:
 * Mics5524 - A3 (5v)
 * This sensor is sensitive to CO ( ~ 1 to 1000 ppm), 
 * Ammonia (~ 1 to 500 ppm), 
 * Ethanol (~ 10 to 500 ppm), 
 * H2 (~ 1 - 1000 ppm), and 
 * Methane / Propane / Iso-Butane (~ 1,000++ ppm). 
 * 
 * Keyestudio PPM2.5 dust sensor - A0 (5v) 
 * 3000 + = Very Bad
 * 1050-3000 = Bad
 * 300-1050 = Ordinary
 * 150-300 = Good
 * 75-150 = Very Good
 * 0-75 = Tiptop
 * 
 * Output LCD:
 * 16x2 SDA/SCL (5v)
 * 
 * Led Strips (Flux Capacitor)
 * D8 - Five LEDs
 * D7, D6 - 7 LEDs
 * 
 * 
 */


#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "FastLED.h"

LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x3F for a 16 chars and 2 line display


// Pins
int dustsensor = 0; //A0
int dustled = 2; //D2
int gassensor = 3; //A3

//Dust Sensor settings
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float dustvoMeasured = 0;
float dustcalcVoltage = 0;
float dustDensity = 0;
float gasvoMeasured =0;

// LED Strips
#define LED_Pin_FIVEB 8
#define FIVEB_NUM 5
#define LED_Pin_SEVENL 7
#define LED_Pin_SEVENR 6
#define SEVENLEDS_NUM 7
#define NUM_STRIPS 3

CRGB five_leds[FIVEB_NUM];
CRGB seven_leds[SEVENLEDS_NUM];
CLEDController *controllers[NUM_STRIPS];
uint8_t gBrightness = 10;   //full brightness = 128 


void setup() {
  Serial.begin(9600);

  //LED Strip
  controllers[0] = &FastLED.addLeds<WS2812,LED_Pin_FIVEB>(five_leds, FIVEB_NUM); 
  controllers[1] = &FastLED.addLeds<WS2812,LED_Pin_SEVENL>(seven_leds, SEVENLEDS_NUM); 
  controllers[2] = &FastLED.addLeds<WS2812,LED_Pin_SEVENR>(seven_leds, SEVENLEDS_NUM); 

  //Dust sensor led pin setup
  pinMode(dustled,OUTPUT);
  digitalWrite(dustled,HIGH); // power off the LED
  
  //Setup LCD
  lcd.init();                      // initialize the lcd 
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.backlight();
  
  lcd.setCursor(0,0);
  lcd.print("Air Quality Test");
  lcd.setCursor(0,1);
  lcd.print("Calibrating...");
  delay(2000);
 
  

}

void loop() {
 

  // LED Strip Test
  controllers[0]->showLeds(gBrightness);  
  // Turn the LED on, then pause
  five_leds[4] = CRGB::Red;
  controllers[1]->showLeds(gBrightness);  
  // Turn the LED on, then pause
  seven_leds[6] = CRGB::Red;
  controllers[2]->showLeds(gBrightness);  
  // Turn the LED on, then pause
  seven_leds[6] = CRGB::Red;
  FastLED.show();
  delay(500);
  





  delay(samplingTime);
  digitalWrite(dustled,LOW); // power on the LED
  delay(deltaTime);
  dustvoMeasured = analogRead(dustsensor); // read the dust value
  gasvoMeasured = analogRead(gassensor); // read the gas value
  
  delay(deltaTime);
 digitalWrite(dustled,HIGH); // power off the LED
  delay(sleepTime);
 
  // 0 - 5V mapped to 0 - 1023 integer values
  // recover voltage
  dustcalcVoltage = dustvoMeasured * (5.0 / 1024.0);

  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = 170 * dustcalcVoltage - 0.1;

  // Air quality based on particulates
  String quality;
  if (dustDensity < 75) {quality = "Excellent";}
  else if (dustDensity > 75 && dustDensity < 150) {quality = "Very Good"; }
  else if (dustDensity > 150 && dustDensity < 300) {quality = "Good"; }
  else if (dustDensity > 300 && dustDensity < 1050) {quality = "Normal"; }
  else if (dustDensity > 1050 && dustDensity < 3000) {quality = "Bad";} 
  else if (dustDensity > 3000) {quality = "Very Bad"; } 

  // Air quality based on VOC and CO gasses
  float gasquality;
  int gassymbol;
  gasquality = map(gasvoMeasured, 1, 1000, 0, 100);

 


  byte Skull[8] = {
0b00000,
0b01110,
0b10101,
0b11011,
0b01110,
0b01110,
0b00000,
0b00000
};

byte Check[8] = {
0b00000,
0b00001,
0b00011,
0b10110,
0b11100,
0b01000,
0b00000,
0b00000
};

byte Smile[8] = {
  0b00000,
  0b01010,
  0b01010,
  0b00000,
  0b00100,
  0b10001,
  0b01110,
  0b00000
};

 lcd.createChar(0, Smile);
 lcd.createChar(1, Check);
 lcd.createChar(2, Skull);

 if (gasquality < 40) { gassymbol = 1; }
 else if (gasquality > 50) { gassymbol = 2; }

  Serial.print("Dust Density = ");
  Serial.print(dustDensity); // unit: ug/m3
  Serial.print(" ");
  Serial.println(quality);
  Serial.print("Gas Measure = ");
  Serial.print(gasvoMeasured); //unit:
  Serial.print(" - ");
  Serial.print(gasquality);
  Serial.println("% VOC to CO Mix"); 

  

  
  // Output to LCD 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Dust:");
  lcd.setCursor(6,0);
  lcd.print(quality);
  lcd.setCursor(0,1);
  lcd.print("VOC/CO:");
  lcd.setCursor(8,1);
  lcd.print(String(gasquality)+"%");
  lcd.setCursor(15,1);
  lcd.write(byte(gassymbol));
  

// LED Strip Test Flux Capacitor Pattern
  for (int i=6;i>=0; i--){
    if (i>4){   
      controllers[0]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      five_leds[4] = CRGB::White;
      controllers[1]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::White;
      controllers[2]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::White;
      FastLED.show();
    }
    else{
      controllers[0]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      five_leds[i] = CRGB::White;
      controllers[1]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::White;
      controllers[2]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::White;
      FastLED.show();
    }
   delay(500);
   if (i == 0) {
    delay (1500);
   }
  }

  
  //Turn LEDS off
  for (int i=0; i< 7; i++){
    if (i > 4){
      controllers[1]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::Black;
      controllers[2]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::Black;
    }
    else {
             // LED Strip Test
      controllers[0]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      five_leds[i] = CRGB::Black;
      controllers[1]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::Black;
      controllers[2]->showLeds(gBrightness);  
      // Turn the LED on, then pause
      seven_leds[i] = CRGB::Black;
    }

 
  }
  
 FastLED.show();
 delay(2000);
}
