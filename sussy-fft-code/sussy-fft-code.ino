// -------------------- FFT SETUP --------------------
#include "arduinoFFT.h"
#define SAMPLES 256             //Must be a power of 2
#define SAMPLING_FREQUENCY 10000 //Hz, must be less than 10000 due to ADC

arduinoFFT FFT = arduinoFFT();

unsigned int sampling_period_us;
unsigned long microseconds;

double vReal[SAMPLES];
double vImag[SAMPLES];
double peak;

#define MIC_PIN A0

// -------------------- OLED --------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "bmp.h"
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // < See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char peakString[10];

float frequencies[] { 130.8, 138.6, 146.8, 155.6, 164.8, 174.6, 185.0, 196.0, 207.7, 220.0, 232.2, 247.0 };
String notes[] { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
int freqLength = sizeof frequencies / sizeof frequencies[0];

#define AMP_THRESH 500
#define NOISE_THRESH 200
#define NUM_BANDS 128
int bands[NUM_BANDS];
int band_width = 128 / NUM_BANDS;
int influence_thresh = (int)((SAMPLES / 2) / NUM_BANDS);
int max_amp = 0;


// -------------------- RGB LED --------------------
#define R_PIN 14 //D5
#define G_PIN 12 //D6
#define B_PIN 13 //D7
byte color[] { 0, 0, 0 };
double h;


// -------------------- RGB LED STRIP --------------------
#include "FastLED.h"
#define DATA_PIN    2
//#define CLK_PIN   4
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_LEDS    200
#define BRIGHTNESS  32

CRGB leds[NUM_LEDS];


// -------------------- OTA --------------------
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
unsigned long previousBlinkMillis = 0;
const long blinkInterval = 1000;
int ledState = LOW;

#include "credentials.h"

void printDisplay(String msg, int x = 0, int y = 0, bool newLine = true) {
  if (newLine) Serial.println(msg);
  else Serial.print(msg);
  display.setCursor(x, y);
  display.write(msg.c_str());
  display.display();
}

void setup() {
  Serial.begin(115200);

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  pinMode(R_PIN, OUTPUT);
  pinMode(G_PIN, OUTPUT);
  pinMode(B_PIN, OUTPUT);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("[DISPLAY] SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text

  printDisplay("[WIFI] Booting...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    display.clearDisplay();
    printDisplay("[WIFI] Connection Failed! Rebooting...");

    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    display.clearDisplay();
    printDisplay("[OTA] Update Type: ", 0, 0, false);
    printDisplay(type, 0, 16);

    ledState = HIGH;
    digitalWrite(R_PIN, ledState);
    digitalWrite(G_PIN, LOW);
    digitalWrite(B_PIN, LOW);
  });
  ArduinoOTA.onEnd([]() {
    display.clearDisplay();
    printDisplay("[OTA] End!!!");
    ledState = LOW;
    digitalWrite(R_PIN, ledState);
    digitalWrite(G_PIN, HIGH);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));

    display.clearDisplay();
    display.setCursor(0, 0);
    display.write("[OTA] Progress: ");
    display.setCursor(100, 0);
    display.write(progress / (total / 100));
    display.write("%");

    display.fillRect(0, 16, (int)(progress / total * 128), 16, SSD1306_WHITE);

    display.display();

    if (millis() - previousBlinkMillis >= blinkInterval) {
      previousBlinkMillis = millis();
      ledState = not(ledState);
      digitalWrite(R_PIN,  ledState);
      digitalWrite(G_PIN, LOW);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.write("[OTA] Error[");
    display.write(error);
    display.write("]:");
    if (error == OTA_AUTH_ERROR)          printDisplay("Auth Failed", 0, 16);
    else if (error == OTA_BEGIN_ERROR)    printDisplay("Begin Failed", 0, 16);
    else if (error == OTA_CONNECT_ERROR)  printDisplay("Connect Failed", 0, 16);
    else if (error == OTA_RECEIVE_ERROR)  printDisplay("Receive Failed", 0, 16);
    else if (error == OTA_END_ERROR)      printDisplay("End Failed", 0, 16);
    ledState = HIGH;
    digitalWrite(R_PIN, ledState);
  });
  ArduinoOTA.begin();

  display.clearDisplay();
  printDisplay("[OTA] Ready", 0, 0);
  printDisplay("[WIFI] IP address: ", 0, 10, false);
  printDisplay(WiFi.localIP().toString().c_str(), 0, 20);

  delay(3000);
  // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
  .setCorrection(TypicalLEDStrip)
  .setDither(BRIGHTNESS < 255);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  ArduinoOTA.handle();

  /*SAMPLING*/
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();    //Overflows after around 70 minutes!

    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;

    while (micros() < (microseconds + sampling_period_us)) {
      /* wait by doing nothing */
    }
  }

  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  /*PROCESS RESULTS*/
  for (int i = 2; i < (SAMPLES / 2); i++) {
    /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
    //    //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
    //    //Serial.print(" ");
    //    //Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins

    int curr_band = (int)(i / influence_thresh) - 1;
    bands[curr_band] += vReal[i] / influence_thresh;
    max_amp = vReal[i] > max_amp ? vReal[i] : max_amp;
  }

//  if nothing has an amplitude more than NOISE_THRESH
  if (max_amp < NOISE_THRESH) {

    display.clearDisplay();
    display.drawBitmap(
      (display.width()  - BMP_WIDTH ) / 2,
      (display.height() - BMP_HEIGHT) / 2,
      bmp_open, BMP_WIDTH, BMP_HEIGHT, 1);
    display.display();
    
    digitalWrite(R_PIN, HIGH);
    digitalWrite(G_PIN, HIGH);
    digitalWrite(B_PIN, HIGH);
  } else {
    display.clearDisplay();
    for (int j = 0; j < NUM_BANDS; j++) {
      bands[j] = (int)map(bands[j], 0, AMP_THRESH, 0, 32);
      display.drawRect(j * band_width, 32, band_width, -bands[j], SSD1306_WHITE);
    }
    display.display();

    dtostrf(peak, 10, 2, peakString);
    display.setCursor(64, 0);
    display.write(peakString);
    display.write(" ");

    // transform frequency into 3rd octave
    while (peak < frequencies[0]) peak *= 2.0;
    while (peak > frequencies[freqLength - 1]) peak *= 0.5;
    //  display.setCursor(64, 10);
    //  display.write(findFrequency(peak).c_str());
    display.display();

    h = map(peak, frequencies[0], frequencies[freqLength - 1], 0, 360);
    hslToRgb(h, 1.0, 0.5, color);

    analogWrite(R_PIN, 255 - color[0]);
    analogWrite(G_PIN, 255 - color[1]);
    analogWrite(B_PIN, 255 - color[2]);

    //  pride();
    updateLEDs();
    FastLED.show();
  }

  max_amp = 0;
  delay(1);
}

String findFrequency(float freq) {
  int closestIndex = 0;
  double diff = frequencies[closestIndex];

  for (int i = 0; i < freqLength; i++) {
    diff = (abs(frequencies[i] - freq) < diff) ? abs(frequencies[i] - freq) : diff;
    if (diff == abs(frequencies[i] - freq)) {
      closestIndex = i;
    }
  }
  return notes[closestIndex];
}

double hue2rgb(double p, double q, double t) {
  if (t < 0) t += 1;
  if (t > 1) t -= 1;
  if (t < 1 / 6.0) return p + (q - p) * 6 * t;
  if (t < 1 / 2.0) return q;
  if (t < 2 / 3.0) return p + (q - p) * (2 / 3.0 - t) * 6;
  return p;
}

void hslToRgb(double h, double s, double l, byte rgb[]) {
  double r, g, b;
  h /= 360;

  if (s == 0) {
    r = g = b = l; // achromatic
  } else {
    double q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    double p = 2 * l - q;
    r = hue2rgb(p, q, h + 1 / 3.0);
    g = hue2rgb(p, q, h);
    b = hue2rgb(p, q, h - 1 / 3.0);
  }

  rgb[0] = r * 255;
  rgb[1] = g * 255;
  rgb[2] = b * 255;
}

void updateLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(color[0], color[1], color[2]);
  }
}

// This function draws rainbows with an ever-changing,
// widely-varying set of parameters.
//void pride()
//{
//  static uint16_t sPseudotime = 0;
//  static uint16_t sLastMillis = 0;
//  static uint16_t sHue16 = 0;
//
//  uint8_t sat8 = beatsin88( 87, 220, 250);
//  uint8_t brightdepth = beatsin88( 341, 96, 224);
//  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
//  uint8_t msmultiplier = beatsin88(147, 23, 60);
//
//  uint16_t hue16 = sHue16;//gHue * 256;
//  uint16_t hueinc16 = beatsin88(113, 1, 3000);
//
//  uint16_t ms = millis();
//  uint16_t deltams = ms - sLastMillis ;
//  sLastMillis  = ms;
//  sPseudotime += deltams * msmultiplier;
//  sHue16 += deltams * beatsin88( 400, 5,9);
//  uint16_t brightnesstheta16 = sPseudotime;
//
//  for( uint16_t i = 0 ; i < NUM_LEDS; i++) {
//    hue16 += hueinc16;
//    uint8_t hue8 = hue16 / 256;
//
//    brightnesstheta16  += brightnessthetainc16;
//    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;
//
//    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
//    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
//    bri8 += (255 - brightdepth);
//
//    CRGB newcolor = CHSV( hue8, sat8, bri8);
//
//    uint16_t pixelnumber = i;
//    pixelnumber = (NUM_LEDS-1) - pixelnumber;
//
//    nblend( leds[pixelnumber], newcolor, 64);
//  }
//}
