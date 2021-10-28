// -------------------- FFT SETUP --------------------
#include "arduinoFFT.h" 
#define SAMPLES 64             //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC
 
arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

#define MIC_PIN A0

// -------------------- OLED --------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // < See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

char peakString[8];

float frequencies[] { 130.8, 138.6, 146.8, 155.6, 164.8, 174.6, 185.0, 196.0, 207.7, 220.0, 232.2, 247.0 };
String notes[] { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
int freqLength = sizeof frequencies / sizeof frequencies[0];

// -------------------- RGB LED --------------------
#define R_PIN 6
#define G_PIN 5
#define B_PIN 3
byte color[] { 0, 0, 0 };
double h;

void setup() {
    Serial.begin(115200);
    
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    // Clear the buffer
    display.clearDisplay();
 
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));

    pinMode(R_PIN, OUTPUT);
    pinMode(G_PIN, OUTPUT);
    pinMode(B_PIN, OUTPUT);
}
 
void loop() {
    
    /*SAMPLING*/
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
     
        vReal[i] = analogRead(MIC_PIN);
        vImag[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){ }
    }
 
    /*FFT*/
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
 
    /*PRINT RESULTS*/
    //Serial.println(peak);     //Print out what frequency is the most dominant.

    vReal[0] = 0;
    vReal[1] = 0;
    
    for(int i=0; i<(SAMPLES/2); i++)
    {
        /*View all these three lines in serial terminal to see which frequencies has which amplitudes*/
        //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
        //Serial.print(" ");
        Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
    }

    dtostrf(peak, 6, 2, peakString);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE); // Draw white text
    
    display.write(peakString);
    display.write(" ");

    // transform frequency into 3rd octave
    while (peak < frequencies[0]) peak *= 2.0;
    while (peak > frequencies[freqLength-1]) peak *= 0.5; 
    display.write(findFrequency(peak).c_str());
    
    display.display();

    h = map(peak, frequencies[0], frequencies[freqLength-1], 0, 360);
    hslToRgb(h, 1.0, 0.5, color);
    
    analogWrite(R_PIN, 255-color[0]);
    analogWrite(G_PIN, 255-color[1]);
    analogWrite(B_PIN, 255-color[2]);
 
    delay(1);
    //while(1);       //Run code once
}

String findFrequency(float freq) {
  int closestIndex = 0;
  double diff = frequencies[closestIndex];
  
  for (int i=0; i<freqLength; i++) {
    diff = min(abs(frequencies[i]-freq), diff);
    if (diff == abs(frequencies[i]-freq)) {
      closestIndex = i;
    }
  }
  return notes[closestIndex];
}

double hue2rgb(double p, double q, double t) {
    if(t < 0) t += 1;
    if(t > 1) t -= 1;
    if(t < 1/6.0) return p + (q - p) * 6 * t;
    if(t < 1/2.0) return q;
    if(t < 2/3.0) return p + (q - p) * (2/3.0 - t) * 6;
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
        r = hue2rgb(p, q, h + 1/3.0);
        g = hue2rgb(p, q, h);
        b = hue2rgb(p, q, h - 1/3.0);
    }

    rgb[0] = r * 255;
    rgb[1] = g * 255;
    rgb[2] = b * 255;
}
