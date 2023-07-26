#include <SoftwareSerial.h>
/*
Modified by: Shreyas
Date:        26 Jul 2023
Description: Firmware for Blue Robotics Lumen LEDs to be controlled
             via RS232 on our vehicle. Extends on the code provided by a member
             on the Blue Robotics forum.
             https://discuss.bluerobotics.com/t/lumen-control-using-the-onboard-attiny45-20/1758/25
             If the board temperature hits 85C, the lights will flash a couple times and
             operate on reduced brightness, until temperature reduces to 70C
Steps:       Compile here with board ATtiny25/45/85, Clock "External 8MHz", Processor "ATtiny45"
             Then flash the .hex file using avrdude and USBTinyISP programmer
             .\avrdude.exe -c usbtiny -p attiny45 -U flash:w:lumen-mod.ino.hex:a

Blue Robotics Lumen LED Embedded Software
------------------------------------------------
 
Title: Lumen LED Light Embedded Microcontroller Software, modified by Seatools Pty Ltd
Description: This file is used on the ATtiny45 microcontroller
on the Lumen LED Light and controls the dimming of the light, using serial commands.
Additionally, a thermistor is used to sense temperature and automatically
dim the light to level 1 if it is overheating.
-------------------------------
The MIT License (MIT)
Copyright (c) 2016 Blue Robotics Inc.
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

// HARDWARE PIN DEFINITIONS
#define LED_PIN 1    // pin 6
#define TEMP_PIN A1  // pin 7
#define rxPin 0      // D0, Pin 5
#define txPin -1     // undefined pin, ignore

// set up serial port
// SoftwareSerial(rx, tx, inverted) - setting inverted to true to read RS232 directly
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin, true);

char c;
bool firstDim = true;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  // define pin mode for rx:
  pinMode(rxPin, INPUT);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  // Setup up PWM on output pin
  // The default analogWrite frequency can mess with cameras
  TCCR0B = _BV(CS01);  // Set prescalar to 8 for 8M/8/256 = 3906 Hz
}

void turnLedOnFromInput(char c) {
  switch(c) {
      case 0x30:
        digitalWrite(LED_PIN, LOW);
        break;
      case 0x31:
        analogWrite(LED_PIN, 21);
        break;
      case 0x32:
        analogWrite(LED_PIN, 70);
        break;
      case 0x33:
        analogWrite(LED_PIN, 120);
        break;
      case 0x34:
        analogWrite(LED_PIN, 170);
        break;
      case 0x35:
        analogWrite(LED_PIN, 230);
        break;
      default:
        digitalWrite(LED_PIN, HIGH);
    }
}

void loop() {
  // Read serial input if available, use serial code to set dimming level
  while (mySerial.available()) {
    c = mySerial.read();

    // Output PWM to LED driver based on character read from serial input 
    turnLedOnFromInput(c);
  }

  // temperature dimming - reduce to level 1 and flash a couple of times
  if (analogRead(TEMP_PIN) <= 265 && c != '0') {
    if (firstDim) {
      analogWrite(LED_PIN, 21);
      delay(500);
      analogWrite(LED_PIN, LOW);
      delay(500);
      analogWrite(LED_PIN, 21);
      delay(500);
      analogWrite(LED_PIN, LOW);
      delay(500);
      analogWrite(LED_PIN, 21);
      firstDim = false;
    }
  }

  if (analogRead(TEMP_PIN) > 355 && firstDim == false) {
    turnLedOnFromInput(c);
    firstDim = true;
  }

  delay(50);
}