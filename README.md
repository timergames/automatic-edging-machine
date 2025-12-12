# automatic-edging-machine
A male masturbator that automatically stops before you cum (open source project)

Licence - PRIVATE USE IS FREE. COMMERCIAL USE PROHIBITED.

Yes - Print and use the 3D prints of the 3D model for private use;
Yes - Share the images of your 3D prints of the 3D model on communication media such as social networks or websites.

No - commercial use or public sharing of the 3D model;
No - modification or adaptation of the 3D model for public sharing or sale;
No - distribution, sale, donation or exchange of the digital files of the 3D model.

(C) Copyright 2025 Badsub Designs. All rights reserved.

Project Overview 

Download the STL files from https://cults3d.com/en/3d-model/naughties/automatic-edging-machine-arduino-project 

Diagram

Here is the logical flow:
[12V Power Supply] -> [L293D Driver (Bridged)] -> [DC Motor]
       |                       ^
       V                       | (Control Signals: Pins 9, 10, 11)
[Common GND] <-----> [Arduino Uno]
                       ^       ^      ^
(I2C Data: A4, A5) >---|       |      |----< [Button 2 (Pin 3)]
                       v       |
    [MAX30105 Sensor] & [LCD Screen]  |----< [Button 1 (Pin 2)]
    
Detailed Component-by-Component Wiring Guide

This guide assumes you are using a standard breadboard.

1. Power Foundation (Crucial Step)Before connecting components, establish your power rails.
2. Arduino Power: Connect the Arduino to your computer via USB.
3. 12V Motor Power: Connect the positive (+) wire of your 12V supply to the bottom Red rail of your breadboard. Connect the negative (-) wire to the bottom Blue (GND) rail.
4. Common Ground Link: Connect a jumper wire from an Arduino GND pin to the breadboard's bottom Blue (GND) rail. If you forget this, the motor won't work.
5. 5V Logic Power: Connect a jumper wire from Arduino 5V to the top Red rail of the breadboard. Connect another Arduino GND to the top Blue (GND) rail.
The I2C "Data Highway" (LCD & Sensor)
The LCD and Sensor share the same two data pins on the Arduino (A4 and A5).
Component PinConnects ToBreadboard/Arduino Rai
LCD GND->Top Blue Rail (GND)
LCD VCC->Top Red Rail (5V)
LCD SDA->Arduino Pin A4
LCD SCL->Arduino Pin A5
Sensor GND->Top Blue Rail (GND)
Sensor VIN/VCC->Top Red Rail (5V)
Sensor SDA->Arduino Pin A4 (Shares hole with LCD)
Sensor SCL->Arduino Pin A5 (Shares hole with LCD
The User Inputs (Buttons)
We use the Arduino's internal resistors, so wiring is simple.
Button 1 (Min Set): One leg to Arduino Pin 2. The other leg to GND.
Button 2 (Max Set): One leg to Arduino Pin 3. The other leg to GND.
The Motor Driver (L293D "Parallel Bridge" Setup)
This is the most complex part. Ensure the chip is oriented correctly (Notch at the top).
Phase A: Powering the Chip
Pin 16 (Top Right): -> Top Red Rail (5V Logic)
Pin 8 (Bottom Right): -> Bottom Red Rail (12V Motor Power)
Pins 4, 5, 12, 13 (Middle Pins): -> Blue Rail (GND).
Connect all four to GND for heat dissipation.
Phase B: The "Bridge" (Connecting Left side to Right side)
Add small jumper wires directly across the chip itself over the central divider of the breadboard.
Bridge Pin 1 to Pin 9 (Enable pins)
Bridge Pin 2 to Pin 10 (Input A pins)
Bridge Pin 7 to Pin 15 (Input B pins)
Phase C: Connecting Arduino to the Bridge
Now connect the Arduino signals to those bridged pairs.
Arduino Pin 11 (PWM) -> Connect to L293D Pin 1 (which is bridged to 9).
Arduino Pin 10 -> Connect to L293D Pin 2 (which is bridged to 10).
Arduino Pin 9 -> Connect to L293D Pin 7 (which is bridged to 15).
Phase D: Connecting the Motor to the Bridge
The motor outputs must also be bridged to handle the current.
Motor Wire Red (+): Connect to L293D Pin 3 AND Pin 11. (Use two wires from the breadboard holes meeting at the motor wire).
Motor Wire Black (-): Connect to L293D Pin 6 AND Pin 14.
