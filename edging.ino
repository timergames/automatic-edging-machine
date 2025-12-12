#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "heartRate.h"

// Licence - PRIVATE USE IS FREE. COMMERCIAL USE IS PROHIBITED.
// 
// Yes - Print and use the 3D prints of the 3D model for private use; 
// Yes - Share the images of your 3D prints of the 3D model on communication media such as social networks or websites.
// 
// No - commercial use or public sharing of the 3D model; 
// No - modification or adaptation of the 3D model for public sharing or sale; 
// No - distribution, sale, donation or exchange of the digital files of the 3D model.
// 
// (C) Copyright 2025 Badsub Designs. All rights reserved.

// --- HARDWARE ---
#define MOTOR_ENABLE 11
#define MOTOR_IN1    10
#define MOTOR_IN2    9
#define BTN_MIN_SET  2
#define BTN_MAX_SET  3

// --- OBJECTS ---
MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- SETTINGS ---
// Motor Ramp Settings
const unsigned long RAMP_DURATION = 300000; // 5 Minutes
const int START_SPEED = 120; // High enough to ensure spin (approx 45%)

// Threshold Defaults
int minO2 = 90;
int maxO2 = 95;

// --- VARIABLES ---
double avered = 0; 
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100; // Sampling depth for SpO2 calculation
int oxygen = 98; // Default starting value
long lastBeat = 0; 
int beatAvg = 0;

// Motor State
bool motorRunning = false;
unsigned long rampStartTime = 0;
int currentSpeed = 0;

void setup() {
    Serial.begin(115200);

    // 1. Motor Setup
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    stopMotor();

    // 2. Button Setup
    pinMode(BTN_MIN_SET, INPUT_PULLUP);
    pinMode(BTN_MAX_SET, INPUT_PULLUP);

    // 3. LCD Setup
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0,0);
    lcd.print("System Start...");

    // 4. Sensor Setup
    if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
        lcd.clear(); lcd.print("Sensor Fail!");
        while (1);
    }

    // Setup for Red + IR reading
    particleSensor.setup(); 
    particleSensor.setPulseAmplitudeRed(0x0A); // Low Red to avoid saturation
    particleSensor.setPulseAmplitudeGreen(0); 
    
    lcd.clear();
    lcd.print("Place Finger...");
}

void loop() {
    // --- 1. READ SENSOR DATA ---
    uint32_t irValue = particleSensor.getIR();
    uint32_t redValue = particleSensor.getRed();

    // Restart/Clear if no finger
    if (irValue < 7000) {
        motorRunning = false; // Optional safety: Stop motor if no finger
        stopMotor();
        
        lcd.setCursor(0,0);
        lcd.print("Place Finger... ");
        lcd.setCursor(0,1);
        lcd.print("                ");
        
        // Reset averages
        avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0; i = 0;
        return; // Skip the rest of the loop
    }

    // --- 2. SIMPLE SpO2 MATH (Running Average) ---
    // This runs continuously without "Wait 5 seconds"
    
    // Check for Beat (Heart Rate)
    if (checkForBeat(irValue) == true) {
        long delta = millis() - lastBeat;
        lastBeat = millis();
        float bpm = 60 / (delta / 1000.0);
        if (bpm > 40 && bpm < 150) beatAvg = (beatAvg + bpm) / 2;
    }

    // Calculate SpO2 "AC/DC" Ratio
    double fred = (double)redValue;
    double fir = (double)irValue;
    
    // Remove DC component (Average)
    avered = avered * 0.95 + fred * 0.05;
    aveir = aveir * 0.95 + fir * 0.05;
    
    double fredAC = fred - avered;
    double firAC = fir - aveir;
    
    // Accumulate RMS
    sumredrms += (fredAC * fredAC);
    sumirrms += (firAC * firAC);
    i++;

    // Every 50 samples, update the SpO2 Number
    if ((i % 50) == 0) {
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
        
        // Empirical Formula for SpO2 based on Ratio R
        int currentO2 = -45.060 * R * R + 30.354 * R + 94.845;
        
        // Constrain to realistic human values
        if (currentO2 > 100) currentO2 = 100;
        if (currentO2 < 80) currentO2 = 80; // Floor it to avoid scary numbers
        
        // Smooth the change
        oxygen = (oxygen + currentO2) / 2;

        // Reset counters
        sumredrms = 0.0; sumirrms = 0.0; i = 0;
    }

    // --- 3. MOTOR LOGIC ---
    
    // Start if Oxygen is LOW
    if (!motorRunning && oxygen < minO2) {
        startMotor();
    }
    
    // Stop if Oxygen is HIGH
    if (motorRunning && oxygen > maxO2) {
        stopMotor();
    }

    // Ramp Speed
    if (motorRunning) {
        unsigned long elapsed = millis() - rampStartTime;
        if (elapsed < RAMP_DURATION) {
            currentSpeed = map(elapsed, 0, RAMP_DURATION, START_SPEED, 255);
        } else {
            currentSpeed = 255;
        }
        analogWrite(MOTOR_ENABLE, currentSpeed);
    }

    // --- 4. BUTTONS ---
    // Adjust Min
    if (digitalRead(BTN_MIN_SET) == LOW) {
        minO2--;
        if (minO2 < 80) minO2 = 98;
        lcd.clear(); lcd.print("Min O2: "); lcd.print(minO2);
        delay(200);
    }
    // Adjust Max
    if (digitalRead(BTN_MAX_SET) == LOW) {
        maxO2++;
        if (maxO2 > 100) maxO2 = 90;
        lcd.clear(); lcd.print("Max O2: "); lcd.print(maxO2);
        delay(200);
    }

    // --- 5. DISPLAY (Every 500ms) ---
    static unsigned long lastDisp = 0;
    if (millis() - lastDisp > 500) {
        lcd.setCursor(0,0);
        lcd.print("O2:"); lcd.print(oxygen); lcd.print("% ");
        
        lcd.print("M:"); 
        if (motorRunning) {
            int p = map(currentSpeed, 0, 255, 0, 100);
            lcd.print(p); lcd.print("%  ");
        } else {
            lcd.print("OFF  ");
        }

        lcd.setCursor(0,1);
        lcd.print("Min:"); lcd.print(minO2);
        lcd.print(" Max:"); lcd.print(maxO2);
        
        lastDisp = millis();
    }
}

// --- HELPER FUNCTIONS ---
void startMotor() {
    motorRunning = true;
    rampStartTime = millis();
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENABLE, START_SPEED); // Start with a kick
}

void stopMotor() {
    motorRunning = false;
    currentSpeed = 0;
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENABLE, 0);
}
