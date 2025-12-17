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

// --- HARDWARE PIN DEFINITIONS ---
// L293D Connected via "Parallel Bridge" wiring
#define MOTOR_ENABLE 11  // PWM Speed Control (Shared for both sides)
#define MOTOR_IN1    10  // Direction A (Shared)
#define MOTOR_IN2    9   // Direction B (Shared)

#define BTN_MIN_SET  2   // Button to Capture START Threshold
#define BTN_MAX_SET  3   // Button to Capture STOP Threshold

// --- OBJECTS ---
MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- SETTINGS ---
const unsigned long RAMP_DURATION = 300000; // 5 Minutes in milliseconds
const int START_SPEED = 130; // PWM to start (approx 50% to ensure spin)

// --- VARIABLES ---
// Oxygen Calculation Variables
double avered = 0; double aveir = 0;
double sumirrms = 0; double sumredrms = 0;
int i = 0;
int Num = 100;
int oxygen = 80; // Start conservative until sensor settles
long lastBeat = 0; 

// Thresholds (Defaults as requested)
int minO2 = 80; // Motor STARTS when O2 drops to this
int maxO2 = 99; // Motor STOPS when O2 rises to this

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
    stopMotor(); // Ensure OFF at startup

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

    // Configure Sensor (Low Red LED for preventing saturation)
    particleSensor.setup(); 
    particleSensor.setPulseAmplitudeRed(0x0A); 
    particleSensor.setPulseAmplitudeGreen(0); 
    
    lcd.clear();
    lcd.print("Calibrating...");
    delay(1000);
}

void loop() {
    // --- 1. SENSOR READING ---
    uint32_t irValue = particleSensor.getIR();
    uint32_t redValue = particleSensor.getRed();

    // Safety: If finger removed, stop everything
    if (irValue < 7000) {
        if (motorRunning) stopMotor();
        
        lcd.setCursor(0,0);
        lcd.print("Place Finger... ");
        lcd.setCursor(0,1);
        lcd.print("Motor STOPPED   ");
        
        // Reset math
        avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0; i = 0;
        return; 
    }

    // --- 2. OXYGEN CALCULATION (Continuous) ---
    // Standard AC/DC ratio algorithm for continuous reading
    double fred = (double)redValue;
    double fir = (double)irValue;
    
    avered = avered * 0.95 + fred * 0.05;
    aveir = aveir * 0.95 + fir * 0.05;
    
    double fredAC = fred - avered;
    double firAC = fir - aveir;
    
    sumredrms += (fredAC * fredAC);
    sumirrms += (firAC * firAC);
    i++;

    // Update O2 value every 50 samples
    if ((i % 50) == 0) {
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
        int currentO2 = -45.060 * R * R + 30.354 * R + 94.845;
        
        // Constrain and Smooth
        if (currentO2 > 100) currentO2 = 100;
        if (currentO2 < 70) currentO2 = 70; // Floor 
        oxygen = (oxygen + currentO2) / 2;  // Rolling average

        sumredrms = 0.0; sumirrms = 0.0; i = 0;
    }

    // --- 3. BUTTONS (CAPTURE LOGIC) ---
    
    // Button 1: CAPTURE Current O2 as MIN (Start Threshold)
    if (digitalRead(BTN_MIN_SET) == LOW) {
        minO2 = oxygen;
        // Safety: Min cannot be higher than Max
        if (minO2 >= maxO2) minO2 = maxO2 - 1; 
        
        lcd.clear(); 
        lcd.print("Saved MIN Limit:");
        lcd.setCursor(0,1); lcd.print(minO2); lcd.print("%");
        delay(1000); // Show confirmation
    }

    // Button 2: CAPTURE Current O2 as MAX (Stop Threshold)
    if (digitalRead(BTN_MAX_SET) == LOW) {
        maxO2 = oxygen;
        // Safety: Max cannot be lower than Min
        if (maxO2 <= minO2) maxO2 = minO2 + 1;

        lcd.clear(); 
        lcd.print("Saved MAX Limit:");
        lcd.setCursor(0,1); lcd.print(maxO2); lcd.print("%");
        delay(1000); // Show confirmation
    }

    // --- 4. MOTOR CONTROL CYCLE ---

    // STOP Condition: O2 has reached or exceeded the Target (Max)
    if (motorRunning && oxygen >= maxO2) {
        stopMotor();
    }

    // START Condition: O2 has dropped to (or below) the Trigger (Min)
    if (!motorRunning && oxygen <= minO2) {
        startMotor();
    }

    // RAMP Logic: If active, increase speed over time
    if (motorRunning) {
        unsigned long elapsed = millis() - rampStartTime;
        if (elapsed < RAMP_DURATION) {
            // Scale time (0-5mins) to PWM (StartSpeed-255)
            currentSpeed = map(elapsed, 0, RAMP_DURATION, START_SPEED, 255);
        } else {
            currentSpeed = 255; // Full speed cap
        }
        analogWrite(MOTOR_ENABLE, currentSpeed);
    }

    // --- 5. DISPLAY UPDATE (Every 500ms) ---
    static unsigned long lastDisp = 0;
    if (millis() - lastDisp > 500) {
        lcd.setCursor(0,0);
        lcd.print("O2:"); lcd.print(oxygen); lcd.print("% ");
        
        // Show Motor Status
        if (motorRunning) {
            lcd.print("M:"); 
            int p = map(currentSpeed, 0, 255, 0, 100);
            lcd.print(p); lcd.print("%  ");
        } else {
            lcd.print("M:OFF  ");
        }

        // Show Thresholds
        lcd.setCursor(0,1);
        lcd.print("L:"); lcd.print(minO2); // L for Low/Min
        lcd.print(" H:"); lcd.print(maxO2);// H for High/Max
        lcd.print("     ");
        
        lastDisp = millis();
    }
}

// --- HELPER FUNCTIONS ---
void startMotor() {
    motorRunning = true;
    rampStartTime = millis(); // Reset ramp timer
    
    // Set Direction (Left bridged to Right)
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    
    // Initial Kick
    analogWrite(MOTOR_ENABLE, START_SPEED);
}

void stopMotor() {
    motorRunning = false;
    currentSpeed = 0;
    
    // Cut Power
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENABLE, 0);
}
