#include <Servo.h>

// ===== PIN ASSIGNMENTS FOR ARDUINO MEGA =====
// TB6600 Stepper motor driver pins
const int stepPin = 24;    // STEP/PUL pin
const int dirPin = 22;     // DIR pin
const int enPin = 26;      // Enable pin

// Servo motor pins
const int servo1Pin = 11;
const int servo2Pin = 12;

// Gas sensor
const int gasPin = A0;
const int gasLED = 30;
const int gasThreshold = 870; // Adjusted to 200 ppm

// First ultrasonic sensor (fill level 1)
const int trigPin1 = 32;
const int echoPin1 = 33;
const int ledPin1 = 34;
const int distanceThreshold1 = 20; // cm

// Second ultrasonic sensor (fill level 2)
const int trigPin2 = 36;
const int echoPin2 = 37;
const int ledPin2 = 38;
const int distanceThreshold2 = 20; // cm

// ===== STEPPER MOTOR CONFIGURATION =====
// Steps per revolution for your motor
const int stepsPerRevolution = 200;
// Delay between pulses for the stepper (microseconds)
const int stepDelay = 1000;

// Initialize servo motors
Servo servo1;
Servo servo2;

// Variable to track current position of the stepper motor
int currentPosition = 0;

// Default servo positions
const int servo1Default = 0;
const int servo2Default = 180;

// Current positions for servos
int servo1Pos = servo1Default;
int servo2Pos = servo2Default;

// Variable to store the previous command
String previousCommand = "";

// Delay time for servo movement (in milliseconds)
const int servoDelay = 1000;

void setup() {
    Serial.begin(9600);
    
    // Setup TB6600 pins
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    
    // Enable the stepper motor driver (LOW = enabled)
    digitalWrite(enPin, LOW);
    
    // Attach servo motors
    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);
    
    // Initialize servos to default position
    servo1.write(servo1Default);
    servo2.write(servo2Default);
    
    // Initialize pins for gas and ultrasonic sensors
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
    pinMode(gasPin, INPUT);
    
    pinMode(gasLED, OUTPUT);
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    
    // Turn off all LEDs initially
    digitalWrite(gasLED, LOW);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    
    Serial.println("Automated Waste Segregation System");
    Serial.println("Enter command: BIO, NONBIO, or RECY");
}

// Function to rotate stepper motor
void stepperMove(int steps) {
    // Set direction based on the sign of steps
    digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
    
    // Convert steps to absolute value
    steps = abs(steps);
    
    // Move the required number of steps
    for(int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
}

void loop() {
    // === PART 1: MONITOR SENSORS ===
    // Read gas sensor
    int gasValue = analogRead(gasPin);
    if (gasValue > gasThreshold) {
        digitalWrite(gasLED, HIGH);
        Serial.println("WARNING: Toxic gas detected!");
    } else {
        digitalWrite(gasLED, LOW);
    }
    
    // Read first ultrasonic sensor (container 1)
    long duration1 = readUltrasonicSensor(trigPin1, echoPin1);
    float distance1 = duration1 * 0.034 / 2;
    
    if (distance1 < distanceThreshold1) {
        digitalWrite(ledPin1, HIGH);
        Serial.println("WARNING: Container 1 is full!");
    } else {
        digitalWrite(ledPin1, LOW);
    }
    
    // Read second ultrasonic sensor (container 2)
    long duration2 = readUltrasonicSensor(trigPin2, echoPin2);
    float distance2 = duration2 * 0.034 / 2;
    
    if (distance2 < distanceThreshold2) {
        digitalWrite(ledPin2, HIGH);
        Serial.println("WARNING: Container 2 is full!");
    } else {
        digitalWrite(ledPin2, LOW);
    }
    
    // === PART 2: PROCESS COMMANDS FOR WASTE SEGREGATION ===
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        
        // Check if the command is the same as the previous one
        if (command == previousCommand) {
            Serial.println("Same command detected. Skipping stepper rotation.");
            
            // Still operate the servo motors
            if (command == "BIO" || command == "NONBIO" || command == "RECY") {
                // Rotate servos to their action positions
                servo1.write(180);
                servo2.write(0);
                servo1Pos = 180;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 180 degrees and Servo2 to 0 degrees");
                
                // Wait for a moment
                delay(servoDelay);
                
                // Return servos to default positions
                servo1.write(servo1Default);
                servo2.write(servo2Default);
                servo1Pos = servo1Default;
                servo2Pos = servo2Default;
                Serial.println("Returning servos to default positions");
            }
        }
        else {
            // First, return to home position (zero) if not already there
            if (currentPosition != 0) {
                Serial.print("Resetting stepper position from ");
                Serial.print(currentPosition);
                Serial.println(" steps to 0...");
                
                // Move back to zero position using TB6600 driver
                stepperMove(-currentPosition);
                currentPosition = 0;
            }
            
            // Execute the stepper motor command
            if (command == "BIO") {
                // For BIO, you had 0 steps in original code, using same here
                stepperMove(0);
                currentPosition = 0;
                Serial.println("Moving stepper 0 steps (BIO)");
                
                // Now move servo motors
                servo1.write(180);
                servo2.write(0);
                servo1Pos = 180;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 180 degrees and Servo2 to 0 degrees");
                
                // Wait for a moment
                delay(servoDelay);
                
                // Return servos to default positions
                servo1.write(servo1Default);
                servo2.write(servo2Default);
                servo1Pos = servo1Default;
                servo2Pos = servo2Default;
                Serial.println("Returning servos to default positions");
            }
            else if (command == "NONBIO") {
                // Move stepper motor using TB6600
                stepperMove(4266);
                currentPosition = 4266;
                Serial.println("Moving stepper forward 4266 steps (NONBIO)");
                
                // Now move servo motors
                servo1.write(180);
                servo2.write(0);
                servo1Pos = 180;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 180 degrees and Servo2 to 0 degrees");
                
                // Wait for a moment
                delay(servoDelay);
                
                // Return servos to default positions
                servo1.write(servo1Default);
                servo2.write(servo2Default);
                servo1Pos = servo1Default;
                servo2Pos = servo2Default;
                Serial.println("Returning servos to default positions");
            }
            else if (command == "RECY") {
                // Move stepper motor using TB6600
                stepperMove(-4266);
                currentPosition = -4266;
                Serial.println("Moving stepper backward 4266 steps (RECY)");
                
                // Now move servo motors
                servo1.write(180);
                servo2.write(0);
                servo1Pos = 180;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 180 degrees and Servo2 to 0 degrees");
                
                // Wait for a moment
                delay(servoDelay);
                
                // Return servos to default positions
                servo1.write(servo1Default);
                servo2.write(servo2Default);
                servo1Pos = servo1Default;
                servo2Pos = servo2Default;
                Serial.println("Returning servos to default positions");
            }
            else {
                Serial.println("Unknown command. Please enter BIO, NONBIO, or RECY.");
                // Don't update previousCommand for unknown commands
                return;
            }
            
            // Update the previous command
            previousCommand = command;
        }
        
        Serial.print("Current stepper position: ");
        Serial.println(currentPosition);
        Serial.print("Current servo positions: Servo1 = ");
        Serial.print(servo1Pos);
        Serial.print(", Servo2 = ");
        Serial.println(servo2Pos);
    }
    
    // Small delay to prevent CPU hogging
    delay(100);
}

// Function to read ultrasonic sensor
long readUltrasonicSensor(int trigPin, int echoPin) {
    // Clear the trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    
    // Set the trigger pin HIGH for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    // Read the echo pin, returns the sound wave travel time in microseconds
    return pulseIn(echoPin, HIGH);
}