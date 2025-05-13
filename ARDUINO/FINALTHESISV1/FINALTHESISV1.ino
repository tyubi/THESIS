#include <Servo.h>

// ===== PIN ASSIGNMENTS FOR ARDUINO MEGA =====
// TB6600 Stepper motor driver pins
const int stepPin = 24; //PUL
const int dirPin = 22;  //DIR
const int enPin = 26;   //EN

// Servo motor pins
const int servo1Pin = 8;
const int servo2Pin = 9;

// Gas sensor
const int gasPin = A0;
const int gasLED = 30;

// Gas sensor mapping values
const int gasMinRaw = 200;
const int gasMaxRaw = 800;

// CO2 thresholds (in ppm)
const int co2NormalMax = 1000;     // Below this is normal
const int co2AboveNormalMax = 2000; // Below this is above normal, above this is toxic

// NH3 thresholds (in ppm)
const int nh3NormalMax = 20;       // Below this is normal
const int nh3AboveNormalMax = 50;  // Below this is above normal, above this is toxic

// First ultrasonic sensor
const int trigPin1 = 6;
const int echoPin1 = 7;
const int ledPin1 = 34;
const int distanceThreshold1 = 20; // cm

// Second ultrasonic sensor
const int trigPin2 = 4;
const int echoPin2 = 5;
const int ledPin2 = 41;
const int distanceThreshold2 = 20; // cm

// Container fill level calibration (distance in cm)
const float fillLevels[] = {
    6.0,   // 100%
    10.5,  // 90%
    14.5,  // 80%
    19.0,  // 70%
    23.5,  // 60%
    28.0,  // 50%
    32.5,  // 40%
    37.0,  // 30%
    41.5,  // 20%
    46.0   // 10%
};

// ===== STEPPER MOTOR CONFIGURATION =====
const int stepsPerRevolution = 1600;
const int stepDelay = 2500; // microseconds - increased from 500 to 2000 for slower rotation

Servo servo1;
Servo servo2;

int currentPosition = 0;

const int servo1Default = 0;
const int servo2Default = 130;

int servo1Pos = servo1Default;
int servo2Pos = servo2Default;

String previousCommand = "";

const int servoDelay = 500;

// Updated to 3 minutes
unsigned long previousSensorReadTime = 0;
const unsigned long sensorReadInterval = 180000; // 3 minutes in milliseconds

// Function to calculate fill percentage based on distance
int calculateFillPercentage(float distance) {
    // If distance is less than or equal to minimum (6cm), return 100%
    if (distance <= fillLevels[0]) return 100;
    
    // If distance is greater than or equal to maximum (46cm), return 0%
    if (distance >= fillLevels[9]) return 0;
    
    // Find the appropriate range and interpolate
    for (int i = 0; i < 9; i++) {
        if (distance >= fillLevels[i] && distance < fillLevels[i + 1]) {
            // Linear interpolation between two points
            float range = fillLevels[i + 1] - fillLevels[i];
            float position = distance - fillLevels[i];
            float percentage = 100 - (i * 10) - (position / range * 10);
            return (int)percentage;
        }
    }
    
    return 0; // Default return
}

void setup() {
    Serial.begin(9600);

    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    digitalWrite(enPin, LOW);

    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);
    servo1.write(servo1Default);
    servo2.write(servo2Default);

    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
    pinMode(gasPin, INPUT);

    pinMode(gasLED, OUTPUT);
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);

    digitalWrite(gasLED, LOW);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);

    Serial.println("Automated Waste Segregation System");
    Serial.println("Enter command: BIO, NONBIO, or RECY");
    Serial.println("Sensor readings every 3 minutes");

    readSensors();
    previousSensorReadTime = millis();
}

void stepperMove(int steps) {
    digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
    steps = abs(steps);
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
}

// Helper function to read one ultrasonic sensor
long readUltrasonicSensor(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30 ms
    return duration;
}

void readSensors() {
    Serial.println("\n--- Reading Sensors ---");

    // GAS SENSOR READINGS
    int gasValue = analogRead(gasPin);
    
    // Calculate CO2 and NH3 estimates
    float co2_ppm = map(gasValue, gasMinRaw, gasMaxRaw, 400, 5000);
    co2_ppm = constrain(co2_ppm, 400, 5000);
    
    float nh3_ppm = map(gasValue, gasMinRaw, gasMaxRaw, 0, 300);
    nh3_ppm = constrain(nh3_ppm, 0, 300);
    
    // Determine CO2 level category
    String co2Level;
    if (co2_ppm <= co2NormalMax) {
        co2Level = "NORMAL";
    } else if (co2_ppm <= co2AboveNormalMax) {
        co2Level = "ABOVE NORMAL";
    } else {
        co2Level = "TOXIC";
    }
    
    // Determine NH3 level category
    String nh3Level;
    if (nh3_ppm <= nh3NormalMax) {
        nh3Level = "NORMAL";
    } else if (nh3_ppm <= nh3AboveNormalMax) {
        nh3Level = "ABOVE NORMAL";
    } else {
        nh3Level = "TOXIC";
    }
    
    // Display gas levels
    Serial.print("CO2 Level: ");
    Serial.print(co2Level);
    Serial.print(" | NH3 Level: ");
    Serial.println(nh3Level);
    Serial.println();

    // ULTRASONIC SENSOR 1
    long duration1 = readUltrasonicSensor(trigPin1, echoPin1);
    float distance1 = duration1 * 0.034 / 2;
    int fillPercentage1 = calculateFillPercentage(distance1);

    if (fillPercentage1 >= 90) {
        digitalWrite(ledPin1, HIGH);
        Serial.print("Container 1: ");
        Serial.print(fillPercentage1);
        Serial.println("% filled (WARNING)");
    } else {
        digitalWrite(ledPin1, LOW);
        Serial.print("Container 1: ");
        Serial.print(fillPercentage1);
        Serial.println("% filled");
    }

    delay(50); // prevent crosstalk

    // ULTRASONIC SENSOR 2
    long duration2 = readUltrasonicSensor(trigPin2, echoPin2);
    float distance2 = duration2 * 0.034 / 2;
    int fillPercentage2 = calculateFillPercentage(distance2);

    if (fillPercentage2 >= 90) {
        digitalWrite(ledPin2, HIGH);
        Serial.print("Container 2: ");
        Serial.print(fillPercentage2);
        Serial.println("% filled (WARNING)");
    } else {
        digitalWrite(ledPin2, LOW);
        Serial.print("Container 2: ");
        Serial.print(fillPercentage2);
        Serial.println("% filled");
    }
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousSensorReadTime >= sensorReadInterval || currentMillis < previousSensorReadTime) {
        readSensors();
        previousSensorReadTime = currentMillis;
    }

    // Add your command reading/processing code here
    // Process commands for waste segregation
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        
        // Check if the command is the same as the previous one
        if (command == previousCommand) {
            Serial.println("Same command detected. Skipping stepper rotation.");
            
            // Still operate the servo motors
            if (command == "BIO" || command == "NONBIO" || command == "RECY") {
                // Rotate servos to their action positions
                servo1.write(210);
                servo2.write(0);
                servo1Pos = 210;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 210 degrees and Servo2 to 0 degrees");
                
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
                servo1.write(210);
                servo2.write(0);
                servo1Pos = 210;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 210 degrees and Servo2 to 0 degrees");
                
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
                stepperMove(1066);
                currentPosition = 1066;
                Serial.println("Moving stepper forward 1066 steps (NONBIO)");
                
                // Now move servo motors
                servo1.write(210);
                servo2.write(0);
                servo1Pos = 210;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 210 degrees and Servo2 to 0 degrees");
                
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
                stepperMove(-1066);
                currentPosition = -1066;
                Serial.println("Moving stepper backward 1066 steps (RECY)");
                
                // Now move servo motors
                servo1.write(210);
                servo2.write(0);
                servo1Pos = 210;
                servo2Pos = 0;
                Serial.println("Rotating Servo1 to 210 degrees and Servo2 to 0 degrees");
                
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

    
