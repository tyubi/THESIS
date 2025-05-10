// #include <Servo.h>

// // Pin Definitions for Servo
// Servo myServo;         // Create a Servo object to control the servo motor
// const int servoPin = 9; // Servo connected to pin 9

// // Pin Definitions for DC Motor
// const int motorPin1 = 3; // Motor driver IN1
// const int motorPin2 = 4; // Motor driver IN2

// void setup() {
//   // Initialize serial communication
//   Serial.begin(9600);

//   // Initialize Servo
//   myServo.attach(servoPin);   // Attach the servo to pin 9
//   myServo.write(90);          // Initialize servo at neutral position (90 degrees)
//   Serial.println("Servo and DC Motor Control Ready!");

//   // Initialize DC Motor Pins
//   pinMode(motorPin1, OUTPUT);
//   pinMode(motorPin2, OUTPUT);
// }

// void loop() {
//   // Check for incoming serial commands for Servo control
//   if (Serial.available() > 0) {
//     char command = Serial.read(); // Read the incoming serial command

//     int angle = -1; // Default value indicating invalid command

//     // Process commands to move Servo to specific bins
//     if (command == 'A') { // BJT detected
//       angle = map(1, 1, 10, 0, 180);  // Map to Bin 1 (0–18 degrees)
//       Serial.println("BJT detected! Moving to Bin 1.");
//     } else if (command == 'B') { // LED detected
//       angle = map(2, 1, 10, 0, 180);  // Map to Bin 2 (18–36 degrees)
//       Serial.println("LED detected! Moving to Bin 2.");
//     } else if (command == 'C') { // Capacitor detected
//       angle = map(3, 1, 10, 0, 180);  // Map to Bin 3 (36–54 degrees)
//       Serial.println("Capacitor detected! Moving to Bin 3.");
//     } else if (command == 'D') { // Defective component detected
//       angle = map(4, 1, 10, 0, 180);  // Map to Bin 4 (54–72 degrees)
//       Serial.println("Defective component detected! Moving to Bin 4.");
//     } else if (command == 'E') { // Resistor detected
//       angle = map(5, 1, 10, 0, 180);  // Map to Bin 5 (72–90 degrees)
//       Serial.println("Resistor detected! Moving to Bin 5.");
//     } else if (command == 'F') { // Unknown component detected
//       angle = map(6, 1, 10, 0, 180);  // Map to Bin 6 (90–108 degrees)
//       Serial.println("Unknown component detected! Moving to Bin 6.");
//     } else if (command == 'G') { // Bin 7
//       angle = map(7, 1, 10, 0, 180);  // Map to Bin 7 (108–126 degrees)
//       Serial.println("Bin 7 detected! Moving to Bin 7.");
//     } else if (command == 'H') { // Bin 8
//       angle = map(8, 1, 10, 0, 180);  // Map to Bin 8 (126–144 degrees)
//       Serial.println("Bin 8 detected! Moving to Bin 8.");
//     } else if (command == 'I') { // Bin 9
//       angle = map(9, 1, 10, 0, 180);  // Map to Bin 9 (144–162 degrees)
//       Serial.println("Bin 9 detected! Moving to Bin 9.");
//     } else if (command == 'J') { // Bin 10
//       angle = map(10, 1, 10, 0, 180);  // Map to Bin 10 (162–180 degrees)
//       Serial.println("Bin 10 detected! Moving to Bin 10.");
//     } else {
//       Serial.println("Invalid command received.");
//     }

//     // If valid angle, move Servo to the calculated position
//     if (angle != -1) {
//       myServo.write(angle);  // Move servo to the corresponding angle
//     }

//     // Clear the serial buffer to avoid command overwriting
//     while (Serial.available() > 0) {
//       Serial.read();
//     }

//     // Send confirmation back
//     Serial.println("DONE");
//   }

//   // Control DC Motor
//   // Step 1: Run motor for 500ms
//   digitalWrite(motorPin1, HIGH);
//   digitalWrite(motorPin2, LOW);
//   delay(500);

//   // Step 2: Stop motor for 750ms
//   digitalWrite(motorPin1, LOW);
//   digitalWrite(motorPin2, LOW);
//   delay(850);

//   // Step 3: Run motor for 1 second
//   digitalWrite(motorPin1, HIGH);
//   digitalWrite(motorPin2, LOW);
//   delay(1000);
// }

#include <Servo.h>

// Pin Definitions for Servo
Servo myServo;         // Create a Servo object to control the servo motor
const int servoPin = 9; // Servo connected to pin 9

// Pin Definitions for DC Motor
const int motorPin1 = 3; // Motor driver IN1
const int motorPin2 = 4; // Motor driver IN2

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize Servo
  myServo.attach(servoPin);   // Attach the servo to pin 9
  myServo.write(70);          // Initialize servo at neutral position (90 degrees)
  Serial.println("Servo and DC Motor Control Ready!");

  // Initialize DC Motor Pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the incoming serial command

    if (command == 'A') { // BJT detected
      myServo.write(180);   // Bin 1: 180 degrees
      Serial.println("BJT detected! Servo moved to Bin 1 (0 degrees).");
    } else if (command == 'B') { // LED detected
      myServo.write(143);  // Bin 2: 140 degrees
      Serial.println("LED detected! Servo moved to Bin 2 (30 degrees).");
    } else if (command == 'C') { // Capacitor detected
      myServo.write(105);  // Bin 3: 105 degrees
      Serial.println("Capacitor detected! Servo moved to Bin 3 (60 degrees).");
    } else if (command == 'D') { // Defective component detected
      myServo.write(35);  // Bin 4: 350 degrees
      Serial.println("Defective component detected! Servo moved to Bin 4 (90 degrees).");
    } else if (command == 'E') { // Resistor detected
      myServo.write(70); // Bin 5: 70 degrees
      Serial.println("Resistor detected! Servo moved to Bin 5 (120 degrees).");
    } else if (command == 'F') { // Unknown component detected
      myServo.write(0); // Bin 6: 0 degrees
      Serial.println("Unknown component detected! Servo moved to Bin 6 (150 degrees).");
    } else {
      Serial.println("Invalid command received.");
    }

    // Clear the serial buffer to avoid command overwriting
    while (Serial.available() > 0) {
      Serial.read();
    }

    // Send confirmation back
    Serial.println("DONE");
  }

    // Control DC Motor
  // Step 1: Run motor for 500ms
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  delay(500);

  // Step 2: Stop motor for 750ms
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  delay(850);

  // Step 3: Run motor for 1 second
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  delay(1000);
}
