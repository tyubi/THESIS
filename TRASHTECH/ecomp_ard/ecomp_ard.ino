#include <Servo.h>

Servo myServo; // Create a Servo object to control the servo motor

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  myServo.attach(9);  // Attach the servo to pin 9
  myServo.write(90);  // Initialize servo at neutral position (90 degrees)
  Serial.println("Servo Control Ready! Waiting for commands...");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the incoming serial command

    if (command == 'A') { // BJT detected
      myServo.write(180);   // Bin 1: 0 degrees
      Serial.println("BJT detected! Servo moved to Bin 1 (0 degrees).");
    } else if (command == 'B') { // LED detected
      myServo.write(150);  // Bin 2: 30 degrees
      Serial.println("LED detected! Servo moved to Bin 2 (30 degrees).");
    } else if (command == 'C') { // Capacitor detected
      myServo.write(120);  // Bin 3: 60 degrees
      Serial.println("Capacitor detected! Servo moved to Bin 3 (60 degrees).");
    } else if (command == 'D') { // Defective component detected
      myServo.write(60);  // Bin 4: 90 degrees
      Serial.println("Defective component detected! Servo moved to Bin 4 (90 degrees).");
    } else if (command == 'E') { // Resistor detected
      myServo.write(90); // Bin 5: 120 degrees
      Serial.println("Resistor detected! Servo moved to Bin 5 (120 degrees).");
    } else if (command == 'F') { // Unknown component detected
      myServo.write(30); // Bin 6: 150 degrees
      Serial.println("Unknown component detected! Servo moved to Bin 6 (150 degrees).");
    } else {
      Serial.println("Invalid command received.");
    }
  }
}


// #include <Servo.h>

// Servo myServo; // Create a Servo object to control the servo motor

// void setup() {
//   myServo.attach(9); // Attach the servo to pin 9 (change if needed)
//   Serial.begin(9600); // Initialize serial communication for debugging
// }

// void loop() {
//   Serial.print(180);
//   int angles[] = {0, 45, 90, 135, 180}; // Array of angles to test
//   for (int i = 0; i < 5; i++) {
//     int angle = angles[i]; // Get the current angle
//     myServo.write(angle); // Rotate the servo to the specified angle
//     Serial.print("Servo rotated to: ");
//     Serial.print(angle);
//     Serial.println(" degrees");
//     delay(500); // Wait 1 second before moving to the next angle
//   }
// }


// #include <Servo.h>

// Servo myServo; // Create a Servo object to control the servo motor

// void setup() {
//   myServo.attach(9); // Attach the servo to pin 9 (change if needed)
//   Serial.begin(9600); // Initialize serial communication for debugging
//   Serial.println("Servo Angle Test Initialized");
// }

// void loop() {
//   // Array of angles for the servo to point at
//   int angles[] = {0, 45, 90, 135, 180}; 
//   for (int i = 0; i < 5; i++) {
//     int angle = angles[i]; // Get the current angle
//     myServo.write(angle);  // Rotate the servo to the specified angle
//     Serial.print("Servo pointing at: ");
//     Serial.print(angle);
//     Serial.println(" degrees");
//     delay(2000); // Wait 2 seconds to observe the servo's position
//   }
// }

// void setup() {
//   Serial.begin(9600); // Start serial communication at 9600 baud rate
//   while (!Serial); // Wait for serial port to connect. Needed for some boards
//   Serial.println("Servo Control Ready! Waiting for commands...");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     char command = Serial.read();  // Read the incoming byte
    
//     // Handle different commands
//     if (command == 'X') {
//       Serial.println("Received command: X");
//       // Add your logic for command X here (e.g., rotate servo to position X)
//     }
//     else if (command == 'Y') {
//       Serial.println("Received command: Y");
//       // Add your logic for command Y here (e.g., rotate servo to position Y)
//     }
//     else {
//       Serial.println("Invalid command received.");  // If the command is not recognized
//     }
//   }
// }





