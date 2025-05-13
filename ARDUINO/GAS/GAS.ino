const int mq135Pin = A0; // AOUT pin connected to analog A0

void setup() {
  Serial.begin(9600);
  pinMode(mq135Pin, INPUT);
  Serial.println("MQ-135 CO₂ & NH₃ Estimation Started...");
}

void loop() {
  int sensorValue = analogRead(mq135Pin); // Raw ADC value

  // Estimate CO2 in ppm using a basic approximation
  float co2_ppm = map(sensorValue, 200, 800, 400, 5000); // Rough scale: 400ppm (clean) to 5000ppm (dirty)
  co2_ppm = constrain(co2_ppm, 400, 5000);

  // Estimate NH3 (Ammonia) PPM using similar mapping
  float nh3_ppm = map(sensorValue, 200, 800, 0, 300);
  nh3_ppm = constrain(nh3_ppm, 0, 300);

  Serial.print("Estimated CO2: ");
  Serial.print(co2_ppm);
  Serial.print(" ppm");

  Serial.print(" | Estimated NH3: ");
  Serial.print(nh3_ppm);
  Serial.println(" ppm");

  delay(1000); // Update every second
}
