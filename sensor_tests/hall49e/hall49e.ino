// Defining input and output pins
const int inputPin = A0;  // Analog input pin

void setup() {
  // Initializes serial communication
  Serial.begin(9600); // Sets baud rate to 9600 bps
}

void loop() {
  // Small delay to ensure voltage is applied
  delay(100);
  
  // Reads the voltage on the analog input pin
  int readVoltage = analogRead(inputPin);
  
  // Converts the read value to the corresponding voltage in volts
  float voltageVolts = readVoltage * (5.0 / 1023.0); // 5V is the Arduino reference voltage
  
  // Prints the read value to the serial monitor
  Serial.print("Read voltage: ");
  Serial.print(readVoltage);
  Serial.print(" | Voltage in volts: ");
  Serial.println(voltageVolts);
  
  // Small delay before the next reading
  delay(100);
}
