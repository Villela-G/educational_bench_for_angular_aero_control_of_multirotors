// Including libraries
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; //I2C

void setup() {
  // Starting serial communication
  Serial.begin(9600);
  // Printing a test message to the Serial Monitor
  Serial.println(F("BMP280 test"));
  
  if (!bmp.begin(0x76)) { /*Set I2C address to 0x76. Change to (0x77) if needed.*/
    
    // Print an error message if the address is invalid or sensor is not found
    Serial.println(F(" Could not find a valid BMP280 sensor, check wiring or "
                      "try another address!"));
    while (1) delay(10);
  }
}

void loop() {
  
    // Printing temperature values
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    // Printing pressure values
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    // Wait time: 1 second
    Serial.println();
    delay(1000);
}