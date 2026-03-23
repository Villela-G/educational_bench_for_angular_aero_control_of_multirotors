/*
	Potentiometer angle reader for a tilting bench.

	Calibration concept:
	1) Place the bench at -45 degrees and record its voltage as voltageAtMinus45.
	2) Place the bench at +45 degrees and record its voltage as voltageAtPlus45.
	3) The code linearly maps any measured voltage to the angle range [-45, +45].

	You can set fixed calibration values below, or use Serial commands:
	- Send 'm' when the bench is at -45 degrees to capture voltageAtMinus45.
	- Send 'p' when the bench is at +45 degrees to capture voltageAtPlus45.
*/

const int analogPin = A0;

// Set this to your board reference voltage (typically 5.0V or 3.3V).
const float vRef = 5.0;
const int adcMax = 1023;

// Initial calibration values (example values).
float voltageAtMinus45 = 1.20;
float voltageAtPlus45  = 3.80;

unsigned long lastPrintMs = 0;
const unsigned long printIntervalMs = 200;

float readVoltage() {
	int raw = analogRead(analogPin);
	return (raw * vRef) / adcMax;
}

float mapVoltageToAngle(float voltage) {
	// Protect against invalid calibration.
	if (voltageAtPlus45 <= voltageAtMinus45) {
		return 0.0;
	}

	const float inSpan = voltageAtPlus45 - voltageAtMinus45;
	const float outSpan = 90.0; // from -45 to +45

	float angle = ((voltage - voltageAtMinus45) * outSpan / inSpan) - 45.0;

	// Clamp to expected mechanical range.
	if (angle < -45.0) angle = -45.0;
	if (angle > 45.0) angle = 45.0;

	return angle;
}

void printHelp() {
	Serial.println("Potentiometer bench-angle reader");
	Serial.println("Calibration notes:");
	Serial.println("- Adjust the two calibration voltages to your setup.");
	Serial.println("- Example: 1.18V = -45 deg, 3.86V = +45 deg.");
	Serial.println("Serial commands:");
	Serial.println("  m -> capture current voltage as -45 deg point");
	Serial.println("  p -> capture current voltage as +45 deg point");
	Serial.println("  h -> print this help again");
	Serial.println();
}

void setup() {
	Serial.begin(9600);
	pinMode(analogPin, INPUT);

	delay(300);
	printHelp();
}

void loop() {
	if (Serial.available() > 0) {
		char cmd = (char)Serial.read();

		if (cmd == 'm' || cmd == 'M') {
			voltageAtMinus45 = readVoltage();
			Serial.print("Captured -45 deg voltage: ");
			Serial.print(voltageAtMinus45, 3);
			Serial.println(" V");
		} else if (cmd == 'p' || cmd == 'P') {
			voltageAtPlus45 = readVoltage();
			Serial.print("Captured +45 deg voltage: ");
			Serial.print(voltageAtPlus45, 3);
			Serial.println(" V");
		} else if (cmd == 'h' || cmd == 'H') {
			printHelp();
		}
	}

	unsigned long now = millis();
	if (now - lastPrintMs >= printIntervalMs) {
		lastPrintMs = now;

		int raw = analogRead(analogPin);
		float voltage = (raw * vRef) / adcMax;
		float angle = mapVoltageToAngle(voltage);

		Serial.print("Raw: ");
		Serial.print(raw);
		Serial.print(" | Voltage: ");
		Serial.print(voltage, 3);
		Serial.print(" V | Angle: ");
		Serial.print(angle, 2);
		Serial.print(" deg | Calib(-45,+45): ");
		Serial.print(voltageAtMinus45, 3);
		Serial.print(" V, ");
		Serial.print(voltageAtPlus45, 3);
		Serial.println(" V");
	}
}
