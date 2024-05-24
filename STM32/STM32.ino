#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <MiCS6814-I2C.h>
HardwareSerial SerialESP(PA10, PA9); // RX, TX

const int CO_PIN = PA0;  // CO connected to PA0 (ADC1_IN0)
const int NO2_PIN = PA1; // NO2 connected to PA1 (ADC1_IN1)
const int dustPin = A0;  // Dust sensor connected to A0
const int ledPower = 3;  // Dust sensor LED control pin

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;

void setup() {
  Serial.begin(115200);
  SerialESP.begin(115200);
  analogReadResolution(12);  // 12-bit resolution for the ADC
  pinMode(ledPower, OUTPUT);
}

void loop() {
  int co_adc = analogRead(CO_PIN);
  int no2_adc = analogRead(NO2_PIN);

  float co_voltage = adcToVoltage(co_adc);
  float no2_voltage = adcToVoltage(no2_adc);

  // float co_concentration = voltageToPpmCO(co_voltage); // CO in ppm
  // float no2_concentration = voltageToPpmNO2(no2_voltage); // NO2 in ppm
  float co_concentration = map(co_adc, 0, 4095, 1, 1000); // CO in ppm
  float no2_concentration = map(no2_adc, 0, 4095, 0.05, 10); // NO2 in ppm

  Serial.print("CO ADC: "); Serial.print(co_adc);
  Serial.print(" CO Voltage: "); Serial.println(co_voltage);
  Serial.print("NO2 ADC: "); Serial.print(no2_adc);
  Serial.print(" NO2 Voltage: "); Serial.println(no2_voltage);

  digitalWrite(ledPower, LOW);  // Power on the dust sensor LED
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(dustPin);

  delayMicroseconds(deltaTime);
  digitalWrite(ledPower, HIGH);  // Turn off the dust sensor LED
  delayMicroseconds(sleepTime);

  calcVoltage = voMeasured * (5.0 / 4095.0);  // Convert to voltage with 5V reference
  dustDensity = calculateDustDensity(calcVoltage); // Dust density in mg/m^3

  Serial.print("Dust ADC: "); Serial.print(voMeasured);
  Serial.print(" Dust Voltage: "); Serial.println(calcVoltage);

  const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(3) + 60;
  StaticJsonDocument<capacity> doc;
  JsonObject data = doc.createNestedObject("data");
  data["co"] = serialized(String(co_concentration, 4));  // CO in ppm
  data["no2"] = serialized(String(no2_concentration, 4)); // NO2 in ppm
  data["dust"] = serialized(String(dustDensity, 4));  // Dust density in ug/m^3

  SerialESP.print('<');
  SerialESP.print((doc.as<String>()).c_str());
  SerialESP.println('>');
  Serial.print('<');
  Serial.print((doc.as<String>()).c_str());
  Serial.println('>');

  delay(1000);
}

float adcToVoltage(int adcValue) {
  return ((float)adcValue / 4095.0) * 5.0;
}

float voltageToPpmCO(float voltage) {
  return voltage * 0.01; 
}

float voltageToPpmNO2(float voltage) {
  return voltage * 0.02;
}

float calculateDustDensity(float voltage) {
  float dustDensity = 170.0 * voltage - 0.1;  
  return dustDensity; // Dust density in ug/m^3
}




