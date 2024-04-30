#include <SPI.h>
#include "Adafruit_MAX31855.h"

String adcString[8];

#define GSM_RX 26
#define GSM_TX 25
#define GSM_PWR 22
#define GSM_RST 32

#define MAXDO  19
#define MAXCS   5
#define MAXCLK 18
#define NSS   26

#define OUTPUT1 13
#define OUTPUT2 15

unsigned long int timer1 = 0;

Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);

  delay(1000);
  Serial.println("The device is powered up");

  pinMode(35, INPUT);    // Digital Input 1
  pinMode(34, INPUT);    // Digital Input 2
  pinMode(21, INPUT);    // ADS1115 Interrupt
  pinMode(36, INPUT);    // Analog input
  pinMode(GSM_PWR, OUTPUT);    // 12V boosted output enable
  pinMode(GSM_RST, OUTPUT);    // SIM7000 Power - only on LTE version

  digitalWrite(GSM_RST, HIGH);
  digitalWrite(GSM_PWR, HIGH);
  delay(1000);
  digitalWrite(GSM_PWR, LOW);
  delay(3000);

  timer1 = millis();
  Serial2.println("AT");
  while (millis() < timer1 + 5000) {
    while (Serial2.available()) {
      int inByte = Serial2.read();
      Serial.write(inByte);
    }
  }

  timer1 = millis();
  Serial2.println("AT+CPIN?");
  while (millis() < timer1 + 5000) {
    while (Serial2.available()) {
      int inByte = Serial2.read();
      Serial.write(inByte);
    }
  }

  Serial.println("AT SCAN DONE");

  digitalWrite(MAXCS, LOW);
  digitalWrite(NSS, HIGH);

  digitalWrite(OUTPUT1, HIGH);
  digitalWrite(OUTPUT2, LOW);
  delay(1000);

  digitalWrite(OUTPUT1, LOW);
  digitalWrite(OUTPUT2, HIGH);

  digitalWrite(OUTPUT1, HIGH);
  delay(500);
  digitalWrite(OUTPUT1, LOW);
  delay(1000);

  digitalWrite(OUTPUT2, HIGH);
  delay(500);
  digitalWrite(OUTPUT2, LOW);
  delay(1000);

  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  Serial.println("DONE.");
}

void loop() {
  // Read and print digital inputs
  Serial.print("I1: ");
  Serial.println(digitalRead(35));
  Serial.print("I2: ");
  Serial.println(digitalRead(34));
  
  // Read and print analog input
  Serial.print("Battery Voltage: ");
  Serial.println(readBattery());
  
  // Read and print thermocouple data
  Serial.print("Internal Temp = ");
  Serial.println(thermocouple.readInternal());
  double c = thermocouple.readCelsius();
  if (isnan(c)) {
    Serial.println("Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
    if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
    if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
  } else {
    Serial.print("C = ");
    Serial.println(c);
  }
  
  // Read and print analog reading
  Serial.print("Analog Read : ");
  Serial.print(analogRead(36));

  Serial.println("");
  delay(1000);
}

int readBattery() {
  unsigned int analog_value;
  analog_value = analogRead(36);
  return analog_value;
}
