#include <Arduino.h>
#include <ESP32Servo.h>

int APin = 33;

void setup() {
	Serial.begin(115200);

}
void loop() {
    analogWrite(APin,255);
    delay(500);
    analogWrite(APin,0);
    delay(500);

}