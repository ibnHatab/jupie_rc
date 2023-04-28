

// HW-511 TCRT5000 Infrared track sensor

int sensorPin = A5;
int sensorValue = 0;
unsigned long  time = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100); // 1 sec delay
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(sensorPin);
  time=millis();

  Serial.print(time, DEC);
  Serial.print(" ");
  Serial.println(sensorValue, DEC);
  delay(20);
}
