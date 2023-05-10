#include <Servo.h>



Servo myservo;  // create servo object to control a servo

int potpin = A1;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo
					  // object
  Serial.begin(115200);
  delay(300); // .3 sec delay
}


void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  Serial.print(val, DEC);
  Serial.print(" ");

  val = map(val, 0, 1023, 0, 180);     // scale it to use it with the servo (value between 0 and 180)
  Serial.println(val, DEC);

  myservo.write(val);                  // sets the servo position according to the scaled value

  delay(15);                           // waits for the servo to get there

}

// Stiring  PWM - Left 135 <-45-> 90 <-45-> 45 Right
// Rigt 309 54
// Left 793 139

// (309 + 793) / 2 = 551

//
