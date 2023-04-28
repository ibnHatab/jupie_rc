//
// model: readTchometerFromSerial.m
// BSP
// HW-511 TCRT5000 Infrared track sensor
const int pinSensor = A5;

// main loop
const int sampling_frequence = 100; // Hz
const int loopTime = 1000 / sampling_frequence; // ms

// signal processing
const int gfilter[3] = {-1, 4, -1}; // Gausian filter
int buff[3] = {0, 0, 0};
const int min_val = 0; // clamp signal
const int max_val = 700;
const int threshold = 1600; // spike on edge

// math
const int spikes = 6; // spikes on wheel (num detected edges)
inline float deg2rad(const float degrees) { return (PI/180) * degrees; }
const float rad_spike = deg2rad(360 / spikes);


// reporting
int omega_dec = 0;
int pwm = 0;
long dt = 0;

inline void write_to_serial() {
  Serial.print(dt, DEC);
  Serial.print(" ");
  Serial.println(omega_dec, DEC);
}


int sensorValue = 0;
unsigned long time = 0;
long idle = loopTime;
long time0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  time0 = millis() - 3000; // w/o to print 0 res/s
  delay(300); // .3 sec delay
}

void loop() {
  time=millis();
  sensorValue = analogRead(pinSensor);
  int value = max_val - sensorValue; // invert TTL signal (on:28 ~ off:1200)
  if(value < min_val) value = min_val; // clamp min

  // convolution with filter for edge detection
  buff[0] = buff[1];
  buff[1] = buff[2];
  buff[2] = value;
  int sum = gfilter[0]*buff[0] + gfilter[1]*buff[1] + gfilter[2]*buff[2];

  dt = time - time0;
  if(sum > threshold || dt > 3000) {
	  // found edge or 3s still
	  float omega = rad_spike * 1000 / dt; // rad/s
	  omega_dec = (int)(omega * 100); // rad/s scale 100

	  write_to_serial(); // time, omega, pwm

	  time0 = time;
  }

  // Aling loop time
  idle = loopTime - (millis() - time);
  if(idle > 0) {
	  delay(idle);
  }
}
