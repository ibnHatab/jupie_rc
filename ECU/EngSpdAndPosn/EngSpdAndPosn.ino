/**
   AUTOSAR CP R21-11
   * Engine Speed and Position (EngSpdAndPosn)

   Functions that provide all parameters linked to engine shaft position and speed, in-
   cluding the synchronisation on between crankshaft and camshaft.

   - Crankshaft and camshaft signal acquisition.
   - Calculation of the engine position.
   - Calculation of the relative camshaft position for systems with variable valve timing
     and/or lift.
   - Related diagnosis and plausibility checks.

   * Steer Manager

   A simple steering management system

   - Get angle and torque data from the physical sensors
   - Process the three sets of angle and torque data

   Snsor implementation:
   Tracks the tick count from Quadrature Encoders on left wheel.

   Wheel with 6 spikes adored with 3 white segments for HW-511 TCRT5000
   Infrared track sensor reflection encoder which output TTL to Arduino
   analog read pin.

   Actuator implementation:

   - ESC Speed Control LM-406FB Operating voltage: 6V-9.6V (5-8 Cells)
      Maximum pulse current: (Forward); Maximum continuous current:
      (Forward); Int. resistance ohms: 0.008x2; weight: 50g RECEIVER
      Type: Single Conversion AM Consists of receiver and speed
      controller Forward and reverse Neutral & full power adjustment
      Battery elimination circuitry Built-in pulse checker Directional
      servo.

	  Throttle setup:
	  Remote control:
	  - Bring speed controller to neutral position and switch on Speed

	  controller:
	  - Switch on (without motor connected)
	  - Press setup button: LED flashes once + 1 x beep
	  - Neutral position learned

	  Remote control:
	  - Speed ​​controller to maximum acceleration

	  Speed ​​controller:
	  - Press setup button: LED flashes twice + 2 x beep
	  - maximum acceleration learned

	  Remote control:
	  - Cruise control to maximum deceleration Speed

	  controller:
	  - Press setup button: LED flashes three times + 3 x beep
	  - maximum deceleration learned
	  - End

   - Servo FUTABA PWM
     Range - Left 135 <-45-> (90) <-45-> 45 Right

*/
#include <Servo.h>

#include <ros.h>
#include <std_msgs/Int16.h>

// Encoder output to Arduino analog pin. Tracks the tick count.
#define ENC_IN_LEFT_A A5
// Programm ESC for current speed
#define PWM_ESC_D 6
// Controll wheel direction
#define PWM_SERVO_D 9

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// main loop spinonce frequence
const int sampling_frequence = 100; // Hz
const int loopTime = 1000 / sampling_frequence; // ms

// Analog encoder signal processing
const int gfilter[3] = {-1, 4, -1}; // Gausian filter
int buff[3] = {0, 0, 0};
const int min_val = 0; // clamp signal
const int max_val = 700;
const int threshold = 1600; // spike on edge

// math
const int spikes = 6; // spikes on wheel (num detected edges)
inline float deg2rad(const float degrees) { return (PI/180) * degrees; }
const float rad_spike = deg2rad(360 / spikes);

// delta time for encoder
long dt = 0;

Servo esc_ctrl;  // create servo object to control a servo
std_msgs::Int16 current_esc_pwm_value = {};
uint16_t target_esc_pwm_value = 90;

Servo servo_ctrl;  // create servo object to control a servo
std_msgs::Int16 current_servo_pwm_value = {};
uint16_t target_servo_pwm_value = 90;

// Keep track of the number of wheel ticks
std_msgs::Int16 left_wheel_revolt;
ros::Publisher leftPub("left_wheel_revolt", &left_wheel_revolt);

// Set up ROS subscriber to the velocity command
ros::Subscriber<std_msgs::Int16> subEscVal("set_esc_pwm_val", &esc_pwm_values);
ros::Subscriber<std_msgs::Int16> subServoVal("set_servo_pwm_val", &servo_pwm_values);

ros::Publisher escValPub("esc_pwm_val", &current_esc_pwm_value);
ros::Publisher servoValPub("servo_pwm_val", &current_servo_pwm_value);


// Take the velocity command as input and calculate the PWM values.
void esc_pwm_values(const std_msgs::Int16& val) {
	target_esc_pwm_value = val.data;

}

void servo_pwm_values(const std_msgs::Int16& val) {
	// Range - Left 135 <-45-> (90) <-45-> 45 Right
	static const uint16_t min_servo_val = 45;
	static const uint16_t max_servo_val = 135;

	target_servo_pwm_value = val.data;

	if(target_servo_pwm_value < min_servo_val)
		target_servo_pwm_value = min_servo_val;
	if(target_servo_pwm_value > max_servo_val)
		target_servo_pwm_value = max_servo_val;
}


long time0;
void setup() {

	// Motor control pins are outputs
	pinMode(PWM_SERVO_D, OUTPUT);
	pinMode(PWM_ESC_D, OUTPUT);

	time0 = millis() - 3000; // w/o to print 0 res/s

	esc_ctrl.attach(PWM_ESC_D);
	current_esc_pwm_value.data = esc_ctrl.read();

	servo_ctrl.attach(PWM_SERVO_D);
	current_servo_pwm_value.data = servo_ctrl.read();

	nh.initNode();

	left_wheel_revolt.data = 0;
	nh.advertise(leftPub);
	nh.advertise(escValPub);
	nh.advertise(servoValPub);
}

int sensorValue = 0;
unsigned long time = 0;
long idle = loopTime;

void loop() {
	time=millis();
	sensorValue = analogRead(ENC_IN_LEFT_A);

	int value = max_val - sensorValue; // invert TTL signal (on:28 ~ off:1200)
	if(value < min_val) value = min_val; // clamp min

	// convolution with filter for edge detection
	buff[0] = buff[1];
	buff[1] = buff[2];
	buff[2] = value;
	int sum = gfilter[0]*buff[0] + gfilter[1]*buff[1] + gfilter[2]*buff[2];

	dt = time - time0;
	if(sum > threshold) {
		// found edge or 3s still
		float omega = rad_spike * 1000 / dt; // rad/s
		left_wheel_revolt.data = (uint16_t)(omega * 100); // rad/s times 100
		time0 = time;
	}

	// Handle servo direction increment
	current_esc_pwm_value.data = servo_ctrl.read();
	int servo_inc = 0;
	if(current_esc_pwm_value.data < target_servo_pwm_value)
		servo_inc = PWM_INCREMENT;
	if(current_esc_pwm_value.data > target_servo_pwm_value)
		servo_inc = -PWM_INCREMENT;
	if(servo_inc != 0) {
		servo_ctrl.write(current_esc_pwm_value.data + servo_inc);
	}


	// ROS node signal handle
	leftPub.publish(&left_wheel_revolt);
	escValPub.publish(&current_esc_pwm_value);
	servoValPub.publish(&current_servo_pwm_value);

	nh.spinOnce();

	// Aling loop time
	idle = loopTime - (millis() - time);
	if(idle > 0) {
		delay(idle);
	}
}
