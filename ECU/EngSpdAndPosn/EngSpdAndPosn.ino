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

#include <ros.h>
#include <std_msgs/Int16.h>

#define ENC_IN_LEFT_A A5

// Handles startup and shutdown of ROS
ros::NodeHandle nh;


void setup() {
	nh.initNode();

}

void loop() {
	nh.spinOnce();
	delay(1000);
}
