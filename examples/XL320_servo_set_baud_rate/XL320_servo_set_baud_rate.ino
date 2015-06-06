#include "XL320.h"

// Name your robot!
XL320 robot;

void setup() {

  // Talking standard serial, so connect servo data line to Digital TX 1
  // Set the default servo baud rate which is 1000000 (1Mbps)
  Serial.begin(1000000);

  // Initialise your robot
  robot.begin(Serial); // Hand in the serial object you're using

  // Set the serial connection baud rate
  // writePacket(1, XL_BAUD_RATE, x) sets the baud rate:
  // 0: 9600, 1:57600, 2:115200, 3:1Mbps
  robot.sendPacket(1, XL_BAUD_RATE, 2);
}

void loop() {
	// NOTE: load this sketch to the Arduino > Servo
	// Then power cycle the Arduino + Servo

	// Now your servo should be set to the baud rate 2:115200
	// Try it by running the other servo example
}