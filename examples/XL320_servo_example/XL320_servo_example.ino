#include "XL320.h"

// Name your robot!
XL320 robot;

// If you want to use Software Serial, uncomment this line
#include <SoftwareSerial.h>

// Set the SoftwareSerial RX & TX pins
SoftwareSerial mySerial(10, 11); // (RX, TX)

void setup() {

  // Talking standard serial, so connect servo data line to Digital TX 1
  // Comment out this line to talk software serial
  Serial.begin(115200);

  // Setup Software Serial
  mySerial.begin(115200);

  // Initialise your robot
  robot.begin(mySerial); // Hand in the serial object you're using
  
  // I like fast moving servos, so set the joint speed to max!
  robot.setJointSpeed(1, 1023);

}

void loop() {

  // LED test.. let's randomly set the colour (0-7)
  char rgb[] = "rgybpcwo";
  robot.LED(1, &rgb[random(0,7)] );
  delay(300);

  // Servo test.. let's randomly set the position (0-1023)
  robot.moveJoint(1, random(0, 1023));
  delay(1000);
}
