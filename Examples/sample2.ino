/*
Code to test HelloSpoon behavior.

  1. Initialize the comm.
  2. Turn Dynamixel ID 4 LED ON, Yellow color.
  3. Move Dynamixel ID 4.
  4. Random positions and LED colors are generated in Loop.

To know the full list of instructions check inside XL320.h
Made by Luis G III for HelloSpoon robot.
*/

#include "HelloSpoon.h"

void setup(){

  hs.begin();
  delay(1000);

  hs.LED(4, 4);
  hs.setJointSpeed(4, 1023);
  hs.moveJoint(4, 212);
  delay(1000);

}

void loop(){  

  hs.moveJoint(4,random(212,512));
  hs.LED(4,random(1,7));
  delay(1000);  

}
