/*
Code to test XL-320 with Arduino.

1. Initialize the comm.
2. Move XL-320 actuator.
3. Turn on LED of XL-320.

To know the full list of instructions check inside XL320.h

Made by Luis G III for HelloSpoon robot.

*/

#include "HelloSpoon.h"

void setup(){
  XL320.begin();
  delay(1000);
  XL320.writeWord(5,30,512);
  delay(1000);
  XL320.writeWord(5,25,1);
  delay(1000);
}
void loop(){}
