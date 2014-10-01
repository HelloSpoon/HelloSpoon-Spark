#include <XL320.h>

void setup(){

  XL320.begin();
  delay(1000);

  XL320.LED(5, 4);
  XL320.setJointSpeed(5, 1023);
  XL320.moveJoint(5, 212);
  delay(1000);

}

void loop(){  

  XL320.moveJoint(5,random(212,512));
  XL320.LED(5,random(1,6));
  delay(1000);  

}
