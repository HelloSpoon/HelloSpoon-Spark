#include "HelloSpoon.h"

void setup(){

  hs.begin();
  delay(1000);

  hs.LED(5, 4);
  hs.setJointSpeed(5, 1023);
  hs.moveJoint(5, 212);
  delay(1000);

}

void loop(){  

  hs.moveJoint(5,random(212,512));
  hs.LED(5,random(1,7));
  delay(1000);  

}
