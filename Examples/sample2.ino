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
