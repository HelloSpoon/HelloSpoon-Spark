/*
This is the original code provided with your HelloSpoon.

All the Mobile Suite from HelloSpoon will work 
while this code is loaded in your robot.
If you load a new code directly from Spark IDE (http://spark.io/build)
all the Mobile Suite (Animator, Controller and HelloSpoon App) will stop working.

To learn how to upload this code and custom codes to HelloSpoon
please visit the Wiki included in this repository.

Hope this helps you!
Happy coding :-)!

- QOLbotics Team.

*/

#include "HelloSpoon-Spark/HelloSpoon-Spark.h"
#include "Serial2/Serial2.h"

SYSTEM_MODE(SEMI_AUTOMATIC); //This runs setup() immidiately and you have to manually connect to the Spark Cloud.

HelloSpoon robot;

void setup() {
    
    //EEPROM.write(1, 0); //Mandatory use the first time you load the code, then please comment it.
    
    if(EEPROM.read(1) == 0){
        HelloSpoon_FirstUsage();    
    }
    else{
        HelloSpoon_setup();
    }
    
}

void loop() {
    
    if(Serial2.available()){
        if(Serial2.read()=='w'){
            HelloSpoon_Online();
        }
    }
    
}

void HelloSpoon_setup(){
    /*Start HelloSpoon robot*/
    robot.begin();
    /*Start Bluetooth communication*/
    Serial2.begin(57600);
    /*Turn off Wifi module to save battery*/
    WiFi.off();
    delay(1000);
    /*Activate HelloSpoon's trunk*/
    robot.activateTrunk();
}

void HelloSpoon_Online(){
    /*Turn on WiFi module*/
    WiFi.on();
    delay(1000);
    /*Connect to the already known Wifi network*/
    WiFi.connect();
    delay(1000);
    /*Connect to Spark Cloud*/
    Spark.connect();
}

void HelloSpoon_FirstUsage(){
    EEPROM.write(1, 0x01); //Change value on EEPROM's second byte.
    /*Turn on WiFi module*/
    WiFi.on();
    delay(1000);
    /*Connect to the already known Wifi network*/
    WiFi.connect();
    delay(1000);
    /*Connect to Spark Cloud*/
    Spark.connect();
}
