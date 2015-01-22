/*

 Code based on:
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.

 Modified to work only with Dynamixel XL-320 actuator.

 Modifications made by Luis G III for HelloSpoon robot.
 Webpage: http://hellospoonrobot.com
 Twitter: @HelloSpoon
 Youtube: http://youtube.com/user/hellospoonrobot

 This file can be used and be modified by anyone, 
 don't forget to say thank you to OP!
 
 */
#ifdef SPARK
#include "application.h" //Spark Core compatible library
#else
#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#endif

#include "dxl_pro.h"
#include "HelloSpoon-Spark.h"

// Macro for the selection of the Serial Port
#define sendData(args)  (Serial1.write(args))    // Write Over Serial
#define beginCom(args)  (Serial1.begin(args))    // Begin Serial Comunication
#define readData()		(Serial1.read())	

// Select the Switch to TX/RX Mode Pin
#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode)) 
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

int sendPacket(int ID, int Address, int value);
int RXsendPacket(int ID, int Address);

HelloSpoon::HelloSpoon() {
	// TODO Auto-generated constructor stub

}

HelloSpoon::~HelloSpoon() {
	// TODO Auto-generated destructor stub
}

void HelloSpoon::begin()
{	
	setDPin(Direction_Pin=4,OUTPUT);
	beginCom(1000000);
}	

int HelloSpoon::writeWord(int ID, int Address, int value){
word cont, wchecksum, wpacklen;

byte txbuffer[255];

gbpParamEx[0]	= (unsigned char)DXL_LOBYTE(Address);
gbpParamEx[1]	= (unsigned char)DXL_HIBYTE(Address);
//insert data to parameter bucket
gbpParamEx[2]	= DXL_LOBYTE(value);
gbpParamEx[3]	= DXL_HIBYTE(value);

txbuffer[0] = 0xff;
txbuffer[1] = 0xff;
txbuffer[2] = 0xfd;
txbuffer[3] = 0x00;

txbuffer[4] = ID;
txbuffer[5] = DXL_LOBYTE(4+3);
txbuffer[6] = DXL_HIBYTE(4+3);

txbuffer[7] = 0x03;

for(cont = 0; cont < 4; cont++)
    {
        txbuffer[cont+8] = gbpParamEx[cont];
    }

wchecksum = 0;

wpacklen = DXL_MAKEWORD(txbuffer[5], txbuffer[6])+5;
if(wpacklen > (MAXNUM_TXPACKET)){
        return 0;
    }

wchecksum = update_crc(0, txbuffer, wpacklen);
txbuffer[wpacklen] = DXL_LOBYTE(wchecksum);
txbuffer[wpacklen+1] = DXL_HIBYTE(wchecksum);

wpacklen += 2;

switchCom(Direction_Pin, Tx_MODE);

for(cont = 0; cont < wpacklen; cont++)
    {
        sendData(txbuffer[cont]);
    }

}

int HelloSpoon::readWord(int ID, int Address){

	/*Work in progress...*/
	RXsendPacket(ID, Address);
}

void HelloSpoon::moveJoint(int Joint, int value){
	int Address = XL_GOAL_POSITION_L;
	
	if(Joint == 1){
		sendPacket(1, Address, value);
		delay(1);
	}
	else if(Joint == 2){
		sendPacket(2, Address, value);
		delay(1);
		sendPacket(3, Address, 1023-value);
		delay(1);
	}
	else{
		sendPacket(Joint+1, Address, value);
		delay(1);
	}
}

void HelloSpoon::setJointSpeed(int Joint, int value){
	int Address = XL_GOAL_SPEED_L;
	if(Joint == 1){
		sendPacket(1, Address, value);
		delay(1);
	}
	else if(Joint == 2){
		sendPacket(2, Address, value);
		delay(1);
		sendPacket(3, Address, value);
		delay(1);
	}
	else{
		sendPacket(Joint+1, Address, value);
		delay(1);
	}
}

void HelloSpoon::LED(int Joint, char led_color[]){
	int Address = XL_LED;
	int val = 0;
	
	if(led_color[0] == 'r'){
		val = 1;
	}

	else if(led_color[0] == 'g'){
		val = 2;
	}

	else if(led_color[0] == 'y'){
		val = 3;
	}

	else if(led_color[0] == 'b'){
		val = 4;
	}

	else if(led_color[0] == 'p'){
		val = 5;
	}

	else if(led_color[0] == 'c'){
		val = 6;
	}

	else if(led_color[0] == 'w'){
		val = 7;
	}
	
	else if(led_color[0] == 'o'){
		val = 0;
	}
	
	if(Joint == 1){
		sendPacket(1, Address, val);
		delay(1);
	}
	else if(Joint == 2){
		sendPacket(2, Address, val);
		delay(1);
		sendPacket(3, Address, val);
		delay(1);
	}
	else{
		sendPacket(Joint+1, Address, val);
		delay(1);
	}
}	

void HelloSpoon::setJointTorque(int Joint, int value){
	int Address = XL_GOAL_TORQUE;
	if(Joint == 1){
		sendPacket(1, Address, value);
		delay(1);
	}
	else if(Joint == 2){
		sendPacket(2, Address, value);
		delay(1);
		sendPacket(3, Address, value);
		delay(1);
	}
	else{
		sendPacket(Joint+1, Address, value);
		delay(1);
	}
}

void HelloSpoon::TorqueON(int Joint){
	
	int Address = XL_TORQUE_ENABLE;
	int value = 1;
	
	if(Joint == 1){
		sendPacket(1, Address, value);
		delay(1);
	}
	else if(Joint == 2){
		sendPacket(2, Address, value);
		delay(1);
		sendPacket(3, Address, value);
		delay(1);
	}
	else{
		sendPacket(Joint+1, Address, value);
		delay(1);
	}
}

void HelloSpoon::TorqueOFF(int Joint){
	
	int Address = XL_TORQUE_ENABLE;
	int value = 0;
	
	if(Joint == 1){
		sendPacket(1, Address, value);
		delay(1);
	}
	else if(Joint == 2){
		sendPacket(2, Address, value);
		delay(1);
		sendPacket(3, Address, value);
		delay(1);
	}
	else{
		sendPacket(Joint+1, Address, value);
		delay(1);
	}
}

void HelloSpoon::activateTrunk(){
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_TORQUE_ENABLE, 1);
		delay(1);
		sendPacket(id, XL_LED, 4);
		delay(1);
	}
}

void HelloSpoon::deactivateTrunk(){
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_TORQUE_ENABLE, 0);
		delay(1);
		sendPacket(id, XL_LED, 5);
		delay(1);
	}
}

void HelloSpoon::quickTest(){
	
	int position_tmp = 0;
	
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_LED, random(1,7));
		delay(1);
		sendPacket(id, XL_GOAL_SPEED_L, 200);
		delay(300);
	}
	
	for(int id = 1; id < 6; id++){
	    
	    position_tmp = random(0,512); 
		
		if(id != 3){
		    sendPacket(id, XL_GOAL_POSITION_L, position_tmp);
			delay(1000);
		}
		
		else{
			sendPacket(3, XL_GOAL_POSITION_L, 512-position_tmp);
			delay(1000);
		}
	}
	
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_LED, 2);
		delay(1);
		sendPacket(id, XL_GOAL_SPEED_L, 1023);
		delay(300);
	}
	
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_LED, 0);
		delay(300);
	}
	
}

int HelloSpoon::getSpoonLoad(){
	int spoon = RXsendPacket(5, XL_PRESENT_LOAD);
	delay(1);
	return spoon;
}

int HelloSpoon::getJointPosition(int Joint){
    int pos = 0;
	switch(Joint){
		case 1: pos = RXsendPacket(1, XL_PRESENT_POSITION); 
		        break;
		case 2: pos = RXsendPacket(2, XL_PRESENT_POSITION); 
		        break;
		case 3: pos = RXsendPacket(4, XL_PRESENT_POSITION); 
		        break;
		case 4: pos = RXsendPacket(5, XL_PRESENT_POSITION); 
		        break;
	}
	delay(1);
	return pos;
}

int HelloSpoon::getJointSpeed(int Joint){
    int speed = 0;
	switch(Joint){
		case 1: speed = RXsendPacket(1, XL_PRESENT_SPEED); 
		        break;
		case 2: speed = RXsendPacket(2, XL_PRESENT_SPEED); 
		        break;
		case 3: speed = RXsendPacket(4, XL_PRESENT_SPEED); 
		        break;
		case 4: speed = RXsendPacket(5, XL_PRESENT_SPEED); 
		        break;
	}
	delay(1);
	return speed;
}

int HelloSpoon::getJointLoad(int Joint){
    int load = 0;
	switch(Joint){
		case 1: load = RXsendPacket(1, XL_PRESENT_LOAD); 
		        break;
		case 2: load = RXsendPacket(2, XL_PRESENT_LOAD); 
		        break;
		case 3: load = RXsendPacket(4, XL_PRESENT_LOAD); 
		        break;
		case 4: load = RXsendPacket(5, XL_PRESENT_LOAD); 
		        break;
	}
	delay(1);
	return load;
}

int HelloSpoon::getJointTemperature(int Joint){
    int temp = 0;
	switch(Joint){
		case 1: temp = RXsendPacket(1, XL_PRESENT_TEMPERATURE); 
		        break;
		case 2: temp = RXsendPacket(2, XL_PRESENT_TEMPERATURE); 
		        break;
		case 3: temp = RXsendPacket(4, XL_PRESENT_TEMPERATURE); 
		        break;
		case 4: temp = RXsendPacket(5, XL_PRESENT_TEMPERATURE); 
		        break;
	}
	delay(1);
	return temp;
}

int HelloSpoon::isJointMoving(int Joint){
    int motion = 0;
	switch(Joint){
		case 1: motion = RXsendPacket(1, XL_MOVING);
		        break;
		case 2: motion = RXsendPacket(2, XL_MOVING); 
		        break;
		case 3: motion = RXsendPacket(4, XL_MOVING); 
		        break;
		case 4: motion = RXsendPacket(5, XL_MOVING); 
		        break;
	}
	delay(1);
	return motion;
}

int sendPacket(int ID, int Address, int value){

	/*Dynamixel 2.0 communication protocol
	  used by Dynamixel XL-320 and Dynamixel PRO only.
	*/

	word cont, wchecksum, wpacklen;
	volatile char gbpParamEx[130+10];
	unsigned char Direction_Pin;

	byte txbuffer[255];

	gbpParamEx[0]	= (unsigned char)DXL_LOBYTE(Address);
	gbpParamEx[1]	= (unsigned char)DXL_HIBYTE(Address);

	gbpParamEx[2]	= DXL_LOBYTE(value);
	gbpParamEx[3]	= DXL_HIBYTE(value);

	txbuffer[0] = 0xff;
	txbuffer[1] = 0xff;
	txbuffer[2] = 0xfd;
	txbuffer[3] = 0x00;

	txbuffer[4] = ID;
	txbuffer[5] = DXL_LOBYTE(4+3);
	txbuffer[6] = DXL_HIBYTE(4+3);

	txbuffer[7] = 0x03;

	for(cont = 0; cont < 4; cont++)
    	{
        	txbuffer[cont+8] = gbpParamEx[cont];
    	}

	wchecksum = 0;

	wpacklen = DXL_MAKEWORD(txbuffer[5], txbuffer[6])+5;
	if(wpacklen > (MAXNUM_TXPACKET)){
        return 0;
    }

	wchecksum = update_crc(0, txbuffer, wpacklen);
	txbuffer[wpacklen] = DXL_LOBYTE(wchecksum);
	txbuffer[wpacklen+1] = DXL_HIBYTE(wchecksum);

	wpacklen += 2;

	switchCom(Direction_Pin, Tx_MODE);

	for(cont = 0; cont < wpacklen; cont++)
    {
    	sendData(txbuffer[cont]);
    }

	//switchCom(Direction_Pin, Rx_MODE);

}

int RXsendPacket(int ID, int Address){

	/*Dynamixel 2.0 communication protocol
	  used by Dynamixel XL-320 and Dynamixel PRO only.
	*/

	word cont, wchecksum, wpacklen;
	volatile char gbpParamEx[130+10];
	unsigned char Direction_Pin;

	byte txbuffer[255];

	gbpParamEx[0]	= (unsigned char)DXL_LOBYTE(Address);
	gbpParamEx[1]	= (unsigned char)DXL_HIBYTE(Address);

	gbpParamEx[2]	= 2;
	gbpParamEx[3]	= 0;

	txbuffer[0] = 0xff;
	txbuffer[1] = 0xff;
	txbuffer[2] = 0xfd;
	txbuffer[3] = 0x00;

	txbuffer[4] = ID;
	txbuffer[5] = DXL_LOBYTE(4+3);
	txbuffer[6] = DXL_HIBYTE(4+3);

	txbuffer[7] = 0x03;

	for(cont = 0; cont < 4; cont++)
    	{
        	txbuffer[cont+8] = gbpParamEx[cont];
    	}

	wchecksum = 0;

	wpacklen = DXL_MAKEWORD(txbuffer[5], txbuffer[6])+5;
	if(wpacklen > (MAXNUM_TXPACKET)){
        return 0;
    }

	wchecksum = update_crc(0, txbuffer, wpacklen);
	txbuffer[wpacklen] = DXL_LOBYTE(wchecksum);
	txbuffer[wpacklen+1] = DXL_HIBYTE(wchecksum);

	wpacklen += 2;

	switchCom(Direction_Pin, Tx_MODE);

	for(cont = 0; cont < wpacklen; cont++)
    {
    	sendData(txbuffer[cont]);
    }

	switchCom(Direction_Pin, Rx_MODE);

}