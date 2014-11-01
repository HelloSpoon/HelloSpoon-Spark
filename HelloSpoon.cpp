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
#include "HelloSpoon.h"

// Macro for the selection of the Serial Port
#define sendData(args)  (Serial.write(args))    // Write Over Serial
#define beginCom(args)  (Serial.begin(args))    // Begin Serial Comunication
#define readData()		(Serial.read())	

// Select the Switch to TX/RX Mode Pin
#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode)) 
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

int sendPacket(int ID, int Address, int value);
int RXsendPacket(int ID, int Address);

void DynamixelPro::begin()
{	
	setDPin(Direction_Pin=4,OUTPUT);
	beginCom(1000000);
}	

int DynamixelPro::writeWord(int ID, int Address, int value){
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

int DynamixelPro::readWord(int ID, int Address){

	/*Work in progress...*/
	RXsendPacket(ID, Address);
}

void DynamixelPro::moveJoint(int ID, int value){
	int Address = XL_GOAL_POSITION_L;
	sendPacket(ID, Address, value);
	delay(1);
}

void DynamixelPro::setJointSpeed(int ID, int value){
	int Address = XL_GOAL_SPEED_L;
	sendPacket(ID, Address, value);
	delay(1);
}

void DynamixelPro::LED(int ID, int value){
	int Address = XL_LED;
	sendPacket(ID, Address, value);
	delay(1);
}	

void DynamixelPro::setJointTorque(int ID, int value){
	int Address = XL_GOAL_TORQUE;
	sendPacket(ID, Address, value);
	delay(1);
}

void DynamixelPro::TorqueON(int ID){
	sendPacket(ID, XL_TORQUE_ENABLE, 1);
	delay(1);
}

void DynamixelPro::TorqueOFF(int ID){
	sendPacket(ID, XL_TORQUE_ENABLE, 0);
	delay(1);
}

void DynamixelPro::activateTrunk(){
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_TORQUE_ENABLE, 1);
		delay(1);
	}
}

void DynamixelPro::deactivateTrunk(){
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_TORQUE_ENABLE, 0);
		delay(1);
	}
}

void DynamixelPro::quickTest(){
	
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

int DynamixelPro::getSpoonLoad(){
	int spoon = RXsendPacket(4, XL_PRESENT_LOAD);
	delay(1);
	return spoon;
}

int DynamixelPro::getJointPosition(int ID){
	int pos = RXsendPacket(ID, XL_PRESENT_POSITION);
	delay(1);
	return pos;
}

int DynamixelPro::getJointSpeed(int ID){
	int speed = RXsendPacket(ID, XL_PRESENT_SPEED);
	delay(1);
	return speed;
}

int DynamixelPro::getJointLoad(int ID){
	int load = RXsendPacket(ID, XL_PRESENT_LOAD);
	delay(1);
	return load;
}

int DynamixelPro::getJointTemperature(int ID){
	int temp = RXsendPacket(ID, XL_PRESENT_TEMPERATURE);
	delay(1);
	return temp;
}

int DynamixelPro::isJointMoving(int ID){
	int motion = RXsendPacket(ID, XL_MOVING);
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

DynamixelPro hs;
