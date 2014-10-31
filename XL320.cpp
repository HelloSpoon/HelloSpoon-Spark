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
#include "XL320.h"

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

switchCom(Direction_Pin, Rx_MODE);

}

int DynamixelPro::readWord(int ID, int Address){

	/*Work in progress...*/

	RXsendPacket(ID, Address);
}

int DynamixelPro::moveJoint(int ID, int value){
	int Address = 30;
	sendPacket(ID, Address, value);
	delay(1);
}

int DynamixelPro::setJointSpeed(int ID, int value){
	int Address = 32;
	sendPacket(ID, Address, value);
	delay(1);
}

int DynamixelPro::LED(int ID, int value){
	int Address = 25;
	sendPacket(ID, Address, value);
	delay(1);
}	

int DynamixelPro::getSpoonLoad(int ID){

}

int DynamixelPro::setJointTorque(int ID, int value){
	int Address = 35;
	sendPacket(ID, Address, value);
	delay(1);
}

int DynamixelPro::TorqueON(int ID){
	sendPacket(ID, 24, 1);
	delay(1);
}

int DynamixelPro::TorqueOFF(int ID){
	sendPacket(ID, 24, 0);
	delay(1);
}

int DynamixelPro::activateTrunk(){
	for(int id = 1; id < 6; id++){
		sendPacket(id, 24, 1);
		delay(1);
	}
}

int DynamixelPro::deactivateTrunk(){
	for(int id = 1; id < 6; id++){
		sendPacket(id, 24, 0);
		delay(1);
	}
}

int DynamixelPro::quickTest(){
	
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

	switchCom(Direction_Pin, Rx_MODE);

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

DynamixelPro XL320;
