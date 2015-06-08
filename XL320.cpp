/*

 Code based on:
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 And Dynamixel Pro library for OpenCM-9.04 made by ROBOTIS, LTD.

 Modified to work only with Dynamixel XL-320 actuator.

 Modifications made by Luis G III for XL320 robot.
 Webpage: http://hellospoonrobot.com
 Twitter: @XL320
 Youtube: http://youtube.com/user/hellospoonrobot

 This file can be used and be modified by anyone, 
 don't forget to say thank you to OP!
 
 */

#include "Arduino.h"
#include "dxl_pro.h"
#include "XL320.h"

// Macro for the selection of the Serial Port
#define sendData(args)  (this->stream->write(args))    // Write Over Serial
#define beginCom(args)      // Begin Serial Comunication
#define readData()		(this->stream->read())	

// Select the Switch to TX/RX Mode Pin
#define setDPin(DirPin,Mode)    
#define switchCom(DirPin,Mode)   // Switch to TX/RX Mode

#define NANO_TIME_DELAY 12000


XL320::XL320() {

}

XL320::~XL320() {
}

void XL320::begin(Stream &stream)
{	
	//setDPin(Direction_Pin=4,OUTPUT);
	//beginCom(1000000);
    this->stream = &stream;
}	

void XL320::moveJoint(int id, int value){
	int Address = XL_GOAL_POSITION_L;
	
	sendPacket(id, Address, value);
	this->stream->flush();

	nDelay(NANO_TIME_DELAY);
}

void XL320::setJointSpeed(int id, int value){
	int Address = XL_GOAL_SPEED_L;
	sendPacket(id, Address, value);
	this->stream->flush();

	nDelay(NANO_TIME_DELAY);

}

void XL320::LED(int id, char led_color[]){
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
	
	sendPacket(id, Address, val);
	this->stream->flush();
	
	nDelay(NANO_TIME_DELAY);
}	

void XL320::setJointTorque(int id, int value){
	int Address = XL_GOAL_TORQUE;
	sendPacket(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);

}

void XL320::TorqueON(int id){
	
	int Address = XL_TORQUE_ENABLE;
	int value = 1;
	
	sendPacket(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}

void XL320::TorqueOFF(int id){
	
	int Address = XL_TORQUE_ENABLE;
	int value = 0;
	
	sendPacket(id, Address, value);
	this->stream->flush();
	nDelay(NANO_TIME_DELAY);
}


void XL320::quickTest(){
	
	int position_tmp = 0;
	
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_LED, random(1,7));
		nDelay(NANO_TIME_DELAY);
		this->stream->flush();
		sendPacket(id, XL_GOAL_SPEED_L, 200);
		nDelay(NANO_TIME_DELAY);
		this->stream->flush();
	}
	
	for(int id = 1; id < 6; id++){
	    
	    position_tmp = random(0,512); 
		
		if(id != 3){
		    sendPacket(id, XL_GOAL_POSITION_L, position_tmp);
			delay(1000);
			this->stream->flush();
		}
		
		else{
			sendPacket(3, XL_GOAL_POSITION_L, 512-position_tmp);
			delay(1000);
			this->stream->flush();
		}
	}
	
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_LED, 2);
		nDelay(NANO_TIME_DELAY);
		this->stream->flush();
		sendPacket(id, XL_GOAL_SPEED_L, 1023);
		nDelay(NANO_TIME_DELAY);
		this->stream->flush();
	}
	
	for(int id = 1; id < 6; id++){
		sendPacket(id, XL_LED, 0);
		nDelay(NANO_TIME_DELAY);
		this->stream->flush();
	}
	
}

int XL320::getSpoonLoad(){
	int spoon = RXsendPacket(5, XL_PRESENT_LOAD);
	nDelay(NANO_TIME_DELAY);
	this->stream->flush();
	return spoon;
}

int XL320::getJointPosition(int id){
    int pos = 0;
    pos = RXsendPacket(id, XL_PRESENT_POSITION); 
    this->stream->flush();
    nDelay(NANO_TIME_DELAY);
    return pos;
}

int XL320::getJointSpeed(int id){
    int speed = RXsendPacket(id, XL_PRESENT_SPEED); 
    this->stream->flush();
    nDelay(NANO_TIME_DELAY);
    return speed;
}

int XL320::getJointLoad(int id){
    int load = RXsendPacket(id, XL_PRESENT_LOAD); 
    this->stream->flush();
    nDelay(NANO_TIME_DELAY);
    return load;
}

int XL320::getJointTemperature(int id){
    int temp = RXsendPacket(id, XL_PRESENT_TEMPERATURE); 
    this->stream->flush();
    nDelay(NANO_TIME_DELAY);
    return temp;
}

int XL320::isJointMoving(int id){
    int motion = RXsendPacket(id, XL_MOVING);
    this->stream->flush();
    nDelay(NANO_TIME_DELAY);
    return motion;
}

int XL320::sendPacket(int id, int Address, int value){

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

	txbuffer[4] = id;
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

	//switchCom(Direction_Pin, Tx_MODE);

	for(cont = 0; cont < wpacklen; cont++)
    {
    	sendData(txbuffer[cont]);
    	nDelay(NANO_TIME_DELAY);
    }
    
    //Clean buffer
	for(cont = 0; cont < wpacklen; cont++)
    {
    	txbuffer[cont] = 0;
    }
	//switchCom(Direction_Pin, Rx_MODE);

}

void XL320::nDelay(uint32_t nTime){
    /*
	uint32_t max;
	for( max=0; max < nTime; max++){

	}
    */
}

int XL320::flush() {
    this->stream->flush();
}

int XL320::RXsendPacket(int id, int Address){

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

	txbuffer[4] = id;
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

	//switchCom(Direction_Pin, Tx_MODE);

	for(cont = 0; cont < wpacklen; cont++)
	{
	   sendData(txbuffer[cont]);
	}

	//switchCom(Direction_Pin, Rx_MODE);

}

// from http://stackoverflow.com/a/133363/195061

#define FSM
#define STATE(x)        s_##x : if(!stream->readBytes(&BUFFER[I++],1)) goto sx_timeout ; if(I>=SIZE) goto sx_overflow; sn_##x :
#define LASTBYTE        (BUFFER[I-1])
#define NEXTSTATE(x)    goto s_##x
#define NEXTSTATE_NR(x) goto sn_##x
#define OVERFLOW        sx_overflow :
#define TIMEOUT         sx_timeout :

int XL320::readPacket(unsigned char *BUFFER, size_t SIZE) {
    int C;
    int I = 0;    

    int length = 0;

      // define fsm here
      // write in to buffer 
      // keep track of index
      // return if we overflox max_txlength(?)
      // return if reads timeout? setTimeout behaviour?	

    FSM {
      STATE(start) {
	if(LASTBYTE==0xFF) NEXTSTATE(header_ff_1);
	I=0; NEXTSTATE(start);
      }
      STATE(header_ff_1) {
	if(LASTBYTE==0xFF) NEXTSTATE(header_ff_2);
	I=0; NEXTSTATE(start);	
      }
      STATE(header_ff_2) {
	if(LASTBYTE==0xFD) NEXTSTATE(header_fd);
      }
      STATE(header_fd) {
      }
      STATE(header_reserved) {
      }
      STATE(id) {
	length = LASTBYTE;
      }
      STATE(length_1) {
	length += LASTBYTE<<8; // eg: length=4
      }
      STATE(length_2) {
      }
      STATE(instr) {
        // check length. I==9 here
        // action and reboot commands have no parameters
	if(I-length>=5) NEXTSTATE(checksum_1);
      }
      STATE(params) {
	  // check length
	  if(I-length>=5) NEXTSTATE(checksum_1);
	  NEXTSTATE(params);
      }
      STATE(checksum_1) {
      }
      STATE(checksum_2) {
	  // done
	  return I; 
      }
      OVERFLOW {
          return -1;
      }
      TIMEOUT {
	  return -2;
      }

    }
}

/*
	class Packet {
	  public:
	    unsigned char *data;
	    size_t data_size;

	    unsigned char getId();
	    int getLength();
	    int getParameterCount();
	    unsigned char getInstruction();
            unsigned char getParameter(int n);
	    bool isValid();

	}
	*/

XL320::Packet::Packet(unsigned char *data, size_t size) {
    this->data = data;
    this->data_size = size;
}

unsigned char XL320::Packet::getId() {
    return data[4];
}

int XL320::Packet::getLength() {
    return data[5]+(data[6]<<8);
}

int XL320::Packet::getParameterCount() {
    return getLength()-3;
}

unsigned char XL320::Packet::getInstruction() {
    return data[7];
}

unsigned char XL320::Packet::getParameter(int n) {
    return data[8+n];
}

bool XL320::Packet::isValid() {
    int length = getLength();
    unsigned short storedChecksum = data[length+5]+(data[length+6]<<8);
    return storedChecksum == update_crc(0,data,length+5);
}
