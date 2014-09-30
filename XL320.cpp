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

#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "dxl_pro.h"
#include "XL320.h"

// Macro for the selection of the Serial Port
#define sendData(args)  (Serial.write(args))    // Write Over Serial
#define beginCom(args)  (Serial.begin(args))    // Begin Serial Comunication

// Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

void DynamixelPro::begin()
{	
	beginCom(1000000);
}	

int DynamixelPro::writeWord(int ID, int Address, int value){
word cont, wchecksum, wpacklen;

byte txbuffer[255];

volatile byte gbpParamEx[130+10];

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

DynamixelPro XL320;
