#ifndef RFID_HPP
#define RFID_HPP

#include "epuckInterface.hpp"
#include "serialPort.hpp" 
#include <iostream>
#include <stdio.h>

/** \file
 * Header file of the RFID.cpp (select/read/write data on RFID tag)
 */

#define packet_size 41


class rfid : public EpuckInterface
{
public:


//unsigned char packet[10];


rfid(const SerialPort* const serialPort);

//void SendCommand(void) const;

void recPacket(unsigned char* const array,int n) const;

int selectTag(unsigned char* const tag_id) const;

int readTag(unsigned char* const tag_id,int n,int start_block, unsigned char* const read_packet) const;

int writeTag(unsigned char* const tag_id,unsigned char* const data,int n, int start_block, unsigned char* const write_packet) const;

unsigned int CRC16 (unsigned char *dataP, unsigned char n) const;

void to_char(int n, unsigned char* const char_array)const;

int to_int(unsigned char* const char_array)const;

int baudRate(void)const;

};


#endif
