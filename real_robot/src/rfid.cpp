#include "rfid.hpp"

rfid::rfid(const SerialPort* const serialPort)
  :EpuckInterface(serialPort)
{

}

//==========================Recieve_Packet=========================//

void rfid::recPacket(unsigned char* const array,int n) const
	{

	int reading = this->serialPort->recvUnsignedCharArray(array,n);

	}

//==========================Select_Tag=========================//
int rfid::selectTag(unsigned char* const tag_id) const
	{



	this->SendRequest(EpuckInterface::SELECT_COMMAND);

	int crc;
	int i;
	int packet_length=7;
	i=0;
	unsigned char rec_pkt[packet_size];

	unsigned char packet[packet_length];
		packet[i++]=0x02;
		packet[i++]=0x05;
		packet[i++]=0x20;
		packet[i++]=0x14;
		packet[i++]=0x01;

		crc=CRC16 (&(packet[1]), 4);
		packet[i++]=(crc>>8)&255;
		packet[i]=crc & 255;



	this->serialPort->sendCharArray(packet, packet_length);



int reading = this->serialPort->recvUnsignedCharArray(rec_pkt,packet_size);



		*(tag_id+0)=*(rec_pkt+3);	//TID
		*(tag_id+1)=*(rec_pkt+4);	//TID
		*(tag_id+2)=*(rec_pkt+5);	//TID
		*(tag_id+3)=*(rec_pkt+6);	//TID
		*(tag_id+4)=*(rec_pkt+7);	//TID
		*(tag_id+5)=*(rec_pkt+8);	//TID
		*(tag_id+6)=*(rec_pkt+9);	//TID
		*(tag_id+7)=*(rec_pkt+10);	//TID

if(rec_pkt[2]!=0x14)
return 0;

else
return 1;

	}

//==========================Read_Tag=========================//

int rfid::readTag(unsigned char* const tag_id,int n,int start_block, unsigned char* const read_packet) const
	{

	this->SendRequest(EpuckInterface::READ_COMMAND);

	int crc;
	int i;
	int packet_length=17;
	unsigned char rec_pkt[packet_size];

	i=0;
	unsigned char packet[packet_length];
		packet[i++]=0x02;		//stx
		packet[i++]=0x0F;		//length
		packet[i++]=0x60;		//flag
		packet[i++]=0x24;		//command
		packet[i++]=0x01;		//tag type
		packet[i++]=*(tag_id+0);	//TID
		packet[i++]=*(tag_id+1);	//TID
		packet[i++]=*(tag_id+2);	//TID
		packet[i++]=*(tag_id+3);	//TID
		packet[i++]=*(tag_id+4);	//TID
		packet[i++]=*(tag_id+5);	//TID
		packet[i++]=*(tag_id+6);	//TID
		packet[i++]=*(tag_id+7);	//TID
		packet[i++]=char(start_block);	//start
		packet[i++]=char(n/4);	//number of blocks

		crc=CRC16 (&(packet[1]), 14);

		packet[i++]=(crc>>8)&255;
		packet[i]=crc & 255;

 	this->serialPort->sendCharArray(packet, packet_length);


int reading = this->serialPort->recvUnsignedCharArray(rec_pkt,packet_size);


for(int i=0;i<n;i++)
{

		*(read_packet+i)=*((rec_pkt+3)+i);	//TID

}



if(rec_pkt[2]!=0x24)
return 0;

else
return 1;

	}

//==========================Write_Tag=========================//

int rfid::writeTag(unsigned char* const tag_id,unsigned char* const data,int n, int start_block, unsigned char* const write_packet) const
	{

	this->SendRequest(EpuckInterface::WRITE_COMMAND);

	int crc;
	int i,j;
	int packet_length=17+n;
	int number_blocks= n/4;
	unsigned char rec_pkt[packet_size];
	i=0;
	
	this->serialPort->sendChar(char(packet_length));

	unsigned char packet[17+n];
		packet[i++]=0x02;
		packet[i++]=(0x0F+n);
		packet[i++]=0x60;
		packet[i++]=0x44;
		packet[i++]=0x01;
		packet[i++]=*(tag_id+0);
		packet[i++]=*(tag_id+1);
		packet[i++]=*(tag_id+2);
		packet[i++]=*(tag_id+3);
		packet[i++]=*(tag_id+4);
		packet[i++]=*(tag_id+5);
		packet[i++]=*(tag_id+6);
		packet[i++]=*(tag_id+7);
		packet[i++]=char(start_block);
		packet[i++]=char(number_blocks);// & 255;
	

	for(j=0;j<n;j++){
		packet[i++]= ((*(data+j)));	
			}
		
		crc=CRC16 (&(packet[1]), 14+n);

		packet[i++]=(crc>>8)&255;
		packet[i]=crc & 255;

	this->serialPort->sendCharArray(packet, packet_length);


int reading = this->serialPort->recvUnsignedCharArray(rec_pkt,packet_size);

for(int i=0;i<5;i++)
{

		*(write_packet+i)=*(rec_pkt+i);

}


if(rec_pkt[2]!=0x44)
return 0;

else
return 1;


	}

//==========================Baud_rate=========================//

int rfid::baudRate(void) const
	{

	this->SendRequest(EpuckInterface::WRITE_COMMAND);

	int crc;
	int i,j;
	int packet_length=9;
	unsigned char rec_pkt[5];
	i=0;
	
	this->serialPort->sendChar(char(packet_length));

	unsigned char packet[9];
		packet[i++]=0x02; // stx
		packet[i++]=0x07; // len
		packet[i++]=0x20; // flag
		packet[i++]=0x41; // cmd
		packet[i++]=0x03; // starting block
		packet[i++]=0x01; // number of blocks
		packet[i++]=0x03; // data
	
		
		crc=CRC16 (&(packet[1]), 6);

		packet[i++]=(crc>>8)&255;
		packet[i]=crc & 255;

	this->serialPort->sendCharArray(packet, packet_length);



int reading = this->serialPort->recvUnsignedCharArray(rec_pkt,packet_size);

for(int i=0;i<5;i++)
{

printf("%02x ",*(rec_pkt+i));
		

}
printf("\n");

//if(rec_pkt[2]!=0x41)
//return 0;

//else
//return 1;


	}
//==========================CRC=========================//

unsigned int rfid::CRC16 (unsigned char *dataP, unsigned char n) const
{
	unsigned char i, j;     // byte counter, bit counter
	unsigned int  crc_16;   // calculation

	crc_16 = 0x0000;        // PRESET value

	for (i = 0; i < n; i++) // check each byte in the array
	{
		
		//printf("I consider: %02x\n", *(dataP));while(getchar()!='\n');
		crc_16 ^= *dataP++;

		for (j = 0; j < 8; j++) 	  // test each bit in the byte
			if (crc_16 & 0x0001)
			{
				crc_16 >>= 1;
				crc_16 ^= 0x8408; // POLYNOMIAL x^16+x^12+x^5+1
			}
			else
				crc_16 >>= 1;
	}

	//printf("RISULTATO: %02x\n", crc_16);
	return (crc_16);	// returns calculated crc (16 bits)
}



//====================INT-CHAR-INT========================//


void rfid::to_char(int n, unsigned char* const char_array)const

{

    char_array[0] = (n & 0xff000000UL) >> 24;
    char_array[1] = (n & 0x00ff0000UL) >> 16;
    char_array[2] = (n & 0x0000ff00UL) >>  8;
    char_array[3] = (n & 0x000000ffUL)      ;

}

int rfid::to_int(unsigned char* const char_array)const

{
int to_int=0;
to_int |= ( (char_array[0]&0x0FF) << 24 );
to_int |= ( (char_array[1]&0x0FF) <<16 );
to_int |= ( (char_array[2]&0x0FF) << 8 );
to_int |= ( (char_array[3]&0x0FF)  );

return to_int;

}


//==========================EOF=========================//





