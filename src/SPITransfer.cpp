//#include "Arduino.h"

// #if not(defined(MBED_H) || defined(__SAM3X8E__) || defined(DISABLE_SPI_SERIALTRANSFER)) // These boards are/will not be supported by SPITransfer.h

#include "SPITransfer.h"

static CircularBuffer<unsigned char,260> SPIBuffer;
static volatile bool ready_2_send_flag = 0;
static volatile bool sending_flag = 0;

static SPISlave_T4* port;

#define INT_PIN 9


//TODO: try to only send one byte per routine and use static variable to hold index
static void _interrupt_routine()
{
//   Serial.println("ouf");
    while (port->available()) {
        if (!SPIBuffer.isEmpty() && ready_2_send_flag)
        {
			if (!sending_flag) sending_flag = 1;

			// Serial1.print("buffer size: "); Serial1.println(SPIBuffer.size());

			// Serial.print("sending data: "); Serial.println(SPIBuffer.first());
			port->pushr(SPIBuffer.shift());
			port->popr();
			// Serial1.print(port->popr()); Serial1.print(" ");

			if (SPIBuffer.isEmpty())
			{
				ready_2_send_flag = 0;
				sending_flag = 0;
				digitalWrite(INT_PIN,LOW);
			}
        }
        else
        {
			if (ready_2_send_flag && sending_flag)
			{
				ready_2_send_flag = 0;
				sending_flag = 0;
				digitalWrite(INT_PIN,LOW);
			}

		//   Serial1.print("sending_flag: "); Serial1.println(sending_flag);
		//   Serial1.print("ready_2_send_flag: "); Serial1.println(ready_2_send_flag);
        //   Serial1.print("buffer len: "); Serial1.println(SPIBuffer.size());

          if (!ready_2_send_flag)
          {
			port->popr();
			//   Serial1.print("whats popping: "); Serial1.println(port->popr());
          }
        }
    }
}

// //Only send one byte per interrupt routine to keep it short
static void _interrupt_routinev2()
{
    if (port->available()) {
        if (!SPIBuffer.isEmpty() && ready_2_send_flag)
        {
			if (!sending_flag) sending_flag = 1;
			// Serial1.print("buffer size: "); Serial1.println(SPIBuffer.size());
			// Serial1.print(); Serial1.print(" ");
			port->popr();
			// Serial.print("sending data"); // Serial.println(SPIBuffer.first());
			port->pushr(SPIBuffer.shift());

			if (SPIBuffer.isEmpty()) // We are finished transmitting, clear flags
			{
				ready_2_send_flag = 0;
				sending_flag = 0;
				digitalWrite(INT_PIN,LOW);
			}
        }
        else 
        {
			if (!ready_2_send_flag) // We are not transmitting
			{
				port->popr();
				// Serial1.print("whats popping:"); Serial1.println();
			}
        }
    }
}

//TODO:
// then move sendData function to interrupt routine
// must also move packet to header static

/*
 void SPITransfer::begin(SPIClass &_port, configST configs, const uint8_t &_SS)
 Description:
 ------------
  * Advanced initializer for the SPITransfer Class
 Inputs:
 -------
  * const SPIClass &_port - SPI port to communicate over
  * const configST configs - Struct that holds config
  * const uint8_t &_SS - SPI buslave select pin used
  values for all possible initialization parameters
 Return:
 -------
  * void
*/

void SPITransfer::begin(SPISlave_T4* _port, const configST configs, const uint8_t& _SS)
{
	port = _port;
	packet.begin(configs);
  	port->begin(MSBFIRST, SPI_MODE0);
	port->onReceive(_interrupt_routine);
	ssPin = _SS;

	
  	pinMode(INT_PIN,OUTPUT);
  	digitalWrite(INT_PIN,LOW);
}



/*
 void SPITransfer::begin(SPIClass &_port, const uint8_t &_SS, const bool _debug, Stream &_debugPort)
 Description:
 ------------
  * Simple initializer for the SPITransfer Class
 Inputs:
 -------
  * const Stream &_port - SPI port to communicate over
  * const uint8_t &_SS - SPI buslave select pin used
  * const bool _debug - Whether or not to print error messages
  * const Stream &_debugPort - Serial port to print error messages
 Return:
 -------
  * void
*/


void SPITransfer::begin(SPISlave_T4* _port, const uint8_t& _SS, const bool _debug, Stream& _debugPort)
{
	port = _port;
	packet.begin(_debug, _debugPort);
	port->begin(MSBFIRST, SPI_MODE0);
	port->onReceive(_interrupt_routine);
	ssPin = _SS;

	
  	pinMode(INT_PIN,OUTPUT);
  	digitalWrite(INT_PIN,LOW);
}

/*
 uint8_t SPITransfer::sendData(const uint16_t &messageLen, const uint8_t packetID)
 Description:
 ------------
  * Send a specified number of bytes in packetized form
 Inputs:
 -------
  * const uint16_t &messageLen - Number of values in txBuff
  to send as the payload in the next packet
  * const uint8_t packetID - The packet 8-bit identifier
 Return:
 -------
  * uint8_t numBytesIncl - Number of payload bytes included in packet
*/
uint8_t SPITransfer::sendData(const uint16_t& messageLen, const uint8_t packetID)
{
	// TODO: just set flag here, then do the rest in the interrupt routine

	Serial.println("sending data!");

	uint8_t numBytesIncl = packet.constructPacket(messageLen, packetID);

	// digitalWrite(SS, LOW); // Enable SS (active low)
	for (uint8_t i = 0; i < sizeof(packet.preamble); i++)
	{
		// delay(1); // This delay is needed
		// port->transfer(packet.preamble[i]);
		_transfer(packet.preamble[i]);
	}

	for (uint8_t i = 0; i < numBytesIncl; i++)
	{
		// delay(1); // This delay is needed
		// port->transfer(packet.txBuff[i]);
		_transfer(packet.txBuff[i]);
	}

	for (uint8_t i = 0; i < sizeof(packet.postamble); i++)
	{
		// delay(1); // This delay is needed
		// port->transfer(packet.postamble[i]);
		_transfer(packet.postamble[i]);
	}
	// digitalWrite(SS, HIGH); // Disable SS (active low)
	
	// for (uint8_t i = 0; i < 260 - numBytesIncl - sizeof(packet.postamble) - sizeof(packet.preamble); i++)
	// {
	// 	// delay(1); // This delay is needed
	// 	// port->transfer(packet.postamble[i]);
	// 	_transfer(0x00);
	// }

	delay(1);
	ready_2_send_flag = 1;
	digitalWrite(INT_PIN,HIGH);
	return numBytesIncl;
}

/*
 uint8_t SPITransfer::available()
 Description:
 ------------
  * Parses incoming serial data, analyzes packet contents,
  and reports errors/successful packet reception
 Inputs:
 -------
  * void
 Return:
 -------
  * uint8_t bytesRead - Num bytes in RX buffer
*/
uint8_t SPITransfer::available()
{
	volatile uint8_t recChar = SPDR;
	// bytesRead                = packet.parse(recChar);
	status                   = packet.status;

	return 0;
}


uint8_t SPITransfer::finished()
{
	return SPIBuffer.isEmpty() && !ready_2_send_flag;
}

uint8_t SPITransfer::sending()
{
	return sending_flag || ready_2_send_flag;
}

void SPITransfer::_transfer(uint8_t data){
	SPIBuffer.push(data);
	// Serial.print("adding data: "); Serial.println(data);
}

/*
 uint8_t SPITransfer::currentPacketID()
 Description:
 ------------
  * Returns the ID of the last parsed packet
 Inputs:
 -------
  * void
 Return:
 -------
  * uint8_t - ID of the last parsed packet
*/
uint8_t SPITransfer::currentPacketID()
{
	return packet.currentPacketID();
}

// #endif // not (defined(MBED_H) || defined(__SAM3X8E__))




