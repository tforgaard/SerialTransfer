//#include "Arduino.h"

// #if not(defined(MBED_H) || defined(__SAM3X8E__) || defined(DISABLE_SPI_SERIALTRANSFER)) // These boards are/will not be supported by SPITransfer.h

#include "SPITransfer.h"


//TODO: move interrupt routine into here again
// define circularbuffer here as well
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

void SPITransfer::begin(CircularBuffer<unsigned char, 255>* _buff, const configST configs, const uint8_t& _SS)
{
	buff = _buff;
	//SPISlave_T4& _port
	// port = &_port;
	// port = SPISlave_T4<&_port,SPI_8_BITS>();
	packet.begin(configs);
	// port.onReceive(_interrupt_routine);
  	// port.begin();
	ssPin = _SS;
	// memset(SPIBuffer, 0, sizeof(SPIBuffer));// clear the buffer
    // buffer_index = 0;// reset count
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
void SPITransfer::begin(CircularBuffer<unsigned char, 255>* _buff, const uint8_t& _SS, const bool _debug, Stream& _debugPort)
{
	buff = _buff;
	// port = SPISlave_T4<&_port,SPI_BITS::SPI_8_BITS>();
	// port = &_port;
	packet.begin(_debug, _debugPort);
	ssPin = _SS;
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

	uint8_t numBytesIncl = packet.constructPacket(messageLen, packetID);

	// digitalWrite(SS, LOW); // Enable SS (active low)
	for (uint8_t i = 0; i < sizeof(packet.preamble); i++)
	{
		delay(1); // This delay is needed
		// port.transfer(packet.preamble[i]);
		_transfer(packet.preamble[i]);
	}

	for (uint8_t i = 0; i < numBytesIncl; i++)
	{
		delay(1); // This delay is needed
		// port.transfer(packet.txBuff[i]);
		_transfer(packet.txBuff[i]);
	}

	for (uint8_t i = 0; i < sizeof(packet.postamble); i++)
	{
		delay(1); // This delay is needed
		// port.transfer(packet.postamble[i]);
		_transfer(packet.postamble[i]);
	}
	// digitalWrite(SS, HIGH); // Disable SS (active low)

	ready_2_send_flag = 1;
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
	return buff->isEmpty() && !ready_2_send_flag;
}

void SPITransfer::_transfer(uint8_t data){
	// SPIBuffer[buffer_index] = data;
	// buffer_index++;
	buff->push(data);
	Serial.print("adding data: "); Serial.println(data);
	// Serial.print("buffer index: "); Serial.println(buffer_index);
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

