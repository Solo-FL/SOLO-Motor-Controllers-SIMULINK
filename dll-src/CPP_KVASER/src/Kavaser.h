#pragma once

#include "canlib.h"
#include <stdint.h>

class Kavaser
{
public:
	Kavaser();
	bool CANOpenTransmit(int hnd, uint8_t _address, uint16_t _object, uint8_t* _informatrionToSend, int& error);
	bool CANOpenReceive(int hnd, uint8_t _address, uint16_t _object, uint8_t* _informatrionToSend, uint8_t* _informationReceived, int& error);
	bool CANOpenGenericTransmit(int hnd, uint16_t _ID , uint8_t *_DLC, uint8_t* _Data, int& error);
	bool CANOpenGenericReceive(int hnd, uint16_t *_ID , uint8_t *_DLC, uint8_t* _Data);
};
