#include "TrackPacket.h"

//#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

TrackPacket::TrackPacket(uint16_t new_address, 
					 uint8_t new_address_kind) : 
					 	address(new_address), 
						address_kind(new_address_kind), 
						kind(idle_packet_kind), 
						size(1), 
						repeat_count(1),
						priority(false) 
{
  //address = new_address;
  //address_kind = new_address_kind;
  data[0] = 0x00; //default to idle packet
  data[1] = 0x00;
  data[2] = 0x00;
}

uint8_t TrackPacket::getBitstream(uint8_t rawbytes[]) //returns size of array.
{
	int total_size = 1; //minimum size

	if (kind & MULTIFUNCTION_PACKET_KIND_MASK) {
		if (kind == idle_packet_kind) //idle packets work a bit differently:
		// since the "address" field is 0xFF, the logic below will produce C0 FF 00 3F instead of FF 00 FF
		{
			rawbytes[0] = 0xFF;
		} else if (address_kind == DCC_LONG_ADDRESS) //This is a 14-bit address
		{
			rawbytes[0] = (uint8_t)((address >> 8) | 0xC0);
			rawbytes[1] = (uint8_t)(address & 0xFF);
			++total_size;
		} else //we have an 7-bit address
		{
			rawbytes[0] = (uint8_t)(address & 0x7F);
		}

		uint8_t i;
		for (i = 0; i < getSize(); ++i, ++total_size) {
			rawbytes[total_size] = data[i];
		}

		uint8_t XOR = 0;
		for (i = 0; i < total_size; ++i) {
			XOR ^= rawbytes[i];
		}
		rawbytes[total_size] = XOR;

		return total_size + 1;
	} else if (kind & ACCESSORY_PACKET_KIND_MASK) {
		if (kind == basic_accessory_packet_kind) {
			// Basic Accessory Packet looks like this:
			// {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1
			// or this:
			// {preamble} 0 10AAAAAA 0 1AAACDDD 0 (1110CCVV 0 VVVVVVVV 0 DDDDDDDD) 0 EEEEEEEE 1 (if programming)

			rawbytes[0] = 0x80; //set up address byte 0
			rawbytes[1] = 0x88; //set up address byte 1

			rawbytes[0] |= address & 0x03F;
			rawbytes[1] |= (~(address >> 2) & 0x70)
					| (data[0] & 0x07);

			//now, add any programming bytes (skipping first data byte, of course)
			uint8_t i;
			uint8_t total_size = 2;
			for (i = 1; i < getSize(); ++i, ++total_size) {
				rawbytes[total_size] = data[i];
			}

			//and, finally, the XOR
			uint8_t XOR = 0;
			for (i = 0; i < total_size; ++i) {
				XOR ^= rawbytes[i];
			}
			rawbytes[total_size] = XOR;

			return total_size + 1;
		}
	}
	return 0; //ERROR! SHOULD NEVER REACH HERE! do something useful, like transform it into an idle packet or something! TODO
}


void TrackPacket::addData(uint8_t *new_data, uint8_t new_size) //insert freeform data.
{
  for(int i = 0; i < new_size; ++i)
    data[i] = new_data[i];
  size = new_size;
}
