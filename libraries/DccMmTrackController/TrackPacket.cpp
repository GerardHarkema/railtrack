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

TrackPacket::TrackPacket(TRACK_PROTOCOL track_protocol): 
												repeat_count(1),
												priority(false) 
{
	switch(track_protocol){
		case TRACK_PROTOCOL_DCC:
			this->track_protocol = TRACK_PROTOCOL_DCC;
			dcc_data.address = 0;
			dcc_data.address_kind = DCC_SHORT_ADDRESS;
			dcc_data.kind = DCC_IDLE_PACKET_KIND;
			dcc_data.data[0] = 0x00; //default to idle packet
			dcc_data.data[1] = 0x00;
			dcc_data.data[2] = 0x00;
			dcc_data.size  = 1;
			break;
		case TRACK_PROTOCOL_MM:
			this->track_protocol = TRACK_PROTOCOL_MM;
			memset(&mm_data, 0, sizeof(MM2_DATA));
			mm_data.kind = MM2_LOC_SPEED_TELEGRAM;
			break;
		case TRACK_PROTOCOL_UNKNOWN:
		break;
	}
}

uint8_t TrackPacket::dccGetBitstream(uint8_t rawbytes[]) //returns size of array.
{
	int total_size = 1; //minimum size

	if (dcc_data.kind & MULTIFUNCTION_PACKET_KIND_MASK) {
		if (dcc_data.kind == DCC_IDLE_PACKET_KIND) //idle packets work a bit differently:
		// since the "dcc_data.address" field is 0xFF, the logic below will produce C0 FF 00 3F instead of FF 00 FF
		{
			rawbytes[0] = 0xFF;
		} else if (dcc_data.address_kind == DCC_LONG_ADDRESS) //This is a 14-bit dcc_data.address
		{
			rawbytes[0] = (uint8_t)((dcc_data.address >> 8) | 0xC0);
			rawbytes[1] = (uint8_t)(dcc_data.address & 0xFF);
			++total_size;
		} else //we have an 7-bit dcc_data.address
		{
			rawbytes[0] = (uint8_t)(dcc_data.address & 0x7F);
		}

		uint8_t i;
		for (i = 0; i < dccGetSize(); ++i, ++total_size) {
			rawbytes[total_size] = dcc_data.data[i];
		}

		uint8_t XOR = 0;
		for (i = 0; i < total_size; ++i) {
			XOR ^= rawbytes[i];
		}
		rawbytes[total_size] = XOR;

		return total_size + 1;
	} else if (dcc_data.kind & DCC_ACCESSORY_PACKET_KIND_MASK) {
		if (dcc_data.kind == DCC_BASIC_ACCESSORY_PACKET_KIND) {
			// Basic Accessory Packet looks like this:
			// {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1
			// or this:
			// {preamble} 0 10AAAAAA 0 1AAACDDD 0 (1110CCVV 0 VVVVVVVV 0 DDDDDDDD) 0 EEEEEEEE 1 (if programming)

			rawbytes[0] = 0x80; //set up dcc_data.address byte 0
			rawbytes[1] = 0x88; //set up dcc_data.address byte 1

			rawbytes[0] |= dcc_data.address & 0x03F;
			rawbytes[1] |= (~(dcc_data.address >> 2) & 0x70)
					| (dcc_data.data[0] & 0x07);

			//now, add any programming bytes (skipping first dcc_data.data byte, of course)
			uint8_t i;
			uint8_t total_size = 2;
			for (i = 1; i < dccGetSize(); ++i, ++total_size) {
				rawbytes[total_size] = dcc_data.data[i];
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


void TrackPacket::dccAddData(uint8_t *new_data, uint8_t new_size) //insert freeform dcc_data.
{
  for(int i = 0; i < new_size; ++i)
    dcc_data.data[i] = new_data[i];
  dcc_data.size = new_size;
}

uint32_t mm_tribit_lookup[] = {0b00, 	// "0"
													  0b11, 	// "1"
													  0b01}; // "Open"

																	 // 111111110000000000
																	 // 765432109876543210	
uint32_t mm_speed_index_lookup[] = {0b101000100000000000,  //-6..-0
																	  0b001000100000000000,  //-14..-7
																    0b100010000000000000,  //+0..+6
																    0b000010000000000000}; //+7..+14

											 			//  111111110000000000
											 			//  765432109876543210	
#define MM_F1_MASK						0b000010100000000000
#define MM_F2_MASK						0b001000000000000000
#define MM_F3_MASK						0b001010000000000000
#define MM_F4_MASK						0b001010100000000000
#define MM_Fx_MASK_EXEPT_OFF	0b001000100000000000
#define MM_Fx_MASK_EXEPT_ON		0b000010000000000000

void TrackPacket::mm2GetBitstream(uint32_t *bitstream, bool *double_frequency){

	*bitstream = 0;
	uint32_t bitstream_mask;

	// caluclaute address tribits
	int tri_value;
	int div_value = mm_data.address;
	for(int i = 0; i < 4; i++){
		tri_value = div_value%3;
		div_value = div_value/3;
		bitstream_mask = mm_tribit_lookup[tri_value] << (i * 2);
		*bitstream = *bitstream | bitstream_mask;
	}

	uint8_t speed;
	uint8_t speed_mask;
	uint8_t port_mask;
	int8_t operating_level_speed;
	int8_t track_speed;
	bool reverse_direction = false;
	
	switch(mm_data.kind){
		case MM2_LOC_SPEED_TELEGRAM: 
		case MM2_LOC_F1_TELEGRAM: 
		case MM2_LOC_F2_TELEGRAM: 
		case MM2_LOC_F3_TELEGRAM: 
		case MM2_LOC_F4_TELEGRAM: 

			*double_frequency = false;

			// Function tribit
			//bitstream_mask = mm_tribit_lookup[1];
			//*bitstream = *bitstream | bitstream_mask;			

			//Speed bits
			reverse_direction = mm_data.speed < 0 ? true : false;
			operating_level_speed = mm_data.speed < 0 ? -mm_data.speed: mm_data.speed;
			//operating_level_speed = mm_data.speed;
			track_speed = operating_level_speed;
			if(track_speed) track_speed++;
#if 0
			Serial.printf("Speed: %i, reversed direction = %s, operating level speed = %i, track_speed = %i\n", 
										mm_data.speed , reverse_direction ? "true" : "false", operating_level_speed, track_speed);
#endif
			speed_mask = 1;
			for(int i = 0; i < 4; i++){
				if(track_speed & speed_mask){
								          // 111111110000000000
									        // 765432109876543210	
					bitstream_mask = 0b000000010000000000;

//					bitstream_mask = bitstream_mask << (((4-i) * 2) - 2);
					bitstream_mask = bitstream_mask << (i* 2);
					*bitstream = *bitstream | bitstream_mask;
				}
				speed_mask = speed_mask << 1;
			}
			break;
		case MM_SOLENOID_TELEGRAM:
			*double_frequency = true;

			port_mask = 0b1;
											// 111111110000000000
											// 765432109876543210	
			bitstream_mask = 0b000000110000000000; // LSB D0
			for(int i = 0; i < 3; i++){
				if(mm_data.solenoid_sub_address & port_mask)
					*bitstream = *bitstream | bitstream_mask;
				bitstream_mask = bitstream_mask << 2;
				port_mask = port_mask << 1;
			}
											// 111111110000000000
											// 765432109876543210	
			bitstream_mask = 0b110000000000000000; // auxiliary On

			*bitstream = *bitstream | bitstream_mask;	
			break;
	}	
	switch(mm_data.kind){
//		case MM1_LOC_SPEED_TELEGRAM: 
		case MM2_LOC_SPEED_TELEGRAM: 
		case MM2_LOC_F1_TELEGRAM: 
		case MM2_LOC_F2_TELEGRAM: 
		case MM2_LOC_F3_TELEGRAM: 
		case MM2_LOC_F4_TELEGRAM: 
			if(mm_data.auxiliary){
												// 111111110000000000
												// 765432109876543210	
				bitstream_mask = 0b000000001100000000; // auxiliary On

				*bitstream = *bitstream | bitstream_mask;	
			}
		break;
	}
#if 0
	switch(mm_data.kind){
		case MM1_LOC_SPEED_TELEGRAM:
			div_value = mm_data.speed;
			if(div_value == 1) div_value++;
			for(int i = 0; i < 4; i++){
				tri_value = div_value%3;
				div_value = div_value/3;
				// reverse value off address
				//bitstream_mask = mm_tribit_lookup[tri_value] << (((4-i) * 2) + 8);
				bitstream_mask = mm_tribit_lookup[tri_value] << ((i * 2) + 10);
				*bitstream = *bitstream | bitstream_mask;
			}
			break;
	}


	switch(mm_data.kind){
		case MM1_LOC_CHANGE_DIR_TELEGRAM:
											// 111111110000000000
											// 765432109876543210	
			bitstream_mask = 0b000000110000000000; // Change direction

			*bitstream = *bitstream | bitstream_mask;	
			break;
	}

	bitstream_mask = 0;

	switch(mm_data.kind){
		case MM1_LOC_F_TELEGRAM:
			*double_frequency = true;
											// 111111110000000000
											// 765432109876543210	
			bitstream_mask = 0b000000001100000000; 

			*bitstream = *bitstream | bitstream_mask;	

											// 111111110000000000
											// 765432109876543210	
			bitstream_mask = 0b000000110000000000; 
			for(int i = 0; i < 4; i++){
				if(mm_data.function_on_[i]){
					Serial.printf("Index: %i\n", i);
					*bitstream = *bitstream | bitstream_mask;	
				}
				bitstream_mask = bitstream_mask << 2;
			}
			break;
	}
#endif


	switch(mm_data.kind){
		case MM2_LOC_SPEED_TELEGRAM:
			//Function Index
			if(reverse_direction){
				if(operating_level_speed <= 7)
					bitstream_mask = mm_speed_index_lookup[0]; // Operating level: -6..-0
				else
					bitstream_mask = mm_speed_index_lookup[1]; // Operating level: -14..-7\n
			}
			else
			{			
				if(operating_level_speed <= 7)
					bitstream_mask = mm_speed_index_lookup[2]; // Operating level: +0..+6
				else
					bitstream_mask = mm_speed_index_lookup[3]; // Operating level: +7..+14
			}
			break;
		case MM2_LOC_F1_TELEGRAM:
			if((operating_level_speed == 2) && !mm_data.function_on_[0])
			{
					bitstream_mask = MM_Fx_MASK_EXEPT_OFF;
			}
			else if ((operating_level_speed == 10) && mm_data.function_on_[0]){
					bitstream_mask = MM_Fx_MASK_EXEPT_ON;
			}else
				bitstream_mask = MM_F1_MASK;
			if(mm_data.function_on_[0]) bitstream_mask = bitstream_mask | (1 << 17);
			break;
		case MM2_LOC_F2_TELEGRAM: 
			if((operating_level_speed == 3) && !mm_data.function_on_[1])
			{
					bitstream_mask = MM_Fx_MASK_EXEPT_OFF;
			}
			else if ((operating_level_speed == 11) && mm_data.function_on_[1]){
					bitstream_mask = MM_Fx_MASK_EXEPT_ON;
			}else
				bitstream_mask = MM_F2_MASK;
			if(mm_data.function_on_[1]) bitstream_mask = bitstream_mask | (1 << 17);
			break;
		case MM2_LOC_F3_TELEGRAM: 
			if((operating_level_speed == 5) && !mm_data.function_on_[2])
			{
					bitstream_mask = MM_Fx_MASK_EXEPT_OFF;
			}
			else if ((operating_level_speed == 13) && mm_data.function_on_[2]){
					bitstream_mask = MM_Fx_MASK_EXEPT_ON;
			}else
				bitstream_mask = MM_F3_MASK;
			if(mm_data.function_on_[2]) bitstream_mask = bitstream_mask | (1 << 17);
			break;
		case MM2_LOC_F4_TELEGRAM: 
			if((operating_level_speed == 6) && !mm_data.function_on_[3])
			{
					bitstream_mask = MM_Fx_MASK_EXEPT_OFF;
			}
			else if ((operating_level_speed == 14) && mm_data.function_on_[3]){
					bitstream_mask = MM_Fx_MASK_EXEPT_ON;
			}else
				bitstream_mask = MM_F4_MASK;
			if(mm_data.function_on_[3]) bitstream_mask = bitstream_mask | (1 << 17);
			break;
	}
	*bitstream = *bitstream | bitstream_mask;

	uint32_t streem_validate = *bitstream;
	streem_validate = streem_validate >> 10;
	uint8_t bitcount = 0;
	for (int i = 0; i < 4; i++){
		if((streem_validate & 0b11) == 0b01){
			bitcount++;
			//Serial.printf("Error 1, telegram_type = %i\n", mm_data.kind);
		}
		if((streem_validate & 0b11) == 0b10){
			//Serial.printf("Error 2, telegram_type = %i\n", mm_data.kind);
			bitcount++;
			
		}

		streem_validate = streem_validate >> 2;

	}
	if(!bitcount){
		Serial.printf("Telegram error %i\n", mm_data.kind);
		streem_validate = *bitstream;
		streem_validate = streem_validate >> 10;
		for(int i = 0; i < 4; i++){
			Serial.printf("%i", streem_validate & 0b01 ? 1 : 0);
			streem_validate = streem_validate >> 1;
			Serial.printf("%i", streem_validate & 0b01 ? 1 : 0);
			streem_validate = streem_validate >> 1;
			Serial.printf(" ");

		}
			Serial.printf("\n");

	}
}

MM_KIND_TYPE mm2_loc_kind_seqence[] = {MM2_LOC_SPEED_TELEGRAM,
                                       MM2_LOC_F1_TELEGRAM,
                                       MM2_LOC_SPEED_TELEGRAM,
                                       MM2_LOC_F2_TELEGRAM,
                                       MM2_LOC_SPEED_TELEGRAM,
                                       MM2_LOC_F3_TELEGRAM,
                                       MM2_LOC_SPEED_TELEGRAM,
                                       MM2_LOC_F4_TELEGRAM};

int number_of_kind_seqences = sizeof(mm2_loc_kind_seqence)/sizeof(MM_KIND_TYPE);


void TrackPacket::mmSetNextKind(void){

	switch(mm_data.kind ){
		case MM2_LOC_SPEED_TELEGRAM:
    case MM2_LOC_F1_TELEGRAM:
    case MM2_LOC_F2_TELEGRAM:
    case MM2_LOC_F3_TELEGRAM:
    case MM2_LOC_F4_TELEGRAM:
			mm_data.kind = mm2_loc_kind_seqence[mm_data.kind_sequence_index];
			mm_data.kind_sequence_index++;
			if(mm_data.kind_sequence_index == number_of_kind_seqences)mm_data.kind_sequence_index = 0;
		break;
	}

}