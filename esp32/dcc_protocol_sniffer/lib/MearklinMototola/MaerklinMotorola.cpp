/*
  MaerklinMotorola.cpp - Library for decoding the signals from the Märklin-Motorola-protocol. 
  Created by Laserlicht, Februar 27, 2018.
  Released under BSD 2-Clause "Simplified" License.
*/

#include <MaerklinMotorola.h>

MaerklinMotorola::MaerklinMotorola(int p) {
  pin = p;
  DataQueueWritePosition = 0;
  sync = false;
}

MaerklinMotorolaData* MaerklinMotorola::GetData() {
	for(int QueuePos=0; QueuePos<MM_QUEUE_LENGTH;QueuePos++) {
		if(DataGramState_Validated == DataQueue[QueuePos].State) {
			DataQueue[QueuePos].State = DataGramState_Finished;
			return DataQueue + QueuePos;
		}
	}
	return 0;
}

void MaerklinMotorola::Parse() {
	for(int QueuePos=0; QueuePos<MM_QUEUE_LENGTH;QueuePos++) {
		if(DataGramState_ReadyToParse == DataQueue[QueuePos].State) {
		  int period = DataQueue[QueuePos].Timings[0]+DataQueue[QueuePos].Timings[1]; //calculate bit length
		  bool valid = true;
		  bool parsed = false;

		  DataQueue[QueuePos].IsMagnet = false;
		  DataQueue[QueuePos].IsMM2 = false;
		  DataQueue[QueuePos].Address = 0;
		  DataQueue[QueuePos].SubAddress = 0;
		  DataQueue[QueuePos].Function = false;
		  DataQueue[QueuePos].Stop = false;
		  DataQueue[QueuePos].ChangeDir = false;
		  DataQueue[QueuePos].Speed = 0;
		  DataQueue[QueuePos].MagnetState = 0;
		  DataQueue[QueuePos].IsMM2FunctionOn = false;
		  DataQueue[QueuePos].MM2FunctionIndex = 0;
		  DataQueue[QueuePos].MM2Direction = MM2DirectionState_Unavailable;
		  DataQueue[QueuePos].DecoderState = MM2DecoderState_Unavailable;
		  DataQueue[QueuePos].PortAddress = 0;
		  
			byte BitStream[18];
			//DataxQueue[QueuePos].BitStream
		  
		  for(unsigned char i=0;i<35;i+=2) { //decode bits
				BitStream[i/2] = (DataQueue[QueuePos].Timings[i]>(period>>1)) ? 1 : 0; //longer than half: 1

				if(i<33) {
					int period_tmp = DataQueue[QueuePos].Timings[i] + DataQueue[QueuePos].Timings[i+1];
					if(period_tmp > 125 && period_tmp < 175) valid = false; //MFX herausfiltern
				}
		  }

#if (defined INCLUDE_BITSTREAM_DISPLAY)
 			uint32_t mask = 0b1;
			DataQueue[QueuePos].BitStream = 0;
			for(int i = 0; i < 18; i++){
				if(BitStream[i]){
					DataQueue[QueuePos].BitStream = DataQueue[QueuePos].BitStream | mask;
				}
				mask = mask << 1;
			}
#endif
		  //The first 5 "trits" are always ternary (MM1 and MM2) - For MM2, the least 4 "trits" are quarternar

		  for(unsigned char i=0;i<9;i++) { //decode trits from bits
				if(BitStream[i*2] == 0 && BitStream[i*2+1] == 0)
				{
					//00
					DataQueue[QueuePos].Trits[i] = 0;
				}
				else if(BitStream[i*2] == 1 && BitStream[i*2+1] == 1)
				{
					//11
					DataQueue[QueuePos].Trits[i] = 1;
				}
				else if(BitStream[i*2] == 1 && BitStream[i*2+1] == 0)
				{
					//10
					DataQueue[QueuePos].Trits[i] = 2;
					if(i>=5)
					{
						//MM1 trailing "trits" only use "11" and "00" so we have MM2 here
						DataQueue[QueuePos].IsMM2 = true;
					}
				}
				else
				{
					//01 -> MM2 only and only for trits 5...9
					if(i<5)
					{
						//Pattern 01 can't occur on trits 0...4 -> invalid input
						valid = false;
						break;
					}
					else
					{
						//MM1 trailing "trits" only use "11" and "00" so we have MM2 here
						DataQueue[QueuePos].Trits[i] = 3;	
						DataQueue[QueuePos].IsMM2 = true;
					}
				}
		  }

		  //Decoder
		  if(DataQueue[QueuePos].tm_package_delta > 1300 && DataQueue[QueuePos].tm_package_delta < 4200 && valid) { //protocol-specific telegram length: turnouts or locomotive protocol
			DataQueue[QueuePos].IsMagnet = ((period < 150) ? true : false);  //distinction protocol (fixed-time)
			
			DataQueue[QueuePos].Address = DataQueue[QueuePos].Trits[3] * 27 + DataQueue[QueuePos].Trits[2] * 9 + DataQueue[QueuePos].Trits[1] * 3 + DataQueue[QueuePos].Trits[0];

			if(!DataQueue[QueuePos].IsMagnet) { //Loktelegramm
			  DataQueue[QueuePos].Function = (DataQueue[QueuePos].Trits[4] == 1) ? true : false;

			  unsigned char s = BitStream[10] + BitStream[12] * 2 + BitStream[14] * 4 + BitStream[16] * 8;
			  DataQueue[QueuePos].Stop = (s==0) ? true : false;
			  DataQueue[QueuePos].ChangeDir = (s==1) ? true : false;
			  DataQueue[QueuePos].Speed = (s==0) ?  0 : s-1;
			  if(DataQueue[QueuePos].IsMM2)
			  {
				//convert MM2 bits to one number
				unsigned char sMM2 = BitStream[17] + BitStream[15] * 2 + BitStream[13] * 4 + BitStream[11] * 8;
				DataQueue[QueuePos].IsSpeed = false; 
				switch(sMM2)
				{

					case 2:
					case 3:
					DataQueue[QueuePos].MM2FunctionIndex = 2;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;

					case 4:
					DataQueue[QueuePos].MM2Direction = MM2DirectionState_Forward_H;
					DataQueue[QueuePos].IsSpeed = true;
					break;
					case 5:
					DataQueue[QueuePos].MM2Direction = MM2DirectionState_Forward_L;
					DataQueue[QueuePos].IsSpeed = true;
					break;

					case 6:
					case 7:
					DataQueue[QueuePos].MM2FunctionIndex = 3;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;

					case 10:
					DataQueue[QueuePos].MM2Direction = MM2DirectionState_Backward_H;
					DataQueue[QueuePos].IsSpeed = true;
					break;
					case 11:
					DataQueue[QueuePos].MM2Direction = MM2DirectionState_Backward_L;
					DataQueue[QueuePos].IsSpeed = true;
					break;

					case 12:
					case 13:
					DataQueue[QueuePos].MM2FunctionIndex = 1;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;

					case 14:
					case 15:
					DataQueue[QueuePos].MM2FunctionIndex = 4;
					DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					break;
					
					default:
					break;
				}
			  }

			  parsed=true;
			} else { //magnet telegram
			  if(DataQueue[QueuePos].Trits[4]==0) {
					DataQueue[QueuePos].IsAdditionalFunction = false;
					unsigned char s = BitStream[10] + BitStream[12] * 2 + BitStream[14] * 4;
					DataQueue[QueuePos].SubAddress = s;				
					DataQueue[QueuePos].PortAddress = (( DataQueue[QueuePos].Address - 1) * 4) + (s >> 1) + 1;
					if (BitStream[16]==1) {
						DataQueue[QueuePos].MagnetState = true;
						DataQueue[QueuePos].DecoderState = BitStream[10] ? MM2DecoderState_Green : MM2DecoderState_Red;				    
					}
			  }
				else{
					DataQueue[QueuePos].IsAdditionalFunction = true;
					DataQueue[QueuePos].IsMM1FunctionOn = BitStream[10] + BitStream[12] * 2 + BitStream[14] * 4  + BitStream[16] * 8;
				}
				parsed=true;
			}  
		  }	
		  if(parsed) {
			  //Get previous DataGram from Queue
			  int previousDataGramPos = QueuePos > 0 ? QueuePos - 1 : MM_QUEUE_LENGTH - 1;
			  DataQueue[QueuePos].State = DataGramState_Parsed;
			  if(DataGramState_Parsed == DataQueue[previousDataGramPos].State) {
				  //Check if previous DataGram was identical
				  if(0 == memcmp(DataQueue[QueuePos].Trits, DataQueue[previousDataGramPos].Trits, 9)) {
					  DataQueue[QueuePos].State = DataGramState_Validated;
				  }
			  }
		  }
		  else {
			  //Invalid frame
			  DataQueue[QueuePos].State = DataGramState_Error;
		  }
		}
	}
}

void MaerklinMotorola::PinChange() {
  unsigned long tm = micros();
  unsigned long tm_delta = tm - last_tm;

  if(sync) { //collect bits only after syncronization
    DataQueue[DataQueueWritePosition].Timings[timings_pos] = int(tm_delta); //filing the time difference between the last edges
    timings_pos++;

	if(tm_delta>500) {
		//timeout - resync
		timings_pos = 0;
		sync = true;
		sync_tm = tm;
	}
    if(timings_pos==35) {
      DataQueue[DataQueueWritePosition].tm_package_delta = tm - sync_tm; //calculate package length
	  DataQueue[DataQueueWritePosition].State = DataGramState_ReadyToParse;
	  DataQueueWritePosition ++;
	  //Queue end - go to queue start
	  if(MM_QUEUE_LENGTH <= DataQueueWritePosition)
	  {
		 DataQueueWritePosition = 0;
	  }
	  
	  DataQueue[DataQueueWritePosition].State = DataGramState_Reading;
      sync = false;
      timings_pos = 0;
    }
  } else {
    if(tm_delta>500) { //protocol-specific pause time
      sync = true;
      sync_tm = tm;
    }
  }

  last_tm = tm;
}
