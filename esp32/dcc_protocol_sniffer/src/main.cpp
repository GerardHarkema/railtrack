///////////////////////////////////////////////////////
//
// DCC packet analyze: Ruud Boer, October 2015
// DCC packet capture: Robin McKay, March 2014
//
// The DCC signal is detected on Arduino digital pin 2
//
// Set the Serial Monitor Baud Rate to 38400 !!
//
// Keyboard commands that can be sent via Serial Monitor:
// 1 = 1s refresh time
// 2 = 2s 
// 3 = 4s (default)
// 4 = 8s
// 5 = 16s
// 6 = 4 DCC packet buffer
// 7 = 8
// 8 = 16
// 9 = 32 (default)
// 0 = 64
// a = show accessory packets toggle
// l = show locomotive packets toggle
// p = show packet details toggle
//
////////////////////////////////////////////////////////

#include <Arduino.h>

byte refreshTime = 4; // Time between DCC packets buffer refreshes in s
byte packetBufferSize = 32; // DCC packets buffer size

#define TIMER_PRESCALER 64
#define DccBitTimerCount (F_CPU * 80L / TIMER_PRESCALER / 1000000L)
// 16000000 * 80 / 64 / 1000000 = 20; 20 x 4usecs = 80us

boolean packetEnd;
boolean preambleFound;

const byte bitBufSize = 50; // number of slots for bits
volatile byte bitBuffer[bitBufSize]; 
volatile byte bitBuffHead = 1;
volatile byte bitBuffTail = 0;

byte pktByteCount=0;
byte packetBytesCount;
byte preambleOneCount;
byte dccPacket[6]; // buffer to hold a packet
byte instrByte1;
byte decoderType; //0=Loc, 1=Acc
byte bufferCounter=0;
byte isDifferentPacket=0;
byte showLoc=1;
byte showAcc=1;
bool displayPacket = true;


unsigned int decoderAddress;
unsigned int packetBuffer[64];
unsigned int packetNew=0;

unsigned long timeToRefresh = millis() + refreshTime*1000;

#define TRACK_PULSE_INPUT_PIN 2  //for arduino uno

//========================

byte nextBitSlot(byte slot) {
  slot ++;
  if (slot >= bitBufSize) slot = 0;
  return(slot);
}

//========================

byte getBit() {
  // gets the next bit from the bitBuffer
  // if the buffer is empty it will wait indefinitely for bits to arrive
  byte nbs = bitBuffHead;
//  while (nbs == bitBuffHead) byte nbs = nextBitSlot(bitBuffTail); //Buffer empty
  while (nbs == bitBuffHead) nbs = nextBitSlot(bitBuffTail); //Buffer empty
  bitBuffTail = nbs;
  return (bitBuffer[bitBuffTail]);
}

//========================

void getNextByte() {
  byte newByte = 0;
  for (byte n = 0; n < 8; n++) newByte = (newByte << 1) + getBit();
  packetBytesCount ++;  
  dccPacket[packetBytesCount] = newByte;
  dccPacket[0] = packetBytesCount;
  if (getBit() == 1) packetEnd = true;
}

//========================

void checkForPreamble() {
  byte nextBit = getBit();
  if (nextBit == 1) preambleOneCount++;
  if (preambleOneCount < 10 && nextBit == 0) preambleOneCount = 0;
  if (preambleOneCount >= 10 && nextBit == 0) preambleFound = true;
}

//========================

void getPacket() {
  preambleFound = false;
  packetEnd = false;
  packetBytesCount = 0;
  preambleOneCount = 0;
  while (! packetEnd) {
    //Serial.println("y");
    if (preambleFound) getNextByte();
    else checkForPreamble();
  }
}


//========================

void startTimer() {
  OCR0B = TCNT0 + DccBitTimerCount;			//Set counter value of timer 0; as this is the interrupt routine of each rising edge of pin 2 timer is set to DccBitTimerCount
  TIMSK0 |= B00000100;                      //Timer 0 interrupt mask register; set bit 2 that is field OCIE0B. So interrupt is allowed on timer 0 reaching comparator B
  TIFR0  |= B00000100;                      //Timer 0 interrupt flag register; set bit 2 that is field OCF0B.  So interrupt is allowed on timer 0 reaching comparator B
//  Serial.println("Start Timer");
}

//========================

void beginBitDetection() {
  pinMode(TRACK_PULSE_INPUT_PIN, INPUT);
//  attachInterrupt(digitalPinToInterrupt(TRACK_PULSE_INPUT_PIN), mm_isr, CHANGE);

  TCCR0A &= B11111100;						//Timer/Counter 0 Control Register A; clear bits 0 and 1 to 0; leave rest of bits; without changing TCCR0B it looks like setting timer 0 to normal operation
  //attachInterrupt(TRACK_PULSE_INPUT_PIN, startTimer, RISING);   //Older method of assigning Arduino pin 2 to interrupt function. After rising edge of pin 2 timer 0 is started for 80 usec.
  attachInterrupt(0, startTimer, RISING);   //Older method of assigning Arduino pin 2 to interrupt function. After rising edge of pin 2 timer 0 is started for 80 usec.
}


//========================

ISR(TIMER0_COMPB_vect) {                       //If timer 0 reaches comparator B, so in the middle of the first part of a DCC-0 which still has value 1 or in the middle of second part of a DCC-1 which has value 0 an interrupt is arising. 
//  Serial.println(".");
  byte bitFound = ! ((PIND & B00000100) >> 2); //PIND is digital pin 2 on the Arduino Uno; invert as explained above
  TIMSK0 &= B11111011;                         //Timer 0 interrupt mask register; clear bit 2 that is field OCIE0B. The timer will end after another 80 usec; thus it could interrupt in the middle of the second part of a DCC-0. We must wait until next rising edge so disable interrupt of timer.
  byte nbs = nextBitSlot(bitBuffHead);
  if (nbs == bitBuffTail) return;
  else {
    bitBuffHead = nbs;
    bitBuffer[bitBuffHead] = bitFound;
  }
}


void printBinByte(uint8_t byte){
  uint8_t mask = 0x80;
  Serial.print("(");
  Serial.print(byte);
  Serial.print("|0x");
  Serial.print(byte, HEX);
  Serial.print("|b");
  for(int i = 0; i < 8; i++){
    byte & mask ? Serial.print("1") :Serial.print("0");
    mask = mask >> 1;
    if(i == 3)Serial.print(" ");
  }
  Serial.print(")");
}


//========================

void printPacket() {
  if(displayPacket){
    Serial.print(" Packet = ");
    //Serial.print(" pktByteCount = ");
    //Serial.print(pktByteCount);
    Serial.print(" ");
    for (byte n=1; n<pktByteCount; n++) {
        //    Serial.print(" ");
      printBinByte(dccPacket[n]);
      // Serial.print(dccPacket[n],BIN);
    }
  }
  Serial.println(" ");
}

//========================

void refreshBuffer() {
  timeToRefresh = millis() + refreshTime*1000;
  for (byte n=0; n<packetBufferSize; n++) packetBuffer[n]=0;
  bufferCounter=0;
  Serial.println("-");
#if 0
  Serial.print("Loc ");
  Serial.print(showLoc);
  Serial.print(" / Acc ");
  Serial.print(showAcc);
  Serial.print(" / Time ");
  Serial.print(refreshTime);
  Serial.print(" / Buff ");
  Serial.print(packetBufferSize);
  Serial.print(" / Details ");
  Serial.println(displayPacket);
  Serial.println(" ");
#endif

}

void display_menu(){
  Serial.println("Select display function by pressing:");
  Serial.println("  1 = 1s refresh time");
  Serial.println("  2 = 2s"); 
  Serial.println("  3 = 4s (default)");
  Serial.println("  4 = 8s");
  Serial.println("  5 = 16s");
  Serial.println("  6 = 4 DCC packet buffer");
  Serial.println("  7 = 8");
  Serial.println("  8 = 16");
  Serial.println("  9 = 32 (default)");
  Serial.println("  0 = 64");
  Serial.println("  a = show accessory packets toggle");
  Serial.println("  l = show locomotive packets toggle");
  Serial.println("  p = display packet details toggle");
  Serial.print("Loc ");
  Serial.print(showLoc);
  Serial.print(" / Acc ");
  Serial.print(showAcc);
  Serial.print(" / Time ");
  Serial.print(refreshTime);
  Serial.print(" / Buff ");
  Serial.print(packetBufferSize);
  Serial.print(" / Details ");
  Serial.println(displayPacket);
  Serial.println(" ");  

}

void handleMenu(){
  if (Serial.available()) {
    Serial.println(" ");
    char input = Serial.read();
    switch (input) {
      case '1': 
        Serial.println("Refresh Time = 1s");
        refreshTime=1;
      break;
      case '2':
        Serial.println("Refresh Time = 2s");
        refreshTime=2;
      break;
      case '3':
        Serial.println("Refresh Time = 4s");
        refreshTime=4;
      break;
      case '4':
        Serial.println("Refresh Time = 8s");
        refreshTime=8;
      break;
      case '5':
        Serial.println("Refresh Time = 16s");
        refreshTime=16;
      break;
      case '6':
        Serial.println("Buffer Size = 4");
        packetBufferSize=2;
      break;
      case '7':
        Serial.println("Buffer Size = 8");
        packetBufferSize=8;
      break;
      case '8':
        Serial.println("Buffer Size = 16");
        packetBufferSize=16;
      break;
      case '9':
        Serial.println("Buffer Size = 32");
        packetBufferSize=32;
      break;
      case '0':
        Serial.println("Buffer Size = 64");
        packetBufferSize=64;
      break;
      case 'a':
        if (showAcc) showAcc=0; else showAcc=1;
        Serial.print("show loc packets = ");
        Serial.println(showAcc);
      break;
      case 'l':
        if (showLoc) showLoc=0; else showLoc=1;
        Serial.print("show loc packets = ");
        Serial.println(showLoc);
      case 'p':
        displayPacket = displayPacket ? false : true;
        Serial.print("show packets = ");
        Serial.println(displayPacket);
      break;
    }
    display_menu();
    Serial.println(" ");
  }
}

//========================

void setup() {
  Serial.begin(115200); // 38400 when on DCC, 9600 when testing on 123Circuits !!!!!!!!!!!!!!!!!!!!!!!
  

  Serial.println("DCC protocol-sniffer started");
  display_menu();
  Serial.println("---");
  Serial.println("DCC Packet Analyze started");
  Serial.print("Updates every ");
  Serial.print(refreshTime);
  Serial.println(" seconds");
  Serial.println("---");

  beginBitDetection(); //Uncomment this line when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

//====================
void loop() {

  handleMenu();


  getPacket(); //Uncomment this line when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  byte speed;
  byte checksum = 0;
  
  if (millis() > timeToRefresh) refreshBuffer();
  
/* Dummy packet for test purposes. Comment when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Loc 1782 CV Write 3 128
dccPacket[0]=6;
dccPacket[1]=B11000111;
dccPacket[2]=B01101110;
dccPacket[3]=B11101100;
dccPacket[4]=B00000011;
dccPacket[5]=B10000000;
dccPacket[6]=B11111111;
*/  
  pktByteCount = dccPacket[0];
  if (!pktByteCount) return; // No new packet available

  for (byte n = 1; n <= pktByteCount; n++) checksum ^= dccPacket[n];
  //checksum=0; //Comment this line when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (checksum) return; // Invalid Checksum
  
  else { // There is a new packet with a correct checksum
    isDifferentPacket=1;
    for (byte n=0; n<packetBufferSize ; n++) {// Check if packet is not already in the buffer. 
    // The checksum byte is used for the test, not ideal, some new packets may not show (only 256 different checksums)
      if (dccPacket[pktByteCount]==packetBuffer[n]) isDifferentPacket=0; 
    }

    if (isDifferentPacket) {  // packet does not yet exist in the packet buffer
      packetBuffer[bufferCounter] = dccPacket[pktByteCount]; // add new packet to the buffer
      bufferCounter = (++bufferCounter)&(packetBufferSize-1);
      
      if (dccPacket[1]==B11111111) { //Idle packet
        Serial.println("Idle ");
        return;
      }
    
      if (!bitRead(dccPacket[1],7)) { //bit7=0 -> Loc Decoder Short Address
        decoderAddress = dccPacket[1];
        instrByte1 = dccPacket[2];
        decoderType = 0;
      }
      else {
        if (bitRead(dccPacket[1],6)) { //bit7=1 AND bit6=1 -> Loc Decoder Long Address
          decoderAddress = 256 * (dccPacket[1] & B00000111) + dccPacket[2];
          instrByte1 = dccPacket[3];
          decoderType = 0;
        }
        else { //bit7=1 AND bit6=0 -> Accessory Decoder
          decoderAddress = dccPacket[1]&B00111111;
          instrByte1 = dccPacket[2];
          decoderType = 1;
        }
      }
      if (decoderType) { // Accessory Basic
        if (showAcc) {
          if (instrByte1&B10000000) { // Basic Accessory
            decoderAddress = (((~instrByte1)&B01110000)<<2) + decoderAddress;
            byte port = (instrByte1&B00000110)>>1;
            Serial.print("Acc ");
            Serial.print((decoderAddress-1)*4 + port + 1);
            Serial.print(" ");
            Serial.print(decoderAddress);
            Serial.print(":");
            Serial.print(port);
            Serial.print(" -- ");
            Serial.print(bitRead(instrByte1,3));
            if (bitRead(instrByte1,0)) Serial.print(" On");
            else Serial.print(" Off");
          }
          else { // Accessory Extended NMRA spec is not clear about address and instruction format !!!
            Serial.print("Acc Ext ");
            decoderAddress = (decoderAddress<<5) + ((instrByte1&B01110000)>>2) + ((instrByte1&B00000110)>>1);
            Serial.print(decoderAddress);
            Serial.print(" Asp ");
            Serial.print(dccPacket[3],BIN);
            printBinByte(dccPacket[3]);
          }
          printPacket();
        }
      }
      else { // Loc / Multi Function Decoder
        if (showLoc) {
          Serial.print("Loc ");
          Serial.print(decoderAddress);
          byte instructionType = instrByte1>>5;
          switch (instructionType) {

            case 0:
              Serial.print(" Control ");
            break;

            case 1: // Advanced Operations
              if (instrByte1==B00111111) { //128 speed steps
                if (bitRead(dccPacket[pktByteCount-1],7)) Serial.print(" Forw128 ");
                else Serial.print(" Rev128 ");
                byte speed = dccPacket[pktByteCount-1]&B01111111;
                if (!speed) Serial.print(" Stop ");
                else if (speed==1) Serial.print(" E-stop ");
                else Serial.print(speed-1);
              }
              else if (instrByte1==B00111110) { //Speed Restriction
              if (bitRead(dccPacket[pktByteCount-1],7)) Serial.print(" On ");
                else Serial.print(" Off ");
                Serial.print(dccPacket[pktByteCount-1])&B01111111;
              }
            break;

            case 2: // Reverse speed step
              speed = ((instrByte1&B00001111)<<1) - 3 + bitRead(instrByte1,4);
              if (speed==253 || speed==254) Serial.print(" Stop ");
              else if (speed==255 || speed==0) Serial.print(" E-Stop ");
              else {
                Serial.print(" Rev ");
                Serial.print(speed);
              }
            break;

            case 3: // Forward speed step
              speed = ((instrByte1&B00001111)<<1) - 3 + bitRead(instrByte1,4);
              if (speed==253 || speed==254) Serial.print(" Stop ");
              else if (speed==255 || speed==0) Serial.print(" E-Stop ");
              else {
                Serial.print(" Forw ");
                Serial.print(speed);
              }
            break;

            case 4: // Loc Function L-4-3-2-1
              Serial.print(" L F4-F1 ");
              //Serial.print(instrByte1&B00011111,BIN);
              printBinByte(instrByte1&B00011111);
            break;

            case 5: // Loc Function 8-7-6-5
              if (bitRead(instrByte1,4)) {
                Serial.print(" F8-F5 ");
                //Serial.print(instrByte1&B00001111,BIN);
                printBinByte(instrByte1&B00011111);
              }
              else { // Loc Function 12-11-10-9
                Serial.print(" F12-F9 ");
                //Serial.print(instrByte1&B00001111,BIN);
                printBinByte(instrByte1&B00011111);
              }
            break;

            case 6: // Future Expansions
              switch (instrByte1&B00011111) {
                case 0: // Binary State Control Instruction long form
                  Serial.print(" BinStateLong ");
                  Serial.print(256 * dccPacket[pktByteCount-1] + (dccPacket[pktByteCount-2]&B01111111));
                  if bitRead(dccPacket[pktByteCount-2],7) Serial.print(" On ");
                  else Serial.print(" Off ");
                break;
                case B00011101: // Binary State Control
                  Serial.print(" BinStateShort ");
                  Serial.print(dccPacket[pktByteCount-1]&B01111111);
                  if bitRead(dccPacket[pktByteCount-1],7) Serial.print(" On ");
                  else Serial.print(" Off ");
                break;
                case B00011110: // F13-F20 Function Control
                  Serial.print(" F20-F13 ");
                  //Serial.print(dccPacket[pktByteCount-1],BIN);
                  printBinByte(dccPacket[pktByteCount-1]);
                break;
                case B00011111: // F21-F28 Function Control
                  Serial.print(" F28-F21 ");
                  //Serial.print(dccPacket[pktByteCount-1],BIN);
                  printBinByte(dccPacket[pktByteCount-1]);
                break;
              }
            break;

            case 7:
              Serial.print(" CV ");
              if (instrByte1&B00010000) { // CV Short Form
                byte cvType=instrByte1&B00001111;
                switch (cvType) {
                  case B00000010:
                    Serial.print("23 ");
                    Serial.print(dccPacket[pktByteCount-1]);
                  break;
                  case B00000011:
                    Serial.print("24 ");
                    Serial.print(dccPacket[pktByteCount-1]);
                  break;
                  case B00001001:
                    Serial.print("Decoder Lock ");
                    Serial.print(dccPacket[pktByteCount-1]);
                  break;
                }
              }
              else { // CV Long Form
                int cvAddress = 256 * (instrByte1&B00000011) + dccPacket[pktByteCount-2] + 1;
                Serial.print(cvAddress);
                Serial.print(" ");
                switch (instrByte1&B00001100) {
                  case B00000100: // Verify Byte
                    Serial.print("Verify ");
                    Serial.print(dccPacket[pktByteCount-1]);
                  break;
                  case B00001100: // Write Byte
                    Serial.print("Write ");
                    Serial.print(dccPacket[pktByteCount-1]);
                  break;
                  case B00001000: // Bit Write
                    Serial.print("Bit ");
                    if (dccPacket[pktByteCount-2]&B00010000) Serial.print("Verify ");
                    else Serial.print("Write ");
                    Serial.print(dccPacket[pktByteCount-1]&B00000111);
                    Serial.print(" ");
                    Serial.print((dccPacket[pktByteCount-1]&B00001000)>>3);
                  break;
                }
              }
            break;
          }
          printPacket();
        }
      }
    }
  }

}

//=====================
