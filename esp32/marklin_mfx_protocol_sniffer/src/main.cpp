// This is a program to capture data from the Marklin mfx signal stream 
// The mfx signal is a rectangular waveform that switches between nominally plus and minus 18 volts
// The mfx information is encoded in the time that elapses between changes in level - whether the change is from +ve to -ve or -ve to +ve and the actual voltage is largely irrelevant
// What matters is the time the elapses between changes in level - with 50uS (20kHz) being the basis of MOST timing activity
// A single 50uS pulse by itself should only be found in a synchronising signal, two consecutive 50uS pulses are a logic 1, a 100uS pulse is a logic zero
// There are also various pauses in the transmitted signal to allow the decoders time to respond to some commands or provide feedback to the controller
// At this time 5 different raw pulse types are collected on the fly. They are then processed into a more meaningful stream and grouped into "frames" that perform a particular function
// The raw data is collected and identified by the following single character symbols (characters (8 bits) used to maximise readability and data storage use over larger (16 or 32 bit) data types)
// They are all determined from pulse widths - the micros() counter is used to measure between pulse edges
// W = Wait >=1mS - possible large gap left for substantial decoder feedback to controller 
// P = Pause 116us to 1mS - numerous medium length pauses occur in the mfx transmission
// 0 = Logic 0 - nominal 100uS - from 74 to 116uS accepted as a logic 0
// S = Short Pulse - nominal 50uS - 36 to 74uS accepted. Either PART OF a sync sequence that marks the beginning and end of a data frame OR if two consecutive sync pulses occur they represent a logic 1 (WTF)
// * = very short pulses - less than 36uS are not fully understood but looking like part of the feedback signal from a decoder to the controller.
// After processing the S pulses should all resolve into either logic 1's or F's (part of a frame start or end (sync) sequence) - any strays seem to indicate corruption of the mfx signal
// So we end up with a spread of different pulse types - W,P,0,1,S,F,* - which we try to decode
// revmox 16-11-24


#include <Arduino.h>
#include <stdio.h>

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define maxbits 2048                                        //the number of data transitions to collect before analysis - in practice this is roughly 9000 per second of recording time


#define TRACK_PULSE_INPUT_PIN 2  //for arduino uno
                
#if 0                                                           // LCD sheild pin allocations
#define LCD_CS A3                                           // Chip Select
#define LCD_RST A4                                          // LCD harware reset
#define LCD_CD A2                                           // Command/Data
#define LCD_DC A2                                           // Command/Data
#define LCD_WR A1                                           // LCD Write Clock
#define LCD_RD A0                                           // LCD Read Clock
#endif

#define TFT_CS 22
#define TFT_RST 24
#define TFT_DC 26
#define TFT_MOSI 28
#define TFT_CLK 30
#define TFT_MISO 32

//#define A15     1

#define	BLACK   0x0000                                      // Some predefined colours to use
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0xA514
#define ORANGE  0xFC00

int lokADD=0;                                               // lok address extracted from mfx frame
int lokDIR=0;                                               // lok direction extracted from mfx frame
int lokSPD=0;                                               // lok speed extracted from mfx frame
int ID[16];                                                 // array of registration numbers of the loks polled by the controller
int direction[16];                                          // array of directions set for the loks 0=FWD, 1=REV
int speed[16];                                              // array of a lok's currently set speed step
int lokX[16];                                               // x coords of where to place the lok number in the table cells
int lokY[16];                                               // y coords of where to place the lok number in the table cells
int dirX[16];                                               // x coords of where to place the direction in the table cells
int dirY[16];                                               // y coords of where to place the direction in the table cells
int spdX[16];                                               // x coords of where to place the speed in the table cells
int spdY[16];                                               // y coords of where to place the speed in the table cells
int offset=511;                                             // set for a sensor value of zero with input leads shorted - theoretical 511
int scale=21;                                               // set to show correct input voltage - theoretical 21.7 but int 21 gives good enough results
                                                            // with changeover near the half volt levels i.e. 17.5, 18.5, etc.
int val=0;

volatile unsigned long bitcount=0;                          //running tally of the number of transitions collected
volatile unsigned long duration;                            //values used to calculate the time in uS between transitions
volatile unsigned long previous;
volatile unsigned long elapsed;
volatile char data1[maxbits];                               //where the raw sampled data is stored
int start[maxbits/8];                                       //where the start of a frame (sync pulse) is located in the sampled data
int end[maxbits/8];                                         //where the end of a frame (sync pulse) is located in the sampled data - paired with the above array to define an mfx frame
int length[maxbits/8];                                      //the length of a frame
int newmax=0;                                               //as the sampled data is processed there is some compression - this keeps a tab on the total data size through the various operations
int linecount=0;                                            //used to keep the screen printout tidy when displaying data - causes newlines
int numframes=0;                                            //the number of frames extracted from the collected data
char x;                                                     //needed for the users input pauses
volatile bool sync=false;                                   //used in the detection of the initial sync pulse that will trigger recording
String frame;                                               //globally available string created from each frame to allow parsing and analysis
unsigned int crc;
int IRQpin=TRACK_PULSE_INPUT_PIN;//19;                                              //the input pin detecting the mfx signal transitions from the track
                                                            //pin 2 works for all the smaller boards but clashes with most display shields
int IRQflag=0;                                              //used when testing whether there is any signal activity on the track

void collect()                                                          // tentative pulse classifications until mfx is understood much better
{ 
  duration=micros()-previous;                                           // calculate time elapsed since last interrupt
  previous=micros();                                                    // store the current time in order to use when the next interrupt occurs
  data1[bitcount]='W';                                                  // default starting of 1mS and up classed as a wait
  if(duration<1000)data1[bitcount]='P';                                 // from 116uS to 1mS classed as a pause
  if(duration<116)data1[bitcount]='0';                                  // from 72uS to 116uS classed as a logic 0
  if(duration<72)data1[bitcount]='S';                                   // from 36uS to 72uS classed as a short pulse - 1 single pulse probably part of a sync pulse, two consecutive pulses are a logic 1
  if(duration<36)data1[bitcount]='*';                                   // less than 36uS not understood - looks like part of the feedback mechanism from decoder back to controller
  bitcount++;                                                           // increment the bit counter
}

void IRQtest()
{
  if(IRQflag<2048)IRQflag++;                                            //bump the IRQflag - stop overflows
}

void userpause()                                                        // if developer flag defined - wait for user input
{
#if (defined DEVELOPMENT)
  {
  Serial.println("PRESS ENTER TO CONTINUE");
  while(Serial.available() == 0) {}
  x = Serial.read();
  }
#endif
}

void displayData()                                                      // if developer flag defined - display data as it is processed at each step
{
#if (defined DEVELOPMENT)
  {   
  linecount=0;
  for(int c=0;c<newmax;c++)
  {
    Serial.print(data1[c]);
    linecount++;
    if(linecount>=100)
      {Serial.println();linecount=0;}
  }
  Serial.println();
  }
#endif
}

void bintoDEC(int first,int length)
{
  String temp=frame.substring(first,first+length);
  val=0;
  for(int x=0;x<length;x++)
  if(temp.charAt(x)=='1')val=val+(1<<length-x-1);
}

void decode(int offset)                                     // offset points to the place in the data stream after the end of the address (which can be of varying length)
{
  bool cmdval=false;
  String command=frame.substring(offset,offset+3);          // extract the 3 bit mfx command from the frame
  if(command=="000")                                        // drive control short format just FWD/REV and 8 steps - possibly used for parked loks to cut down on frame length?
    {
    cmdval=true;
    //Serial.print("Drive(S)"); 
    if(frame.charAt(offset+3)=='0')lokDIR=0;                // single bit direction control
    else lokDIR=1;
    bintoDEC(offset+4,3);                                   // 3 bits giving 8 speed steps
    lokSPD=val*16;
    }

  if(command=="001")                                        // drive control normal long format FWD/REV and 128 steps
    {
    cmdval=true;       
    //Serial.print("Drive(L)");     
    if(frame.charAt(offset+3)=='0')lokDIR=0;                // single bit direction control
    else lokDIR=1;
    bintoDEC(offset+4,7);                                   // 7 bits giving 128 speed steps
    lokSPD=val;
    }                                                       //here we have a new loco sample to process
#if (defined DEVELOPMENT)
      Serial.print("Lok address = ");Serial.println(lokADD);
      Serial.print("Lok direction = ");Serial.println(lokDIR);
      Serial.print("Lok speed = ");Serial.println(lokSPD);
#endif
      //is the lok already in the table?
      bool found=false;
      bool entered=false;
      int location=0;
      for(int c=0;c<16;c++)                                 //yes it's already there - just update the data
        if(ID[c]==lokADD)
          {
            direction[c]=lokDIR;speed[c]=lokSPD;
            found=true;
          }
      for(int c=0;c<16;c++)                                 //no it's not there - put it in the first empty slot
        if((found==false)&&(entered==false)&&(ID[c]==0))
          {
            ID[c]=lokADD;direction[c]=lokDIR;speed[c]=lokSPD;entered=true;
          }
}

Adafruit_ILI9341 *tft;// = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void LCDsetup()                                             // start the LCD display and draw the background text and grid
{

  tft = new Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
  tft->begin();

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft->readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft->readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft->readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft->readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft->readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 
                                       //initialise the LCD for operation
  tft->setRotation(1);                                     //set portrait display orientation
  tft->fillScreen(BLACK);                                  //clear screen to black
  tft->setCursor(95,0);                                    //print screen title
  tft->setTextColor(GREEN);
  tft->setTextSize(2);
  tft->println("MFX SNIFFER");
  tft->setTextColor(WHITE);
  for(int c=0;c<320;c+=106)tft->drawLine(c,20,c,41,YELLOW);    //draw data grid vertical lines
  for(int c=0;c<320;c+=53)tft->drawLine(c,63,c,239,GREY);
  for(int c=0;c<320;c+=53)tft->drawLine(c,41,c,63,YELLOW);
  for(int c=0;c<320;c+=159)tft->drawLine(c,41,c,239,YELLOW);
  for(int c=19;c<240;c+=22)tft->drawLine(0,c,318,c,GREY);      //draw data grid horizontal lines
  for(int c=19;c<66;c+=22)tft->drawLine(0,c,318,c,YELLOW);
  tft->drawLine(0,239,318,239,YELLOW);
  //calculate all the positions to place text just once and stick them in arrays
  for(int c=0;c<8;c++)                                    //lok x positions                                   
    {lokX[c]=10;lokX[c+8]=169;}
  for(int c=0;c<16;c++)                                   //dir x positions                                                                    
    {dirX[c]=lokX[c]+53;}
  for(int c=0;c<16;c++)                                   //spd x positions                                                                   
    {spdX[c]=lokX[c]+106;}                
  for(int c=0;c<8;c++)                                    //lok y positions
    {lokY[c]=68+c*22;lokY[c+8]=lokY[c];}
  for(int c=0;c<16;c++)                                   //dir y positions
    {dirY[c]=lokY[c];}
  for(int c=0;c<16;c++)                                   //spd y positions
    {spdY[c]=lokY[c];}
  tft->setCursor(11,46);tft->print("LOK");                  //print LOK column titles
  tft->setCursor(170,46);tft->print("LOK");
  tft->setCursor(64,46);tft->print("DIR");                  //print DIR column titles
  tft->setCursor(223,46);tft->print("DIR");
  tft->setCursor(117,46);tft->print("SPD");                 //print SPD column titles
  tft->setCursor(276,46);tft->print("SPD");
  }

void LCDupdate()                                            // display update - takes 267mS on Mega2560, 33mS on Due
{
#if 1
  for(int c=0;c<16;c++)
    if(ID[c])
      {
      int fix=0;                                                        //print and centre justify ID (0-999)
      if(ID[c]<100)fix=fix+6;if(ID[c]<10)fix=fix+6;
      {tft->setCursor((lokX[c]+fix),lokY[c]);
      tft->fillRect(lokX[c],lokY[c],36,14,BLACK);tft->print(ID[c]);} 
      if(!direction[c])
        {tft->setCursor(dirX[c],dirY[c]);                                //print FWD or REV
        tft->fillRect(dirX[c],dirY[c],36,14,BLACK);tft->print("FWD");}
      else
        {tft->setCursor(dirX[c],dirY[c]);
        tft->fillRect(dirX[c],dirY[c],36,14,BLACK);tft->print("REV");}
      fix=0;                                                            //print and centre justify lok speed (0-999)
      if(speed[c]<100)fix=fix+6;if(speed[c]<10)fix=fix+6;
      {tft->setCursor((spdX[c]+fix),spdY[c]);
      tft->fillRect(spdX[c],spdY[c],36,14,BLACK);tft->print(speed[c]);}     
      }
    else
      {
      tft->fillRect(lokX[c],lokY[c],36,14,BLACK);                         //ensure any unused cells are blanked
      tft->fillRect(dirX[c],dirY[c],36,14,BLACK);
      tft->fillRect(spdX[c],spdY[c],36,14,BLACK);
      }
#endif
}

void setup()
{
  pinMode(IRQpin,INPUT);                                    //set the pin monitoring the mfx signal to input
  Serial.begin(115200);                                       //start the serial communications
  while(!Serial){}                                          //wait for comms to start
  delay(500);
  LCDsetup();                                               //initialise LCD and draw screen background (some text and a grid)
#if (defined MEASUREMANT)
  int Vmin=offset;                                          //set up to measure the positive and negative voltage swings applied to the track
  int Vmax=offset;
  for(int c=0;c<100;c++)                                    //analog read time is about 100uS
    {int sensor = analogRead(A15);                          //sample and hold could randomly fall during a transient or long pause
      if(sensor<Vmin)Vmin=sensor;                           //in the track waveform so do multiple samples to greatly reduce the
      if(sensor>Vmax)Vmax=sensor;                           //chance of an incorrect reading
      delay(9);
    }
  Vmax=(Vmax-offset)/scale;                                 //scale the readings to give approximate voltage
  Vmin=(Vmin-offset)/scale;
#endif
#if 1
  tft->setCursor(8,24);                                      //display voltage results results on LCD
  tft->print("Vmax=");tft->print(Vmax);
  tft->setCursor(114,24);
  tft->print("Vmin=");tft->print(Vmin);
  tft->setCursor(232,24);
  tft->print("SYNC");
  tft->fillCircle(295,30,6,RED);                             //start with a red IRQ activity indicator
#endif
  attachInterrupt(digitalPinToInterrupt(IRQpin), IRQtest, CHANGE);  // look for changing signal level on IRQpin just to check the interface cct
  while(IRQflag<1024)delay(1000);                           // wait for 1024 signal edges as proof the mfx signal is there
#if 1
  tft->fillCircle(295,30,6,GREEN);                           //change to green IRQ indicator and proceed when signal is found
#endif
  detachInterrupt(digitalPinToInterrupt(IRQpin));
  Serial.println();
  Serial.println("Starting data collection - please wait");
  previous=micros();
  elapsed=previous;
}

void loop()
{
    if(bitcount==0)
      {
        attachInterrupt(digitalPinToInterrupt(IRQpin), collect, CHANGE);    //enable interrupt pin
        sync=false;
      }
    if(sync==false)
      {
      syncfail:                                             //wait for a sync pulse in the mfx data stream - yes, I know using goto is really bad coding ...
      bitcount=0;                                           //insufficient time for the 8 bit boards to do anything fancy in code so we'll collect and interpret pulses on the fly
      while(bitcount==0);                                   //looking for the unique mfx sync pattern of "0S00S0" being returned by the interrupt routine
      if(data1[0]!='0')goto syncfail;                       //can then start immediately logging of the mfx pulse train 
      while(bitcount==1);
      if(data1[1]!='S')goto syncfail;
      while(bitcount==2);
      if(data1[2]!='0')goto syncfail;
      while(bitcount==3);
      if(data1[3]!='0')goto syncfail;
      while(bitcount==4);
      if(data1[4]!='S')goto syncfail;
      while(bitcount==5);
      if(data1[5]!='0')goto syncfail;
      sync=true;
      }
//detect the end of collection and display some stats
  if(bitcount>=(maxbits))                                   
    {
    detachInterrupt(digitalPinToInterrupt(IRQpin));
    elapsed=(micros()-elapsed)/1000;
#if (defined DEVELOPMENT)
    {
    Serial.print(maxbits);
    Serial.print(" data points collected in ");
    Serial.print(elapsed,DEC);
    Serial.println(" mS");
    }
#endif
    userpause();
    newmax=bitcount;                                        //display the raw data
    displayData();
    userpause(); 
// extract the logic 1s from the data stream
#if (defined DEVELOPMENT)
    Serial.println("Convert any two consecutive short pulses to 1's");
#endif
    int d=0;
    for(int c=0;c<bitcount-1;c++)
      {
      if((data1[c]=='S')&&(data1[c+1]=='S'))
        {data1[d]='1';d++;c++;}
      else
        {data1[d]=data1[c];d++;}
      }
    newmax=d;
    displayData();
    userpause();
// Compress the sync pattern 0S00S0 into a single F character
#if (defined DEVELOPMENT)
    Serial.println("Compress sync patterns to single F characters");
#endif
    d=0;
    for(int c=0;c<newmax;c++)
      {
      if(c<newmax-5)
      {
      if((data1[c]=='0')&&(data1[c+1]=='S')&&(data1[c+2]=='0')&&(data1[c+3]=='0')&&(data1[c+4]=='S')&&(data1[c+5]=='0'))
        {data1[d]='F';d++;c=c+5;}
      else
        {data1[d]=data1[c];d++;}
      }
        else
        {data1[d]=data1[c];d++;}
      }
    newmax=d;
    displayData();
    userpause();
// Remove bit stuffing where a 0 is inserted after eight 1s in a row (used in mfx to avoid a possible clash with DCC protocol sync pulses)
#if (defined DEVELOPMENT)
    Serial.println("Remove any bit stuffing in the data stream");
#endif
    int shuffles=0;
    for(int c=9;c<newmax;c++)
      {
      if((data1[c-8]=='1')&&(data1[c-7]=='1')&&(data1[c-6]=='1')&&(data1[c-5]=='1')&&(data1[c-4]=='1')&&(data1[c-3]=='1')&&(data1[c-2]=='1')&&(data1[c-1]=='1')&&(data1[c]=='0'))
        { 
          for(d=c;d<newmax-1;d++)data1[c]=data1[c+1]; //shuffle everything down 1 place into the bitstuff position
          shuffles++;
        }
      }
    newmax=newmax-shuffles;
    linecount=0;
    displayData();
    userpause();
// Count up the number of frames (between sync pulses = Fs)
    d=0;
    start[0]=0;                                             //recording always starts after a sync pulse
    for(int c=1;c<newmax-1;c++)
      {
        if(data1[c]=='F')
          {
            end[d]=c;
            start[d+1]=c;
          d++;
          }
      }
    numframes=d;                                            //this is total number of frames initially found - but not all contain something significant
// Delete any really short frames - two sync pulses with nothing or just a single item between them
#if (defined DEVELOPMENT)
    Serial.println("Identify and hide any very short frames");
#endif
    for(int c=0;c<numframes;c++)                            //find and store the frame lengths
      length[c]=end[c]-start[c];
    d=0;
    for(int c=0;c<numframes;c++)                            //repack the start/end array only with pointers to the long frames (data left intact in case we need it - only changing the pointers)
      if(length[c]>20)
        {
          start[d]=start[c];
          end[d]=end[c];
          d++;                                              //count how many frames now pass the length test
        }
    numframes=d;                                            //the number of frames that probably contain something of interest after very small frames (padding and clutter) removed
    for(int c=0;c<numframes;c++)                            //repack the length array to match new values in the start/end array
      length[c]=end[c]-start[c];
#if (defined DEVELOPMENT)
      Serial.print("No of frames recovered for display = ");
      Serial.println(numframes);
#endif
      userpause();
//data has been collected and processed - we now have the number of frames and their start, finish and length in the data1 array - now to parse and display
      for(int c=0;c<numframes;c++)
      {
      if(length[c]>=20)                                                   //An adjustable length cut off used during development - no loco speed command should be shorter than this
        {
#if (defined DEVELOPMENT)
        Serial.println();                                                 //print the frame number
        Serial.print("Frame No ");                                     
        Serial.println(c);
        Serial.print("Frame Data ");                                      //print the frame data contents
        for(int d=start[c]+1;d<(end[c]);d++)
          {x=data1[d];Serial.print(x);}    
        Serial.println();
#endif
        frame="";                                                         //convert the data1 array elements of a frame into a single String variable for (easier?) parsing
        for(int d=(start[c]+1);d<end[c];d++)frame=frame+data1[d];
        crc=0x007F;                                           //check CRC to see if it is a valid frame
        for(int c=0;c<frame.length();c++)
        {
          crc=(crc<<1);
          if(frame.charAt(c)=='1')crc=crc+1;
          if((crc&0x0100)>0)crc=(crc&0x00FF)^0x07;
        }
#if (defined DEVELOPMENT)
        Serial.print("CRC ");
        if(crc==0)Serial.println("PASS");
        if(crc!=0)Serial.println("FAIL");
        if(frame.substring(0,9)=="100000000")Serial.println("Controller Address");
#endif
        if((crc==0)&&(frame.substring(0,9)!="100000000"))                 //only process frames that pass CRC and are not the controller (address 0)
        {
                                                                          //check if frame starts with any of the four known address formats and decode if that is correct
        if(frame.substring(0,2)=="10")                                    //7 bit address
          {bintoDEC(2,7);lokADD=val;decode(9);}
        else if(frame.substring(0,3)=="110")                              //9 bit address
          {bintoDEC(3,9);lokADD=val;decode(12);}
        else if(frame.substring(0,4)=="1110")                             //11 bit address
          {bintoDEC(4,11);lokADD=val;decode(15);}
        else if(frame.substring(0,5)=="1111")                             //14 bit address
          {bintoDEC(5,14);lokADD=val;decode(19);}
        }
      }
      userpause();
      }
      LCDupdate();
      userpause();
      bitcount=0;
      Serial.println();
      Serial.println("Starting data collection - please wait");
  }
}

