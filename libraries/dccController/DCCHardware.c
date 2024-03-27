#include "Arduino.h"
#include "DCCHardware.h"

//#define DEBUG     
#if defined DEBUG
  #define DEBUG_PRINT(x)      Serial.print(x)
  #define DEBUG_PRINTLN(x)    Serial.println(x)
#else
  #define DEBUG_PRINT(x)      
  #define DEBUG_PRINTLN(x)    
#endif

/// An enumerated type for keeping track of the state machine used in the timer1 ISR
/** Given the structure of a DCC packet, the ISR can be in one of 5 states.
      *dos_idle: there is nothing to put on the rails. In this case, the only legal thing
                 to do is to put a '1' on the rails.  The ISR should almost never be in this state.
      *dos_send_premable: A packet has been made available, and so we should broadcast the preamble: 14 '1's in a row
      *dos_send_bstart: Each data uint8_t is preceded by a '0'
      *dos_send_uint8_t: Sending the current data uint8_t
      *dos_end_bit: After the final uint8_t is sent, send a '0'.
*/                 
typedef enum  {
  dos_idle,
  dos_send_preamble,
  dos_send_bstart,
  dos_send_uint8_t,
  dos_end_bit
} DCC_output_state_t;

DCC_output_state_t DCC_state = dos_idle; //just to start out

/// The currently queued packet to be put on the rails. Default is a reset packet.
uint8_t current_packet[6] = {0,0,0,0,0,0};
/// How many data uint8_ts in the queued packet?
volatile uint8_t current_packet_size = 0;
/// How many uint8_ts remain to be put on the rails?
volatile uint8_t current_uint8_t_counter = 0;
/// How many bits remain in the current data uint8_t/preamble before changing states?
volatile uint8_t current_bit_counter = 14; //init to 14 1's for the preamble
/// A fixed-content packet to send when idle
//uint8_t DCC_Idle_Packet[3] = {255,0,255};
/// A fixed-content packet to send to reset all decoders on layout
//uint8_t DCC_Reset_Packet[3] = {0,0,0};


/// Timer1 TOP values for one and zero
/** S 9.1 A specifies that '1's are represented by a square wave with a half-period of 58us (valid range: 55-61us)
    and '0's with a half-period of >100us (valid range: 95-9900us)
    Because '0's are stretched to provide DC power to non-DCC locos, we need two zero counters,
     one for the top half, and one for the bottom half.

   Here is how to calculate the timer1 counter values (from ATMega168 datasheet, 15.9.2):
 f_{OC1A} = \frac{f_{clk_I/O}}{2*N*(1+OCR1A)})
 where N = prescalar, and OCR1A is the TOP we need to calculate.
 We know the desired half period for each case, 58us and >100us.
 So:
 for ones:
 58us = (8*(1+OCR1A)) / (16MHz)
 58us * 16MHz = 8*(1+OCR1A)
 58us * 2MHz = 1+OCR1A
 OCR1A = 115

 for zeros:
 100us * 2MHz = 1+OCR1A
 OCR1A = 199
 
 Thus, we also know that the valid range for stretched-zero operation is something like this:
 9900us = (8*(1+OCR1A)) / (16MHz)
 9900us * 2MHz = 1+OCR1A
 OCR1A = 19799
 
*/

uint16_t one_count=58; //58us
uint16_t zero_high_count=100; //100us
uint16_t zero_low_count=100; //100us

/// Setup phase: configure and enable timer1 CTC interrupt, set OC1A and OC1B to toggle on CTC

hw_timer_t *waveform_generator_timer = NULL;

void IRAM_ATTR waveform_generator_timer_isr();

#define OUTPUT_PIN 12
uint8_t output_state = LOW;

void setup_DCC_waveform_generator() {

  // see: https://deepbluembedded.com/esp32-timers-timer-interrupt-tutorial-arduino-ide/
  waveform_generator_timer = timerBegin(3, 80, true); // Time 3 (is it free?) 80 MHz
  if(waveform_generator_timer){
    DEBUG_PRINT("Error creating waveform timer");
    return 0;
  }
  timerAttachInterrupt(waveform_generator_timer, &waveform_generator_timer_isr, true);
  timerWrite(waveform_generator_timer, zero_high_count);

  output_state = LOW; //Power of the track
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, output_state);
  
  //timerStart(waveform_generator_timer);
}

void DCC_waveform_generation_hasshin()
{
  //enable the compare match interrupt
  timerStart(waveform_generator_timer);
}

/// This is the Interrupt Service Routine (ISR) for Timer1 compare match.
void IRAM_ATTR waveform_generator_timer_isr()
{

  //Toggle output
  output_state = output_state ? LOW : HIGH;
  digitalWrite(OUTPUT_PIN, output_state);

  if(output_state) //the pin is high. New cycle is begining. Here's where the real work goes.
  {
     //time to switch things up, maybe. send the current bit in the current packet.
     //if this is the last bit to send, queue up another packet (might be the idle packet).
    switch(DCC_state)
    {
      /// Idle: Check if a new packet is ready. If it is, fall through to dos_send_premable. Otherwise just stick a '1' out there.
      case dos_idle:
        if(!current_uint8_t_counter) //if no new packet
        {
          DEBUG_PRINTLN("X");
          timerWrite(waveform_generator_timer, one_count); //just send ones if we don't know what else to do. safe bet.
          break;
        }
        //looks like there's a new packet for us to dump on the wire!
        //for debugging purposes, let's print it out
#if defined DEBUG
        if(current_packet[1] != 0xFF)
        {
          DEBUG_PRINT("Packet: ");
          for(uint8_t j = 0; j < current_packet_size; ++j)
          {
            DEBUG_PRINT(current_packet[j],HEX);
            DEBUG_PRINT(" ");
          }
          DEBUG_PRINTLN("");
        }
#endif
        DCC_state = dos_send_preamble; //and fall through to dos_send_preamble
      /// Preamble: In the process of producing 14 '1's, counter by current_bit_counter; when complete, move to dos_send_bstart
      case dos_send_preamble:
        timerWrite(waveform_generator_timer, one_count);
        DEBUG_PRINT("P");
        if(!--current_bit_counter)
          DCC_state = dos_send_bstart;
        break;
      /// About to send a data uint8_t, but have to peceed the data with a '0'. Send that '0', then move to dos_send_uint8_t
      case dos_send_bstart:
        timerWrite(waveform_generator_timer, zero_high_count);
        DCC_state = dos_send_uint8_t;
        current_bit_counter = 8;
        DEBUG_PRINT(" 0 ");
        break;
      /// Sending a data uint8_t; current bit is tracked with current_bit_counter, and current uint8_t with current_uint8_t_counter
      case dos_send_uint8_t:
        if(((current_packet[current_packet_size-current_uint8_t_counter])>>(current_bit_counter-1)) & 1) //is current bit a '1'?
        {
          timerWrite(waveform_generator_timer, one_count);
          DEBUG_PRINT("1");
        }
        else //or is it a '0'
        {
          timerWrite(waveform_generator_timer, zero_high_count);
          DEBUG_PRINT("0");
        }
        if(!--current_bit_counter) //out of bits! time to either send a new uint8_t, or end the packet
        {
          if(!--current_uint8_t_counter) //if not more uint8_ts, move to dos_end_bit
          {
            DCC_state = dos_end_bit;
          }
          else //there are more uint8_ts…so, go back to dos_send_bstart
          {
            DCC_state = dos_send_bstart;
          }
        }
        break;
      /// Done with the packet. Send out a final '1', then head back to dos_idle to check for a new packet.
      case dos_end_bit:
        timerWrite(waveform_generator_timer, one_count);
        DCC_state = dos_idle;
        current_bit_counter = 14; //in preparation for a premable...
        DEBUG_PRINTLN(" 1");
        break;
    }
  }
}
