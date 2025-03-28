#include <MaerklinMotorola.h>


#define TRACK_PULSE_INPUT_PIN 2  //for arduino uno

//#define TRACK_PULSE_INPUT_PIN 4

MaerklinMotorola mm(TRACK_PULSE_INPUT_PIN);

void mm_isr() {
#if 0
  Serial.print(".");
#endif
  mm.PinChange();
}

bool display_speed = true;
bool display_solenoids = true;
bool display_mm2_functions = true;
bool display_additional_functions = true;
#if (defined INCLUDE_BITSTREAM_DISPLAY)
bool display_bitstream = false;
#endif


void display_menu(){
  Serial.println("Toggle display function by pressing:");
  Serial.print("  1. Speed messages, current ");
  Serial.println(display_speed ? "On" : "Off");
  Serial.print("  2. Function messages, current ");
  Serial.println(display_mm2_functions ? "On" : "Off");
  Serial.print("  3. Solenoid messages, current ");
  Serial.println(display_solenoids ? "On" : "Off");
  Serial.print("  4. Additional Function messages, current ");
  Serial.println(display_additional_functions ? "On" : "Off");
#if (defined INCLUDE_BITSTREAM_DISPLAY)
  Serial.print("  5. Display bitstream, current ");
  Serial.println(display_bitstream ? "On" : "Off");
#endif
}

void setup() {
  pinMode(TRACK_PULSE_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TRACK_PULSE_INPUT_PIN), mm_isr, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("MM/MM2 protocol-sniffer started");
  display_menu();
}

#if (defined INCLUDE_BITSTREAM_DISPLAY)
void printBitStream(uint32_t bit_stream){
  //Serial.println(bit_stream);
  for(int j = 0; j < 9; j++){
  
  if(j==0)Serial.print("Address: ");
  if(j==4)Serial.print("Auxilary: ");
  if(j==5)Serial.print("Data: ");
  (bit_stream & 0x01) ? Serial.print("1") : Serial.print("0");
  bit_stream = bit_stream >> 1;
  (bit_stream & 0x01) ? Serial.print("1") : Serial.print("0");
  bit_stream = bit_stream >> 1;
  Serial.print(" ");
}
Serial.println();
}
#endif

unsigned int i = 0;
void loop() {

  int port_number;
  int wissel;
  int groen;
  int address;

  int input = digitalRead(TRACK_PULSE_INPUT_PIN);
  //Serial.println(input);
  digitalWrite(LED_BUILTIN, input);  

  if (Serial.available() > 0) {
    // Lees het eerste teken dat is ontvangen
    char input = Serial.read();

    Serial.println(input);
    switch(input){
      case '1':
        display_speed = !display_speed;
        break;
      case '2':
        display_mm2_functions = !display_mm2_functions;
        break;
      case '3':
        display_solenoids = !display_solenoids;
        break;
      case '4':
        display_additional_functions = !display_additional_functions;
        break;
#if (defined INCLUDE_BITSTREAM_DISPLAY)
      case '5':
        display_bitstream = !display_bitstream;
        break;
#endif
    }
    display_menu();
  }

  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();
  if(Data) {
  #if 0
    if(Data->Address != 80){
      for(int i=0;i<9;i++) {
        Serial.print(Data->Trits[i]);
        Serial.print(" ");
      }
      Serial.print("\n");
    }
  #endif

    
    //if(1){
    //if(Data->Address == 1 && Data->MagnetState){
    //if(Data->IsMagnet && Data->MagnetState && !Data->IsAdditionalFunction && display_solenoids){
    if(Data->IsMagnet && !Data->IsAdditionalFunction && display_solenoids){
      address = Data->Address ? Data->Address : 80; // Correct address 0 to 80
      port_number = ((int)address - 1) * 8 + (int)Data->SubAddress;
      wissel = (port_number / 2) + 1;
      groen = port_number % 2;
      Serial.print("MM Solenoid -->");
      Serial.print("; PortAddress = "); Serial.print(Data->PortAddress);
      Serial.print("; Turnout = "); Serial.print(wissel);
      Serial.print("; Magnet = ");
      if(groen)
        Serial.print("Green");
      else
        Serial.print("Red");
      Serial.print("; State = ");
      if(Data->MagnetState)
        Serial.print("On");
      else
        Serial.print("Off");
      Serial.println();
#if (defined INCLUDE_BITSTREAM_DISPLAY)
      if(display_bitstream){
        printBitStream(Data->BitStream);
      }
#endif
    }

    if(Data->IsMagnet && Data->IsAdditionalFunction && display_additional_functions){
      address = Data->Address ? Data->Address : 80; // Correct address 0 to 80
      port_number = ((int)address - 1) * 8 + (int)Data->SubAddress;
      wissel = (port_number / 2) + 1;
      groen = port_number % 2;
      uint8_t mask = 0b1;
      for(int i = 0; i < 4; i++){
        Serial.print("MM1 Additional Function -->");
        Serial.print(" Address = "); Serial.print(Data->Address);
        Serial.print("; Function("+ String(i+1) + ") = "  + String(Data->IsMM1FunctionOn & mask ? "On" : "Off"));
        Serial.println();
        mask = mask << 1;
      }
#if (defined INCLUDE_BITSTREAM_DISPLAY)
      if(display_bitstream){
        printBitStream(Data->BitStream);
      }
#endif
    }
    if(!Data->IsMagnet && display_speed) {
      if(Data->IsMM2){
        if(Data->IsSpeed || display_mm2_functions){
          if(Data->Address != 80){
            Serial.print("MM2 -->");
            Serial.print(" Address = "); Serial.print(Data->Address);
            Serial.print("; Speed = "); Serial.print(Data->Speed);
            switch(Data->MM2Direction){
              case MM2DirectionState_Unavailable:
                Serial.print("; Direction = Unavailable");
                break;
              case MM2DirectionState_Forward_H:
                Serial.print("; Direction = Forward (+6..+14)");
                break;
              case MM2DirectionState_Forward_L:
                Serial.print("; Direction = Forward  (+0..+6)");
                break;
              case MM2DirectionState_Backward_L:
                Serial.print("; Direction = Backward (-6..-0)");
                break;
              case MM2DirectionState_Backward_H:
                Serial.print("; Direction = Backward (-7..-14)");
                break;
            }
            Serial.print("; Auxilary = " + String(Data->Function ? "On" : "Off"));
            if(Data->ChangeDir)Serial.print("; Change Direction");
            if(!Data->IsSpeed){
              Serial.print("; FunctionIndex= "); Serial.print(Data->MM2FunctionIndex);
              Serial.print("; Function = " + String(Data->IsMM2FunctionOn ? "On" : "Off"));
            }
            Serial.println();
#if (defined INCLUDE_BITSTREAM_DISPLAY)
            if(display_bitstream){
              printBitStream(Data->BitStream);
            }
#endif
          }
        }
      }
      else{
        if(Data->Address != 80){
          Serial.print("MM1 -->");
          Serial.print(" Address = "); Serial.print(Data->Address);
          Serial.print("; Speed = "); Serial.print(Data->Speed);
          Serial.print("; Auxilary = " + String(Data->Function ? "On" : "Off"));
          Serial.print("; ChangeDir = "); Serial.print(Data->ChangeDir);
          Serial.println();
#if (defined INCLUDE_BITSTREAM_DISPLAY)
          if(display_bitstream){
              printBitStream(Data->BitStream);
          }
#endif
        }
      }
    }
  }
}



