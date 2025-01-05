#include <MaerklinMotorola.h>


#define INPUT_PIN 2  //for arduino uno

//#define INPUT_PIN 4

MaerklinMotorola mm(INPUT_PIN);

void mm_isr() {
#if 0
  Serial.print(".");
#endif
  mm.PinChange();
}

void setup() {
  pinMode(INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), mm_isr, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);


  Serial.begin(115200);
  Serial.println("MM/MM2 protocolsniffer started");
}

unsigned int i = 0;
void loop() {

  int port_number;
  int keyboard;
  int wissel;
  int groen;
  int address;
  int input = digitalRead(INPUT_PIN);
  //Serial.println(input);
  digitalWrite(LED_BUILTIN, input);  



  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();
  if(Data) {
    if(Data->Address != 80){
  #if 1
      for(int i=0;i<9;i++) {
        Serial.print(Data->Trits[i]);
        Serial.print(" ");
      }
      Serial.print("\n");
  #endif
    }

    
    //if(1){
    //if(Data->Address == 1 && Data->MagnetState){
    if(Data->IsMagnet && Data->MagnetState){
      address = Data->Address ? Data->Address : 80; // Correct address 0 to 80
      port_number = ((int)address - 1) * 8 + (int)Data->SubAddress;
      keyboard = (port_number/32) + 1;
      wissel = (port_number / 2) + 1;
      groen = port_number % 2;
      Serial.print("MM Solenoid -->");
      Serial.print("; Address = "); Serial.print(Data->Address);
      //Serial.print(" Speed= "); Serial.print(Data->Speed);
      //Serial.print(" Step= "); Serial.print(Data->Step);
      //Serial.print(" MM2FunctionIndex= "); Serial.print(Data->MM2FunctionIndex);
      Serial.print("; SubAddress = "); Serial.print(Data->SubAddress);
      Serial.print("; PortAddress = "); Serial.print(Data->PortAddress);
      Serial.print("; PortNumber = "); Serial.print(port_number);
      Serial.print("; Keyboard = "); Serial.print(keyboard);
      Serial.print("; Wissel = "); Serial.print(wissel);
      Serial.print("; MagnetState = " + String(groen ? "groen" : "rood"));
      //Serial.print(" Function= "); Serial.print(Data->Function);
      //Serial.print(" Stop= "); Serial.print(Data->Stop);
      //Serial.print(" ChangeDir= "); Serial.print(Data->ChangeDir);
      //Serial.print(" MagnetState= " + String(Data->MagnetState ? "yes" : "no"));
      //Serial.print(" IsMagnet= " + String(Data->IsMagnet ? "yes" : "no"));
      //Serial.print(" IsMM2= " + String(Data->IsMM2 ? "yes" : "no"));
      //Serial.print(" IsMM2FunctionOn= " + String(Data->IsMM2FunctionOn ? "yes" : "no"));
      Serial.println();
    }
    if(!Data->IsMagnet) {
      if(Data->IsMM2){
        if(Data->Address != 80){
          Serial.print("MM2 -->");
          Serial.print(" Address = "); Serial.print(Data->Address);
          Serial.print("; Speed = "); Serial.print(Data->Speed);
          Serial.print("; Auxilary = " + String(Data->Function ? "On" : "Off"));
          //Serial.print(" Step= "); Serial.print(Data->Step);
          Serial.print("; MM2FunctionIndex= "); Serial.print(Data->MM2FunctionIndex);
          //Serial.print(" SubAddress= "); Serial.print(Data->SubAddress);
          //Serial.print(" PortAddress= "); Serial.print(Data->PortAddress);
          //Serial.print(" PortNumber= "); Serial.print(port_number);
          //Serial.print(" Keyboard= "); Serial.print(keyboard);
          //Serial.print(" Wissel= "); Serial.print(wissel);
          //Serial.print(" MagnetState= " + String(groen ? "groen" : "rood"));
          //Serial.print(" Stop= "); Serial.print(Data->Stop);
          //Serial.print(" ChangeDir= "); Serial.print(Data->ChangeDir);
          //Serial.print(" MagnetState= " + String(Data->MagnetState ? "yes" : "no"));
          //Serial.print(" IsMagnet= " + String(Data->IsMagnet ? "yes" : "no"));
          //Serial.print(" IsMM2= " + String(Data->IsMM2 ? "yes" : "no"));
          Serial.print("; IsMM2Function = " + String(Data->IsMM2FunctionOn ? "On" : "Off"));
          Serial.println();
        }
      }
      else{
        Serial.print("MM1 -->");
        Serial.print(" Address = "); Serial.print(Data->Address);
        Serial.print("; Speed = "); Serial.print(Data->Speed);
        Serial.print("; Auxilary = " + String(Data->Function ? "On" : "Off"));
        //Serial.print(" Step= "); Serial.print(Data->Step);
        //Serial.print(" MM2FunctionIndex= "); Serial.print(Data->MM2FunctionIndex);
        //Serial.print(" SubAddress= "); Serial.print(Data->SubAddress);
        //Serial.print(" PortAddress= "); Serial.print(Data->PortAddress);
        //Serial.print(" PortNumber= "); Serial.print(port_number);
        //Serial.print(" Keyboard= "); Serial.print(keyboard);
        //Serial.print(" Wissel= "); Serial.print(wissel);
        //Serial.print(" MagnetState= " + String(groen ? "groen" : "rood"));
        //Serial.print(" Function= "); Serial.print(Data->Function);
        //Serial.print(" Stop= "); Serial.print(Data->Stop);
        Serial.print("; ChangeDir = "); Serial.print(Data->ChangeDir);
        //Serial.print(" MagnetState= " + String(Data->MagnetState ? "yes" : "no"));
        //Serial.print(" IsMagnet= " + String(Data->IsMagnet ? "yes" : "no"));
        //Serial.print(" IsMM2= " + String(Data->IsMM2 ? "yes" : "no"));
        //Serial.print(" IsMM2FunctionOn= " + String(Data->IsMM2FunctionOn ? "yes" : "no"));
        Serial.println();
      }
    }
  }
}



