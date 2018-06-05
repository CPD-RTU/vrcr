#include "Arduino.h"
// Shield Jumper on SW (using pins 12/13 or 8/9 as RX/TX)
#include "SoftwareSerial.h"
SoftwareSerial port(12, 13);
#include "EasyVR.h"
 
EasyVR easyvr(port);
 
enum Groups
{
  GROUP_1  = 1,
};
 
enum Group1 
{
  G1_STOP = 1,
  G1_NAZAD = 2,
  G1_VPERED = 3,
  G1_VLEVO = 4,
  G1_VPRAVO = 5,
  G1_AVTO = 6
};
 
int8_t group, idx;
 
void setup()
{
  Serial.begin(9600);
  port.begin(9600);
 
  // initialize EasyVR  
  while (!easyvr.detect())
  {
  //pcSerial.println(F("EasyVR not detected!"));
  }
 
  //pcSerial.print(F("EasyVR detected, version "));
  //pcSerial.println(easyvr.getID());
 
  easyvr.setDelay(0); // speed-up replies
 
  easyvr.setTimeout(5);
  easyvr.setLanguage(0); //<-- same language set on EasyVR Commander when code was generated
 
  group = EasyVR::TRIGGER; //<-- start group (customize)
}
 
void golos()
{
  if (group >= 0) // SI wordset/grammar
  {
    //pcSerial.print("Say a command in Group ");
    //pcSerial.println(group);
    easyvr.recognizeCommand(group);
  }
 
  while (!easyvr.hasFinished());
 
  idx = easyvr.getWord();
  if (idx == 0 && group == EasyVR::TRIGGER)
  {
    //pcSerial.println("Word: ROBOT");
    group = GROUP_1;
    return;
  }
  idx = easyvr.getCommand();
  if (idx >= 0)
  {
    int8_t command = idx+1;
    Serial.print(command);
    if(command==G1_VLEVO||command==G1_VPRAVO) {
      delay(500);
      Serial.print(G1_STOP);
    }
   /*// print debug message
    uint8_t train = 0;
    char name[32];
    Serial.print("Command: ");
    pcSerial.print(idx);
    if (easyvr.dumpCommand(group, idx, name, train))
    {
      pcSerial.print(" = ");
      pcSerial.println(name);
    }
    else
      pcSerial.println();
       switch (group)
         {
          case GROUP_1:
             switch (idx)
             {
              case G1_STOP:
                pcSerial.println("000000000000000000000000000000000000000000000");
                break;
              case G1_NAZAD:
                pcSerial.println("---------------------------------------------");
                break;
              case G1_VPERED:
                pcSerial.println("+++++++++++++++++++++++++++++++++++++++++++++");
                break;
              case G1_VLEVO:
                pcSerial.println("<- <- <- <- <- <- <- <- <- <- <- <- <- <- <- ");
                break;
              case G1_VPRAVO:
                pcSerial.println("-> -> -> -> -> -> -> -> -> -> -> -> -> -> ->");
                break;
              case G1_AVTO:
                pcSerial.println("AVTOMTICHECKOE UPRAVLENIE VKLUCHENO. DA POMOZET NAM BOG...");
             } 
      break;
    }*/
  }
  else // errors or timeout
  {
    int16_t err = easyvr.getError();
    if (err >= 0)
    {
      //pcSerial.print("Error ");
      //pcSerial.println(err, HEX);
    }
  }
  return ;
}
 
void loop()
{
 
  golos ();
 
  //Serial.println("__________________________");
 
  delay(250);
}
