#include <EEPROM.h>
#define StarboardSide 1
//#define PortSide 1

void setup() {
  Serial.begin(9600);
  for (int i=0; i<500; i++){
      Serial.println("...");
      delay(10);
    }

  #ifdef PortSide
  Serial.println("Will now assign this as port side!");
  #endif
  #ifdef StarboardSide
  Serial.println("Will now assign this as starboard side!");
  #endif
  for (int i=0; i<10; i++){
      Serial.print(10-i);
      Serial.println("...");
      delay(1000);
    }
  #ifdef PortSide
  EEPROM.write(0, 'p');
  #endif
  #ifdef StarboardSide
  EEPROM.write(0, 's');
  #endif

  Serial.println("Done!");
  

}

void loop() {
}
