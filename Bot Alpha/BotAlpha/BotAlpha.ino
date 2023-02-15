#include <SoftwareSerial.h>

SoftwareSerial configureBT(3, 2); // TX || RX, in feite zijn dit de pinmodes

void setup() {
  // Bluetooth setup 
  Serial.begin(9600);
  configureBT.begin(9600); // Indien je commands wilt uitvoeren, knop op module ingedrukt houden bij het inpluggen en 9600 aanpassen naar 38400
}

void loop() { 
  if (configureBT.available()) {
    Serial.write(configureBT.read());
  }
  
  if (Serial.available()) {
    configureBT.write(Serial.read());
  }

  // Sending



  // Receiving
  
  
}





// Robot A (Master) stuurt signaal naar Robot B (Slave) als hij klaar is met het rijden over de lijn. 
// Robot B start. 
// Wanneer robot B klaar is, stuurt hij signaal naar Robot A. 
// Robot A leest het , stuurt hij signaal naar Robot C (Slave)
// Robot C begint met rijden
