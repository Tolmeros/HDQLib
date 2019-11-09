#include <HDQ.h> 

HDQ HDQ(HDQ_DEFAULT_PIN);

uint8_t DC1;
uint8_t DC2;

void setup() {
  Serial.begin(9600);
}

void loop() {

	for (uint8_t jj = 0; jj < 0x3F; jj++) { 
		DC1 = HDQ.read(jj);
		DC2 = HDQ.read(jj+1);
		int total = word(DC2, DC1);

		Serial.print("Register 0x");
		Serial.print(jj, HEX);
		Serial.print(": ");
		Serial.println(total);
		jj++;

	}
	delay(2000);  
	Serial.println("");
}
