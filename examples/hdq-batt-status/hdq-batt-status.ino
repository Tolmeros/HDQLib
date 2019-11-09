/*
 * iPhone Battery HDQ Status Example
 * 
 * Copyright (c) 2016 Joe Honold http://mozzwald.com
 *  
 * Requires the Arduino HDQ library from
 * https://github.com/mozzwald/HDQLib
 * 
 */

#include <BQ27000_HDQ.h>

HDQ HDQ(HDQ_DEFAULT_PIN); // Arduino digital pin 7

void setup() {
  Serial.begin(57600);

}

void loop() {
  uint8_t BYT1, BYT2;
  int dev, firm, hard, major, minor;

  uint16_t new_dev, tmp;
  
  /* Print all registers for testing purposes
  for (uint8_t jj = 0; jj < 0x3F; jj++) { 
      BYT1 = HDQ.read(jj);
      BYT2 = HDQ.read(jj+1);
      int total = word(BYT2, BYT1);
  
      Serial.print("Register 0x");
      Serial.print(jj, HEX);
      Serial.print(" / 0x");
      Serial.print(jj+1, HEX);
      Serial.print(": ");
      Serial.println(total);
  
      jj++;
    }
  */

  /* Get device type */
  dev = HDQ.deviceType();

/* Attempt to detect which iPhone battery is connected
   based on device, firmware and hardware versions 
   https://ripitapart.wordpress.com/2014/09/15/so-phone-me-maybe-a-list-of-iphone-batteries-with-gas-gauge-functionality/

   Devices (dev var):
      167 = ? Fake ?
      565 = bq27541 (3G, 3GS, 4G, 4GS)
      569 = bq27545 (5G, 5C, 5S)
      
   Firmware (firm var, BCD):
      117 = v1.17 (3GS)
      125 = v1.25 (4G)
      135 = v1.35 (4GS)
      310 = v3.10 (5G, 5C, 5S)
      502 = v5.02 (6G, 6GP)
      601 = v6.01 (6S, 6SP)

   Hardware (hard var):
      ??? = ???
      
*/
     
  if(dev>=0){ // Check if HDQ capable battery is connected
    Serial.println(F(""));
    //Serial.print(BYT2, HEX); Serial.println(BYT1, HEX);
    
    /* Get Firmware Version */
    firm = BCD16bitToWord(HDQ.commandControl(CONTROL_FW_VERSION));
  
    /* Get Hardware Version */
    hard = HDQ.commandControl(CONTROL_HW_VERSION);

    if(dev == 541){ // bq27541 (3G, 3GS, 4G, 4GS)
      if(firm == 117){
        Serial.println(F("iPhone 3GS Battery Detected"));
      }else if(firm == 125){
        Serial.println(F("iPhone 4G Battery Detected"));
      }else if(firm == 135){
        Serial.println(F("iPhone 4GS Battery Detected"));
      }else{
        Serial.println(F("Unknown HDQ Battery Detected"));
      }
    }else if(dev == 545){ // bq27545 (5G, 5C, 5S)
      if(hard == 0xA7){
        Serial.println(F("iPhone 5S / 5C Battery Detected"));
      }else{
        Serial.println(F("Unknown HDQ Battery Detected"));
      }
    }else if(dev == 143){ // Fake?
      Serial.println(F("Unknown (Fake iPhone 5S/5C) HDQ Battery Detected"));
    }else{
      Serial.println(F("Unknown HDQ Battery Detected"));
    }

    /* Display Device */
    Serial.print("Device: ");
    switch(dev) {
      case 541:
        Serial.println(F("bq27541"));
        break;
      case 545:
        Serial.println(F("bq27545"));
        break;
      default:
        Serial.print(F("Unknown ("));
        Serial.print(dev);
        Serial.println(F(")"));
        break;
    }

    /* Display Firmware Version */
    Serial.print(F("Firmware: v"));
    Serial.println((float) firm/100);

    /* Display Hardware Version */
    Serial.print(F("Hardware: 0x"));
    Serial.println(hard, HEX);

    /* Current charge in mAH */
    Serial.print(F("Remaining Capacity: "));
    Serial.print(HDQ.commandRead(BQ27000_COMMAND_RM), DEC);
    Serial.println(F(" mAH"));
    
    /* Full charge capacity mAH */
    Serial.print(F("Full Charge Capacity: "));
    Serial.print(HDQ.commandRead(BQ27000_COMMAND_FCC), DEC);
    Serial.println(F(" mAH"));
  
    /* Design capacity mAH */
    Serial.print(F("Design Capacity: "));
    Serial.print(HDQ.commandRead(BQ27000_COMMAND_DCAP), DEC);
    Serial.println(F(" mAH"));
    
    /* Time to Empty Minutes */
    int tto = HDQ.commandRead(BQ27000_COMMAND_TTE);
    Serial.print(F("Time to empty: "));
    if(tto == -1){
      Serial.println(F("N/A, not discharging"));
    }else{
      Serial.print(tto, DEC);
      Serial.print(F(" minutes (~"));
      Serial.print(tto/60, DEC);
      Serial.println(F(" hours)"));
    }
    
    /* Time to Full Minutes or Filtered FCC*/
    if(dev == 545) {
      Serial.print(F("Filtered FCC: "));
      Serial.print(HDQ.commandRead(BQ27545_COMMAND_FFCC), DEC);
      Serial.println(F(" mAH"));
    } else {
      int ttf = HDQ.commandRead(BQ27000_COMMAND_TTF);
      Serial.print(F("Time to full: "));
      if(ttf == -1){
        Serial.println(F("N/A, not charging"));
      }else{
        Serial.print(ttf, DEC);
        Serial.println(F(" minutes"));
      }
    }
  
    /* State of Charge % */
    Serial.print(F("State of Charge: "));
    Serial.print(HDQ.commandRead(BQ27000_COMMAND_SOC), DEC);
    Serial.println(F("%"));

    /* Voltage mV */
    int mvolts = HDQ.commandRead(BQ27000_COMMAND_VOLT);
    float volts = (mvolts*0.001);
    Serial.print(F("Battery Voltage: "));
    Serial.print(volts);
    Serial.print(F("V ("));
    Serial.print(mvolts);
    Serial.println(F("mV)"));
    
    /* Temperature (in Kelvin to C/F) */
    int tempK = HDQ.commandRead(BQ27000_COMMAND_TEMP);
    float tempC = (tempK-2731)/10;
    float tempF = (1.8*((tempK*0.1)-273))+32;
    Serial.print(F("Temperature: "));
    Serial.print(tempC);
    Serial.print(char(176));
    Serial.print(F("C / "));
    Serial.print(tempF);
    Serial.print(char(176));
    Serial.print(F("F / "));
    Serial.print(tempK);
    Serial.println(F(" raw"));
    
    /* Charge Cycle Count */
    Serial.print(F("Charge Cycle Count: "));
    Serial.print(HDQ.commandRead(BQ27000_COMMAND_CC), DEC);
    Serial.println(F(" times"));

    /* Average Current */
    Serial.print(F("Average Current: "));
    Serial.print(int(HDQ.commandRead(BQ27000_COMMAND_AI)), DEC);
    Serial.println(F(" mA"));

    /* Control Status */

    tmp = HDQ.commandControl(CONTROL_CONTROL_STATUS);
    Serial.print(F("Control Status: 0x"));
    Serial.println(tmp, HEX);
    if (tmp & 0x8000) {
        Serial.print(F("SE "));
    };
    
    if (tmp & 0x4000) {
        Serial.print(F("FAS "));
    };

    if (tmp & 0x2000) {
        Serial.print(F("SS "));
    };

    if (tmp & 0x1000) {
        Serial.print(F("CALMODE "));
    };

    if (tmp & 0x0800) {
        Serial.print(F("CCA "));
    };

    if (tmp & 0x0400) {
        Serial.print(F("BCA "));
    };

    if (tmp & 0x0200) {
        Serial.print(F("RSVD "));
    };

    if (tmp & 0x0100) {
        Serial.print(F("HDQHOSTIN "));
    };

    if (tmp & 0x0080) {
        Serial.print(F("SHUTDWN "));
    };
    
    if (tmp & 0x0040) {
        Serial.print(F("HIBERNATE "));
    };

    if (tmp & 0x0020) {
        Serial.print(F("FULLSLEEP "));
    };

    if (tmp & 0x0010) {
        Serial.print(F("SLEEP "));
    };

    if (tmp & 0x0008) {
        Serial.print(F("LDMD "));
    };

    if (tmp & 0x0004) {
        Serial.print(F("RUP_DIS "));
    };

    if (tmp & 0x0002) {
        Serial.print(F("VOK "));
    };

    if (tmp & 0x0001) {
        Serial.print(F("QEN "));
    };
    Serial.println(".");
    
  }else{
    // HDQ Battery isn't connected
    Serial.println("Battery not detected");
  }
  
  delay(1000);  
  //Serial.println("");
}
