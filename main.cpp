/* DAC53401
Andreas Jonsson

*/

#include <Arduino.h>
#include <Wire.h>
#include "INA229.h"

#define BOVL_C (0x0fff)
#define SHUNT_RES (215)
float current_lsb=0;

uint16_t BOVL_var = BOVL_C;

//INA229 ina1; // initialize DAC1 for i2c
INA229 ina1 = INA229(10); // USE FOR SPI COMMUNICATION, INA229(CS_PIN);


void  setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("RESTARTED"));
  Serial.println(F("Configure INA229"));
  
  //configure CONFIG_1 Register
  /*
  ina1.setADCRANGE(1);
  ina1.setCONVDLY(22);
  ina1.setRST(1);

  //configure ADCCONFIG_2 Register
  ina1.setAVG(2);
  ina1.setVTCT(4);
  ina1.setVSHCT(4);
  ina1.setVBUSCT(4);
  ina1.setMODE(0xf);

  //configure DIAG_ALRT Register
*/
  Serial.println(F("Keep default configuration"));

  Serial.print(F("MANUFACTURER_ID: "));
  Serial.println(ina1.getMANFID(),HEX);
  delay(2000);

  Serial.print(F("getDIE_ID: "));
  Serial.println(ina1.getREV_ID(), HEX);
  delay(2000);

  Serial.print(F("getREV_ID: "));
  Serial.println(ina1.getDIEID(), HEX);
  delay(2000);

  Serial.println(F("getADCRANGE: "));
  if(ina1.getADCRANGE()){
    Serial.println(F("getADCRANGE = 40.96mV fullscale"));
    current_lsb= (40.96e-3/SHUNT_RES)/524288;
  }
  else{
    Serial.println(F("getADCRANGE = 163.84mV fullscale"));
    current_lsb= (163.84e-3/SHUNT_RES)/524288;
  }
  
  Serial.println(F("setBOVL_f(5.00V)"));
  ina1.setBOVL_f(5.00);
  delay(2000);

  Serial.println(F("setBUVL_f(2.50V)"));
  ina1.setBUVL_f(2.50);
  delay(2000);

  Serial.println(F("setSOVL(0.6mA)"));
  ina1.setSOVL_f(0.60e-3, SHUNT_RES);
  delay(2000);

  Serial.println(F("setSUVL(0.25mA)"));
  ina1.setSUVL_f(0.25e-3, SHUNT_RES);
  delay(2000);

}

void loop()
{
  
  delay(2000);
  Serial.print(F("getVBUS_f:"));
  Serial.println(ina1.getVBUS_f(), 2);

  Serial.print(F("getDIETEMP_f:"));
  Serial.println(ina1.getDIETEMP_f(), 2);

  Serial.print(F("getVSHUNT_f:"));
  Serial.println(ina1.getVSHUNT_f(), 4);

  Serial.print(F("getCURRENT:"));
  Serial.println(ina1.getCURRENT());

  float Current = ina1.getCURRENT_f(current_lsb);
  Serial.print(F("calculatedCURRENT:"));
  Serial.println(Current, 8);

  float Power = (ina1.getPOWER() * current_lsb * 3.2);
  Serial.print(F("calculatedPower:"));
  Serial.println(Power, 8);

  float Energy = (ina1.getENERGY() * current_lsb * 3.2 * 16);
  Serial.print(F("calculatedEnergy:"));
  Serial.println(Energy, 8);

// Check Over and under warnings

  if (ina1.getBUSUL())
    Serial.println(F("Bus Under Voltage triggerd"));

  if (ina1.getBUSOL())
    Serial.println(F("Bus Over Voltage triggerd"));

  if (ina1.getSHNTUL())
    Serial.println(F("Shunt Under Voltage triggerd"));

  if (ina1.getSHNTOL())
    Serial.println(F("Shunt Over Voltage triggerd"));


  Serial.println();
}