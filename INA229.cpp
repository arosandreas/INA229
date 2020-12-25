/*
    INA229 - An arduino library for the INA229 current sensor
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY
 */

#include <Wire.h>
#include <SPI.h>
#include "INA229.h"

/*
INA229::INA229()
{
    address = INA229_ADDRESS;
}
*/

INA229::INA229()
{
    address = INA229_ADDRESS;
    I2C = true;
}



INA229::INA229(int CS)
{
    _CS = CS;
    I2C = false;
    SPI.begin();
    pinMode(_CS, OUTPUT);
    digitalWrite(_CS, HIGH);
}



float INA229::_convert2comp2float(uint64_t twocompdata, uint16_t nrofbit, float factor)
{
    uint64_t isnegative = 1;
    isnegative = (isnegative << (nrofbit - 1));
    //Serial.print(F("isnegative="));
    //Serial.println(isnegative, HEX);
    float dietemp = twocompdata;
    if (dietemp > isnegative)
    {
        dietemp=(dietemp-(2*isnegative))*factor;
    }
    else
    {
        dietemp = (dietemp * factor);
        //Serial.println(F("2comp är positiv:"));
        //Serial.println(dietemp,2);
    }
    return dietemp;
}

uint64_t INA229::_register40(byte reg)
{
    uint64_t regdata = 0;
    uint64_t highByte2 = 0;
    uint64_t highByte = 0;
    uint64_t midByte = 0;
    uint64_t lowByte = 0;
    if (I2C)
    {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(address, 4);
        highByte2 = Wire.read();
        highByte = Wire.read();
        midByte = Wire.read();
        lowByte = Wire.read();
        regdata = regdata | lowByte | (midByte << 8) | (highByte << 16) | (highByte2 << 32);
    }
    else
    {
        reg = (reg << 2);
        reg |= 0x01;
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        highByte2 = SPI.transfer(0);
        highByte = SPI.transfer(0);
        midByte = SPI.transfer(0);
        lowByte = SPI.transfer(0);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
        regdata = regdata | lowByte | (midByte << 8) | (highByte << 16) | (highByte2 << 32);
    }
    return regdata;
}

uint32_t INA229::_register24(byte reg)
{
    uint32_t regdata = 0;
    uint32_t highByte = 0;
    uint32_t midByte = 0;
    uint32_t lowByte = 0;
    if (I2C)
    {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(address, 3);
        highByte = Wire.read();
        midByte = Wire.read();
        lowByte = Wire.read();
        regdata = regdata | lowByte | (midByte << 8) | (highByte << 16);

    }
    else
    {
        reg = (reg << 2);
        reg |= 0x01;
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        highByte = SPI.transfer(0);
        midByte = SPI.transfer(0);
        lowByte = SPI.transfer(0);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
        regdata = regdata |lowByte | (midByte << 8) | (highByte<<16);
    
    }
    return regdata;
}

uint16_t INA229::_register16(byte reg)
{
    uint16_t regdata = 0;
    if (I2C)
    {
        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(address, 2);
        regdata = (Wire.read() << 8) | Wire.read();
    }
    else{
        //int highByte = 0;
        //int highLow = 0;
        reg= (reg<<2);
        reg|=0x01;
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        regdata = SPI.transfer16(0);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
    }
    return regdata;
}

void INA229::_register16(byte reg, uint16_t regdata)
{

    if(I2C){
        byte msb = (byte)(regdata >> 8);
        byte lsb = (byte)(regdata & 0xFF);

        Wire.beginTransmission(address);
        Wire.write(reg);
        Wire.write(msb);
        Wire.write(lsb);
        Wire.endTransmission();
    }
    else  {
        reg = (reg << 2);
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
        digitalWrite(_CS, LOW);
        SPI.transfer(reg);
        SPI.transfer16(regdata);
        digitalWrite(_CS, HIGH);
        SPI.endTransaction();
    }
}


uint16_t INA229::_register8(byte reg)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(address, 1);
    return Wire.read();
}

void INA229::_register8(byte reg, byte regdata)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(regdata);
    Wire.endTransmission();
}

uint32_t INA229::read24(byte regi)
{
    return (_register24(regi));
}

uint16_t INA229::read16(byte regi)
{
    return (_register16(regi));
}

void INA229::write16(byte regtemp, uint16_t temp)
{
    _register16(regtemp, temp);
}


//CONFIG_1 0x0
boolean INA229::setADCRANGE(uint8_t value)
{
    INA229_Reg.reg0.all = _register16(CONFIG_1_REG);
    INA229_Reg.reg0.bit.ADCRANGE=value;
    _register16(CONFIG_1_REG, INA229_Reg.reg0.all);
    return (1);
}

boolean INA229::getADCRANGE(void)
{
    INA229_Reg.reg0.all = _register16(CONFIG_1_REG);
    return (INA229_Reg.reg0.bit.ADCRANGE);
}

boolean INA229::setCONVDLY(uint8_t value) //
{
    INA229_Reg.reg0.all = _register16(CONFIG_1_REG);
    INA229_Reg.reg0.bit.CONVDLY = value;
    _register16(CONFIG_1_REG, INA229_Reg.reg0.all);
    return (1);
}

boolean INA229::setRST(uint8_t value)         //
{
    INA229_Reg.reg0.all = _register16(CONFIG_1_REG);
    INA229_Reg.reg0.bit.RST = value;
    _register16(CONFIG_1_REG, INA229_Reg.reg0.all);
    return (1);
}

//ADCCONFIG_2 0x1
boolean INA229::setAVG(uint8_t value) //
{
    INA229_Reg.reg1.all = _register16(ADCCONFIG_2_REG);
    INA229_Reg.reg1.bit.AVG = value;
    _register16(ADCCONFIG_2_REG, INA229_Reg.reg1.all);
    return (1);
}

boolean INA229::setVTCT(uint8_t value) //
{
    INA229_Reg.reg1.all = _register16(ADCCONFIG_2_REG);
    INA229_Reg.reg1.bit.VTCT = value;
    _register16(ADCCONFIG_2_REG, INA229_Reg.reg1.all);
    return (1);
}

boolean INA229::setVSHCT(uint8_t value) //
{
    INA229_Reg.reg1.all = _register16(ADCCONFIG_2_REG);
    INA229_Reg.reg1.bit.VSHCT = value;
    _register16(ADCCONFIG_2_REG, INA229_Reg.reg1.all);
    return (1);
}

boolean INA229::setVBUSCT(uint8_t value) //
{
    INA229_Reg.reg1.all = _register16(ADCCONFIG_2_REG);
    INA229_Reg.reg1.bit.VBUSCT = value;
    _register16(ADCCONFIG_2_REG, INA229_Reg.reg1.all);
    return (1);
}

boolean INA229::setMODE(uint8_t value) //
{
    INA229_Reg.reg1.all = _register16(ADCCONFIG_2_REG);
    INA229_Reg.reg1.bit.MODE = value;
    _register16(ADCCONFIG_2_REG, INA229_Reg.reg1.all);
    return (1);
}

// CURRLSBCALC_3 0x2
boolean INA229::setCURRLSB(uint16_t value)
{
    INA229_Reg.reg2.all = _register16(CURRLSBCALC_3_REG);
    INA229_Reg.reg2.bit.CURRLSB = value;
    _register16(CURRLSBCALC_3_REG, INA229_Reg.reg2.all);
    return (1);
}

// VSHUNT 0x4
boolean INA229::setVSHUNT(uint16_t value)
{
    INA229_Reg.reg4.all = _register16(VSHUNT_REG);
    INA229_Reg.reg4.bit.VSHUNT = value;
    _register16(VSHUNT_REG, INA229_Reg.reg4.all);
    return (1);
}

uint32_t INA229::getVSHUNT(void)
{
    INA229_Reg.reg4.all = _register24(VSHUNT_REG);
    return (INA229_Reg.reg4.bit.VSHUNT);
}

float INA229::getVSHUNT_f(void)
{
    return (_convert2comp2float(getVSHUNT(), 20, 312.5e-9));
}

// VBUS 0x5
uint32_t INA229::getVBUS(void)
{
    INA229_Reg.reg5.all = _register24(VBUS_REG);
    return (INA229_Reg.reg5.bit.VBUS);
}

float INA229::getVBUS_f(void)
{

    return (_convert2comp2float(getVBUS(), 20, 195.3e-6));
}

// DIETEMP 0x6
uint16_t INA229::getDIETEMP(void)
{
    INA229_Reg.reg6.all = _register16(DIETEMP_REG);
    return (INA229_Reg.reg6.bit.DIETEMP);
}

float INA229::getDIETEMP_f(void)
{

    return (_convert2comp2float(getDIETEMP(), 16, 7.8125e-3));
}

// CURRENT 0x7
uint32_t INA229::getCURRENT(void)
{
    INA229_Reg.reg7.all = _register24(CURRENT_REG);
    return (INA229_Reg.reg7.bit.CURRENT);
}

float INA229::getCURRENT_f(float consta)
{

    return (_convert2comp2float(getCURRENT(), 20, consta));
}

// POWER 0x8
uint32_t INA229::getPOWER(void)
{
    INA229_Reg.reg8.all = _register24(POWER_REG);
    return (INA229_Reg.reg8.bit.POWER);

}

// ENERGY 0x9
uint64_t INA229::getENERGY(void)
{
    INA229_Reg.reg9.all = _register40(ENERGY_REG);
    return (INA229_Reg.reg9.bit.ENERGY);
}

// CHARGE 0xa
uint64_t INA229::getCHARGE(void)
{
    INA229_Reg.rega.all = _register40(CHARGE_REG);
    return (INA229_Reg.rega.bit.CHARGE);
}

// DIAG_ALRT 0xb


boolean INA229::getMEMSTAT()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.MEMSTAT);
}

boolean INA229::getCNVRF()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.CNVRF);
}

boolean INA229::getPOL()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.POL);
}

boolean INA229::getBUSUL()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.BUSUL);
}

boolean INA229::getBUSOL()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.BUSOL);
}

boolean INA229::getSHNTUL()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.SHNTUL);
}

boolean INA229::getSHNTOL()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.SHNTOL);
}

boolean INA229::getTMPOL()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.TMPOL);
}

boolean INA229::getMOVF()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.MOVF);
}

boolean INA229::getCHROF()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.CHROF);
}

boolean INA229::getENRGOF()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.ENRGOF);
}

boolean INA229::getAPOL()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.APOL);
}

boolean INA229::getSLWALRT()
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    return (INA229_Reg.regb.bit.SLWALRT);
}

boolean INA229::setCNVR(byte value)
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    INA229_Reg.regb.bit.CNVR = value;
    _register16(DIAG_ALRT_REG, INA229_Reg.regb.all);
    return (1);
}

boolean INA229::setALRLEN(byte value)
{
    INA229_Reg.regb.all = _register16(DIAG_ALRT_REG);
    INA229_Reg.regb.bit.ALRLEN = value;
    _register16(DIAG_ALRT_REG, INA229_Reg.regb.all);
    return (1);
}

// SOVL 0xc
boolean INA229::setSOVL(uint16_t value)
{
    INA229_Reg.regc.all = _register16(SOVL_REG);
    INA229_Reg.regc.bit.SOVL = value;
    _register16(SOVL_REG, INA229_Reg.regc.all);
    return (1);
}

boolean INA229::setSOVL_f(float value, float consta)
{
    uint16_t data;
    if (value >= 0)
    {
         data = (value * consta) / (16 * 312.5e-9);
    }
    else{
        float value_temp;
        value_temp = value*(-1);
        data = (value_temp * consta) / (16 * 312.5e-9);
        data = ~data;
        data+=1;
    }
    setSOVL(data);
    return (1);
}

// SUVL 0xd
boolean INA229::setSUVL(uint16_t value)
{
    INA229_Reg.regd.all = _register16(SUVL_REG);
    INA229_Reg.regd.bit.SUVL = value;
    _register16(SUVL_REG, INA229_Reg.regd.all);
    return (1);
}

boolean INA229::setSUVL_f(float value, float consta)
{
    uint16_t data;
    if (value >= 0)
    {
        data = (value * consta) / (16 * 312.5e-9);
    }
    else
    {
        float value_temp;
        value_temp = value * (-1);
        data = (value_temp * consta) / (16 * 312.5e-9);
        data = ~data;
        data += 1;
    }
    setSUVL(data);
    return (1);
}

// BOVL 0xe
boolean INA229::setBOVL(uint16_t value)
{
    INA229_Reg.rege.all = _register16(BOVL_REG);
    INA229_Reg.rege.bit.BOVL = value;
    _register16(BOVL_REG, INA229_Reg.rege.all);
    return (1);
}

boolean INA229::setBOVL_f(float value)
{
    uint16_t data = value / (16 * 195.3125e-6);
    setBOVL(data);
    return (1);
}

// BUVL 0xf
boolean INA229::setBUVL(uint16_t value)
{
    INA229_Reg.regf.all = _register16(BUVL_REG);
    INA229_Reg.regf.bit.BUVL = value;
    _register16(BUVL_REG, INA229_Reg.regf.all);
    return (1);
}

boolean INA229::setBUVL_f(float value)
{
    uint16_t data = value / (16 * 195.3125e-6);
    setBUVL(data);
    return (1);
}

// TEMP_LIMIT 0x10
boolean INA229::setTOL(uint16_t value)
{
    INA229_Reg.reg10.all = _register16(TEMP_LIMIT_REG);
    INA229_Reg.reg10.bit.TOL = value;
    _register16(TEMP_LIMIT_REG, INA229_Reg.reg10.all);
    return (1);
}

boolean INA229::setTOL_f(float value, float consta)
{
    uint16_t data = value / (16 * consta);
    setTOL(data);
    return (1);
}

// PWR_LIMIT 0x11
boolean INA229::setPOL(uint16_t value)
{
    INA229_Reg.reg11.all = _register16(PWR_LIMIT_REG);
    INA229_Reg.reg11.bit.POL = value;
    _register16(PWR_LIMIT_REG, INA229_Reg.reg11.all);
    return (1);
}

boolean INA229::setPOL_f(float value, float consta)
{
    uint16_t data = value / (16 * consta);
    setPOL(data);
    return (1);
}

// MANUFACTURER_ID 0x3e
uint16_t INA229::getMANFID(void)
{
    INA229_Reg.reg3e.all = _register16(MANUFACTURER_ID_REG);
    return (INA229_Reg.reg3e.bit.MANFID);
}

// DEVICE_ID 0x3f
uint16_t INA229::getREV_ID(void)
{
    INA229_Reg.reg3f.all = _register16(DEVICE_ID_REG);
    return (INA229_Reg.reg3f.bit.REV_ID);
}

uint16_t INA229::getDIEID(void)
{
    INA229_Reg.reg3f.all = _register16(DEVICE_ID_REG);
    return (INA229_Reg.reg3f.bit.DIEID);
}



