/*
    INA229 - An arduino library for the INA229 current sensor
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY
 */

#ifndef INA229_h

#define INA229_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define INA229_ADDRESS 0x48 // 000 AGND


// define ina registers
#define CONFIG_1_REG 0x0
#define ADCCONFIG_2_REG 0x1
#define CURRLSBCALC_3_REG 0x2
#define TEMPCOCONFIG_4_REG 0x3
#define VSHUNT_REG 0x4
#define VBUS_REG 0x5
#define DIETEMP_REG 0x6
#define CURRENT_REG 0x7
#define POWER_REG 0x8
#define ENERGY_REG 0x9
#define CHARGE_REG 0xa
#define DIAG_ALRT_REG 0xb
#define SOVL_REG 0xc
#define SUVL_REG 0xd
#define BOVL_REG 0xe
#define BUVL_REG 0xf
#define TEMP_LIMIT_REG 0x10
#define PWR_LIMIT_REG 0x11
#define MANUFACTURER_ID_REG 0x3e
#define DEVICE_ID_REG 0x3f

//--------------Address 0x0---------------------------------
struct CONFIG_1_REGISTER
{
    uint16_t RESERVED1 : 4;    // 
    uint16_t ADCRANGE : 1;    // 
    uint16_t RESERVED2 : 1;    // 
    uint16_t CONVDLY : 8;     // 
    uint16_t RESERVED3 : 1;    // 
    uint16_t RST : 1;         // 
};
union CONFIG_1_REGISTER_U
{
    uint16_t all;
    struct CONFIG_1_REGISTER bit;
};

//--------------Address 0x1---------------------------------
struct ADCCONFIG_2_REGISTER
{
    uint16_t AVG : 3;                    // 0-2
    uint16_t VTCT : 3;                   // 3-5
    uint16_t VSHCT : 3;                  // 6-8
    uint16_t VBUSCT : 3;                 // 9-11
    uint16_t MODE : 4;                   // 12-15

};
union ADCCONFIG_2_REGISTER_U
{
    uint16_t all;
    struct ADCCONFIG_2_REGISTER bit;
};

//--------------Address 0x2---------------------------------
struct CURRLSBCALC_3_REGISTER
{
    uint16_t CURRLSB : 15;        // 0-14
    uint16_t RESERVED : 1;       // 15
};
union CURRLSBCALC_3_REGISTER_U
{
    uint16_t all;
    struct CURRLSBCALC_3_REGISTER bit;
};

//--------------Address 0x3---------------------------------
struct TEMPCOCONFIG_4_REGISTER
{
    uint16_t TEMPCO : 14;  // 0-13
    uint16_t RESERVED : 2; // 14-15
};
union TEMPCOCONFIG_4_REGISTER_U
{
    uint16_t all;
    struct TEMPCOCONFIG_4_REGISTER bit;
};


//--------------Address 0x4---------------------------------
struct VSHUNT_REGISTER
{
    uint32_t RESERVED : 4; // 0-3
    uint32_t VSHUNT : 20; // 4-23
    //uint32_t RESERVED2 : 8; // 24-31
    //uint16_t VSHUNT : 16; // 0-15
};
union VSHUNT_REGISTER_U
{
    uint32_t all;
    //uint16_t all;
    struct VSHUNT_REGISTER bit;
};

//--------------Address 0x21---------------------------------
struct VBUS_REGISTER
{
    uint32_t RESERVED : 4; // 0-3
    uint32_t VBUS : 20;    // 4-23
    //uint32_t RESERVED2 : 8; // 24-31
    //uint16_t VBUS : 16;           // 0-15
};
union VBUS_REGISTER_U
{
    uint32_t all;
    //uint16_t all;
    struct VBUS_REGISTER bit;
};

//--------------Address 0x25---------------------------------
struct DIETEMP_REGISTER
{
    uint16_t DIETEMP : 16;     // 0-15
    //uint16_t RESERVED : 4;     // 1-0
    //uint16_t DIETEMP : 12;     // 11-2
};
union  DIETEMP_REGISTER_U
{
    uint16_t all;
    struct  DIETEMP_REGISTER  bit;
};

//--------------Address 0x26---------------------------------
struct CURRENT_REGISTER
{
    uint32_t RESERVED : 4; // 0-3
    uint32_t CURRENT : 20; // 4-23
    uint32_t RESERVED2 : 8; // 24-31
    //uint16_t CURRENT : 16;     // 0-15
};
union CURRENT_REGISTER_U
{
    //uint16_t all;
    uint32_t all;
    struct CURRENT_REGISTER bit;
};


//--------------Address 0x8---------------------------------
struct POWER_REGISTER
{
    uint32_t POWER : 24;   // 0-23
    uint32_t RESERVED : 8; // 23-31
};
union POWER_REGISTER_U
{
    uint32_t all;
    struct POWER_REGISTER bit;
};

//--------------Address 0x9---------------------------------
struct ENERGY_REGISTER
{
    uint64_t ENERGY : 40;  // 0-23
    uint64_t RESERVED : 24; // 23-31
};
union ENERGY_REGISTER_U
{
    uint64_t all;
    struct ENERGY_REGISTER bit;
};

//--------------Address 0xa---------------------------------
struct CHARGE_REGISTER
{
    uint64_t CHARGE : 40;   // 0-23
    uint32_t RESERVED : 24; // 23-31
};
union CHARGE_REGISTER_U
{
    uint64_t all;
    struct CHARGE_REGISTER bit;
};

//--------------Address 0xb---------------------------------
struct DIAG_ALRT_REGISTER
{
    uint16_t MEMSTAT : 1;                 // 1-0
    uint16_t CNVRF : 1;                   // 11-2
    uint16_t POL : 1;                     // 1-0
    uint16_t BUSUL : 1;                   // 1-0
    uint16_t BUSOL : 1;                   // 11-2
    uint16_t SHNTUL : 1;                  // 1-0
    uint16_t SHNTOL : 1;                  // 1-0
    uint16_t TMPOL : 1;                   // 11-2
    uint16_t RESERVED1 : 1;                // 1-0
    uint16_t MOVF : 1;                    // 1-0
    uint16_t CHROF : 1;                   // 11-2
    uint16_t ENRGOF : 1;                  // 11-
    uint16_t APOL : 1;                    // 1-0
    uint16_t SLWALRT : 1;                 // 1-0
    uint16_t CNVR : 1;                    // 11-2
    uint16_t ALRLEN : 1;                  // 1-0
};
union DIAG_ALRT_REGISTER_U
{
    uint16_t all;
    struct DIAG_ALRT_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct SOVL_REGISTER
{
    uint16_t SOVL : 16;          // 7-0
};
union SOVL_REGISTER_U
{
    uint16_t all;
    struct SOVL_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct SUVL_REGISTER
{
    uint16_t SUVL : 16; // 7-0
};
union SUVL_REGISTER_U
{
    uint16_t all;
    struct SUVL_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct BOVL_REGISTER
{
    uint16_t BOVL : 15; // 7-0
    uint16_t Reserved : 1; // 7-0
};
union BOVL_REGISTER_U
{
    uint16_t all;
    struct BOVL_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct BUVL_REGISTER
{
    uint16_t BUVL : 15;    // 7-0
    uint16_t Reserved : 1; // 7-0
};
union BUVL_REGISTER_U
{
    uint16_t all;
    struct BUVL_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct TEMP_LIMIT_REGISTER
{
    uint16_t TOL : 16; // 7-0
};
union TEMP_LIMIT_REGISTER_U
{
    uint16_t all;
    struct TEMP_LIMIT_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct PWR_LIMIT_REGISTER
{
    uint16_t POL : 16; // 7-0
};
union PWR_LIMIT_REGISTER_U
{
    uint16_t all;
    struct PWR_LIMIT_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct MANUFACTURER_ID_REGISTER
{
    uint16_t MANFID : 16; // 7-0
};
union MANUFACTURER_ID_REGISTER_U
{
    uint16_t all;
    struct MANUFACTURER_ID_REGISTER bit;
};

//--------------Address 0x98---------------------------------
struct DEVICE_ID_REGISTER
{
    uint16_t REV_ID : 4; // 7-0
    uint16_t DIEID : 12;  // 7-0
};
union DEVICE_ID_REGISTER_U
{
    uint16_t all;
    struct DEVICE_ID_REGISTER bit;
};

// All  registers
typedef struct INA229Reg
{
    union CONFIG_1_REGISTER_U reg0;
    union ADCCONFIG_2_REGISTER_U reg1;
    union CURRLSBCALC_3_REGISTER_U reg2;
    union TEMPCOCONFIG_4_REGISTER_U reg3;
    union VSHUNT_REGISTER_U reg4;
    union VBUS_REGISTER_U reg5;
    union DIETEMP_REGISTER_U reg6;
    union CURRENT_REGISTER_U reg7;
    union POWER_REGISTER_U reg8;
    union ENERGY_REGISTER_U reg9;
    union CHARGE_REGISTER_U rega;
    union DIAG_ALRT_REGISTER_U regb;
    union SOVL_REGISTER_U regc;
    union SUVL_REGISTER_U regd;
    union BOVL_REGISTER_U rege;
    union BUVL_REGISTER_U regf;
    union TEMP_LIMIT_REGISTER_U reg10;
    union PWR_LIMIT_REGISTER_U reg11;
    union MANUFACTURER_ID_REGISTER_U reg3e;
    union DEVICE_ID_REGISTER_U reg3f;

} INA229Reg_t;

class INA229
{
    int address;
    float _convert2comp2float(uint64_t, uint16_t, float);
    uint64_t _register40(byte reg);
    uint32_t _register24(byte reg);
    uint16_t _register8(byte);
    uint16_t _register16(byte);
    void _register8(byte, byte);
    void _register16(byte, uint16_t);
    int _CS = 10;
    bool I2C = false;

public:
    INA229Reg_t INA229_Reg; 
    INA229();
    INA229(int CS);
    uint32_t read24(byte);
    uint16_t read16(byte);
    void write16(byte, uint16_t);


    //CONFIG_1 0x0
    boolean setADCRANGE(uint8_t value); //
    boolean getADCRANGE(void);
    boolean setCONVDLY(uint8_t value); //
    boolean setRST(uint8_t value);      //

    //ADCCONFIG_2 0x1
    boolean setAVG(uint8_t value); //
    boolean setVTCT(uint8_t value); //
    boolean setVSHCT(uint8_t value); //
    boolean setVBUSCT(uint8_t value); //
    boolean setMODE(uint8_t value);   //

    // CURRLSBCALC_3 0x2
    boolean setCURRLSB(uint16_t value); //

    // VSHUNT 0x4
    boolean setVSHUNT(uint16_t value);
    uint32_t getVSHUNT(void);
    float getVSHUNT_f(void);

    // VBUS 0x5
    uint32_t getVBUS(void);
    float getVBUS_f(void);

    // DIETEMP 0x6
    uint16_t getDIETEMP(void);
    float getDIETEMP_f(void);

    // CURRENT 0x7
    uint32_t getCURRENT(void);
    float getCURRENT_f(float consta);

    // POWER 0x8
    uint32_t getPOWER(void);

    // POWER 0x9
    uint64_t getENERGY(void);

    // POWER 0xa
    uint64_t getCHARGE(void);

    // DIAG_ALRT 0xb
    boolean getMEMSTAT();
    boolean getCNVRF();
    boolean getPOL();
    boolean getBUSUL();
    boolean getBUSOL();
    boolean getSHNTUL();
    boolean getSHNTOL();
    boolean getTMPOL();
    boolean getMOVF();
    boolean getCHROF();
    boolean getENRGOF();
    boolean getAPOL();
    boolean getSLWALRT();
    boolean setCNVR(byte);
    boolean setALRLEN(byte);

    // SOVL 0xc
    boolean setSOVL(uint16_t value);
    boolean setSOVL_f(float value, float consta);

    // SUVL 0xd
    boolean setSUVL(uint16_t value);
    boolean setSUVL_f(float value, float consta);

    // BOVL 0xe
    boolean setBOVL(uint16_t value);
    boolean setBOVL_f(float value);

    // BUVL 0xf
    boolean setBUVL(uint16_t value);
    boolean setBUVL_f(float value);

    // TEMP_LIMIT 0x10
    boolean setTOL(uint16_t value);
    boolean setTOL_f(float value, float consta);

    // PWR_LIMIT 0x11
    boolean setPOL(uint16_t value);
    boolean setPOL_f(float value, float consta);

    // MANUFACTURER_ID 0x3e
    uint16_t getMANFID(void);

    // DEVICE_ID 0x3f
    uint16_t getREV_ID(void);
    uint16_t getDIEID(void);



};

#endif