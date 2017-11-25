/*
 Copyright © 2014 José Luis Zabalza  License LGPLv3+: GNU
 LGPL version 3 or later <http://www.gnu.org/copyleft/lgpl.html>.
 This is free software: you are free to change and redistribute it.
 There is NO WARRANTY, to the extent permitted by law.
*/

#ifndef LuxI2C_APDS9301_h
#define LuxI2C_APDS9301_h

#include <Wire.h>
#include <inttypes.h>


class LuxI2C_APDS9301 {
	
public: 
	
    typedef enum {low_gain = 0,high_gain} ADCGain;

    typedef enum {low_time = 0,medium_time,high_time,manual} IntegrationTime;



    LuxI2C_APDS9301( uint8_t i2c_addr);

    uint8_t getPartNum();
    uint8_t getRevNum();

    void powerOn();
    void powerOff();
    bool isPowerOn();


    void setOutputInterruptOn();
    void setOutputInterruptOff();
    bool getOutputInterrupt();

    void generateInterruptEveryADCCycle();
    void generateInterruptAnyValueThresholdRange();
    void setGenerateInterruptPeriod(uint8_t num_periods);
    uint8_t getGenerateInterruptMode();


    void setADCGain(ADCGain newGain);
    ADCGain getADCGain();

    void setIntegrationTime(IntegrationTime newIntegrationTime);
    IntegrationTime getIntegrationTime();

    void setManualIntegrationOff();
    void setManualIntegrationOn();
    bool getManualIntegration();

    unsigned getChannel0();
    unsigned getChannel1();
    float    getLux();

    void setLowThreshold(unsigned newThreshold);
    unsigned getLowThreshold();

    void setHighThreshold(unsigned newThreshold);
    unsigned getHighThreshold();

    void clearInterrupt();

private:

    typedef enum  {
                   Control_reg       = 0,
                   Timing_reg,
                   ThresholdLow_reg,
                   ThresholdHigh_reg = 4,
                   Interrupt_reg     = 6,
                   ID_reg            = 10,
                   Data0_reg         = 12,
                   Data1_reg         = 14

                    } APDS9301Register;


    typedef union _TimingRegister
    {
        struct
        {
            uint8_t integrate_time:2;
            uint8_t reserved1:1;
            uint8_t manual_timing_control:1;
            uint8_t adc_gain:1;
            uint8_t reserved2:4;
        } mbits;

        uint8_t mbyte;

    } TimingRegister;


    typedef union _InterruptRegister
    {
        struct
        {
            uint8_t persist:4;
            uint8_t intr:2;
            uint8_t reserved:2;
        } mbits;

        uint8_t mbyte;

    } InterruptRegister;

    typedef union _IDRegister
    {
        struct
        {
            uint8_t revno:4;
            uint8_t partnum:4;
        } mbits;

        uint8_t mbyte;

    } IDRegister;



    unsigned getReg(APDS9301Register reg);
    void setReg(APDS9301Register reg,unsigned newValue);

    uint8_t m_u8I2CAddr;

	
};

#endif

