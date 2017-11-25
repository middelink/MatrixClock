/*
 Copyright © 2014 José Luis Zabalza  License LGPLv3+: GNU
 LGPL version 3 or later <http://www.gnu.org/copyleft/lgpl.html>.
 This is free software: you are free to change and redistribute it.
 There is NO WARRANTY, to the extent permitted by law.
*/


#include "apds9301.h"
#include <Arduino.h>


//-------------------------------------------------------------------------------
LuxI2C_APDS9301::LuxI2C_APDS9301( uint8_t i2c_addr)
{
//    Wire.begin();

    m_u8I2CAddr = i2c_addr;
}

//-------------------------------------------------------------------------------
float LuxI2C_APDS9301::getLux()
{
    /*
     * Empiric formulas from datasheet
        CH1/CH0			        Sensor Lux Formula
        ========================================================================================
        0 < CH1/CH0 ≤ 0.50	    Sensor Lux = (0.0304 x CH0) – (0.062 x CH0 x ((CH1/CH0)^1.4))
        0.50 < CH1/CH0 ≤ 0.61	Sensor Lux = (0.0224 x CH0) – (0.031 x CH1)
        0.61 < CH1/CH0 ≤ 0.80	Sensor Lux = (0.0128 x CH0) – (0.0153 x CH1)
        0.80 < CH1/CH0 ≤ 1.30	Sensor Lux = (0.00146 x CH0) – (0.00112 x CH1)
        CH1/CH0>1.30		    Sensor Lux = 0
     */
    float CH0 = getReg(Data0_reg);
    float CH1 = getReg(Data1_reg);
    IntegrationTime integration_time = getIntegrationTime();
    ADCGain gain = getADCGain();
    float ChannelRatio = (float)CH1/CH0;

    // Adjust channel values for integration time
    switch (integration_time) {
	    case IntegrationTime::low_time:
		    CH0 *= 322.0/11;
		    CH1 *= 322.0/11;
		    break;
	    case IntegrationTime::medium_time:
		    CH0 *= 322.0/81;
		    CH1 *= 322.0/81;
		    break;
	    case IntegrationTime::high_time:
		    CH0 *= 322.0/322;
		    CH1 *= 322.0/322;
		    break;
	    case IntegrationTime::manual:
		    /* you're on your own dude. */
		    return NAN;
    }
    // Adjust for gain
    switch (gain) {
	    case ADCGain::low_gain:
		    break;
	    case ADCGain::high_gain:
		    CH0 /= 16;
		    CH1 /= 16;
		    break;
    }

    
    float Result = 0;
    if(ChannelRatio <= 0.5F)
        Result = (0.0304 * CH0) - ((0.062 * CH0) * pow(CH1/CH0, 1.4));
    else if(ChannelRatio <= 0.61F)
        Result = (0.0224 * CH0) - (0.031 * CH1);
    else if(ChannelRatio <= 0.80F)
        Result = (0.0128 * CH0) - (0.0153 * CH1);
    else if(ChannelRatio <= 1.30F)
        Result = (0.00146 * CH0) - (0.00112 * CH1);

    return Result;
}

//-------------------------------------------------------------------------------
unsigned LuxI2C_APDS9301::getChannel0()
{
    return(getReg(Data0_reg));
}

//-------------------------------------------------------------------------------
unsigned LuxI2C_APDS9301::getChannel1()
{
    return(getReg(Data1_reg));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setLowThreshold(unsigned newThreshold)
{
    setReg(ThresholdLow_reg,newThreshold);
}

//-------------------------------------------------------------------------------
unsigned LuxI2C_APDS9301::getLowThreshold()
{
    return(getReg(ThresholdLow_reg));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setHighThreshold(unsigned newThreshold)
{
    setReg(ThresholdHigh_reg,newThreshold);
}

//-------------------------------------------------------------------------------
unsigned LuxI2C_APDS9301::getHighThreshold()
{
    return(getReg(ThresholdHigh_reg));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::powerOff()
{
    setReg(Control_reg,0);
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::powerOn()
{
    setReg(Control_reg,3);
}

//-------------------------------------------------------------------------------
bool LuxI2C_APDS9301::isPowerOn()
{
    return((getReg(Control_reg) & 3) != 0);
}

//-------------------------------------------------------------------------------
LuxI2C_APDS9301::ADCGain LuxI2C_APDS9301::getADCGain()
{
    TimingRegister regv;

    regv.mbyte = getReg(Timing_reg);

    return(ADCGain(regv.mbits.adc_gain));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setADCGain(ADCGain newADCGain)
{
    TimingRegister regv;

    regv.mbyte = getReg(Timing_reg);
    regv.mbits.adc_gain = newADCGain;

    setReg(Timing_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
LuxI2C_APDS9301::IntegrationTime LuxI2C_APDS9301::getIntegrationTime()
{
    TimingRegister regv;

    regv.mbyte = getReg(Timing_reg);

    return(IntegrationTime(regv.mbits.integrate_time));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setIntegrationTime(IntegrationTime newIntegrationTime)
{
    TimingRegister regv;

    regv.mbyte = getReg(Timing_reg);
    regv.mbits.integrate_time = newIntegrationTime;

    setReg(Timing_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
bool LuxI2C_APDS9301::getManualIntegration()
{
    TimingRegister regv;

    regv.mbyte = getReg(Timing_reg);
    return(regv.mbits.manual_timing_control);
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setManualIntegrationOn()
{
    TimingRegister regv;

    regv.mbyte = getReg(Timing_reg);
    regv.mbits.manual_timing_control = 1;
    regv.mbits.integrate_time = manual;

    setReg(Timing_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setManualIntegrationOff()
{
    TimingRegister regv;

    regv.mbyte = getReg(Timing_reg);
    regv.mbits.manual_timing_control = 0;
    regv.mbits.integrate_time = manual;

    setReg(Timing_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setOutputInterruptOn()
{
    InterruptRegister regv;

    regv.mbyte = getReg(Interrupt_reg);
    regv.mbits.intr  = 1;

    setReg(Interrupt_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setOutputInterruptOff()
{
    InterruptRegister regv;

    regv.mbyte = getReg(Interrupt_reg);
    regv.mbits.intr = 0;

    setReg(Interrupt_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
bool LuxI2C_APDS9301::getOutputInterrupt()
{
    InterruptRegister regv;

    regv.mbyte = getReg(Interrupt_reg);
    return(regv.mbits.intr);
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::generateInterruptEveryADCCycle()
{
    InterruptRegister regv;

    regv.mbyte = getReg(Interrupt_reg);
    regv.mbits.persist = 0;

    setReg(Interrupt_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::generateInterruptAnyValueThresholdRange()
{
    InterruptRegister regv;

    regv.mbyte = getReg(Interrupt_reg);
    regv.mbits.persist = 1;

    setReg(Interrupt_reg,unsigned(regv.mbyte));
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setGenerateInterruptPeriod(uint8_t num_periods)
{
    InterruptRegister regv;

    if(num_periods >= 2 && num_periods <= 15)
    {
        regv.mbyte = getReg(Interrupt_reg);
        regv.mbits.persist = num_periods;

        setReg(Interrupt_reg,unsigned(regv.mbyte));
    }
}
//-------------------------------------------------------------------------------
uint8_t LuxI2C_APDS9301::getGenerateInterruptMode()
{
    InterruptRegister regv;

    regv.mbyte = getReg(Interrupt_reg);
    return(regv.mbits.persist);
}

//-------------------------------------------------------------------------------
uint8_t LuxI2C_APDS9301::getPartNum()
{
    IDRegister regv;

    regv.mbyte = getReg(ID_reg);
    return(regv.mbits.partnum);
}

//-------------------------------------------------------------------------------
uint8_t LuxI2C_APDS9301::getRevNum()
{
    IDRegister regv;

    regv.mbyte = getReg(ID_reg);
    return(regv.mbits.revno);
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::clearInterrupt()
{
    Wire.beginTransmission(m_u8I2CAddr);
    Wire.write(0xC0); // Reset Interrupt and select Control_reg
    Wire.endTransmission();
// make a fake read. I don't know if it is necessary but the datasheet
// say that the available operation are read or write and not only
// write Command reg
    Wire.requestFrom(m_u8I2CAddr, uint8_t(1));
    if(Wire.available())
        Wire.read();
}

//-------------------------------------------------------------------------------
unsigned LuxI2C_APDS9301::getReg(APDS9301Register reg)
{
    unsigned Result=0xFFFF;
    uint8_t word=0;
    switch(reg)
    {
    case Control_reg:
    case Timing_reg:
    case Interrupt_reg :
    case ID_reg        :
        word = 0;
        break;

    case ThresholdLow_reg :
    case ThresholdHigh_reg :
    case Data0_reg :
    case Data1_reg :
        word = 0x20;
        break;
    }

    Wire.beginTransmission(m_u8I2CAddr);
    Wire.write(0x80 | word | reg); // COMMAND without clear interrupt
    Wire.endTransmission();

    uint8_t c;

    Wire.requestFrom(m_u8I2CAddr, uint8_t(((word) ? 2:1)));
    if(Wire.available())
    {
        c = Wire.read();
        Result = c;
        if(word)
        {
            if(Wire.available())
            {
                c = Wire.read();
                Result |= (unsigned(c)<<8);
            }
            else
            {
#ifdef DEBUG
                Serial.println("Error ");
#endif
                Result = 0xFFFF;
            }
        }
    }
#ifdef DEBUG
    else
        Serial.println("Error");
#endif


#ifdef DEBUG
    Serial.print("getReg["); Serial.print(uint8_t(reg),HEX);Serial.print("]->");Serial.println(Result,HEX);
#endif
    return(Result);
}

//-------------------------------------------------------------------------------
void LuxI2C_APDS9301::setReg(APDS9301Register reg, unsigned newValue)
{
    uint8_t word=0;
#ifdef DEBUG
    Serial.print("setReg["); Serial.print(uint8_t(reg),HEX);Serial.print("]=");Serial.println(newValue,HEX);
#endif
    switch(reg)
    {
    case Control_reg:
    case Timing_reg:
    case Interrupt_reg :
    case ID_reg        :
        word = 0;
        break;

    case ThresholdLow_reg :
    case ThresholdHigh_reg :
    case Data0_reg :
    case Data1_reg :
        word = 0x20;
        break;
    }

    Wire.beginTransmission(m_u8I2CAddr);
    Wire.write(0x80 | word | reg); // pointer reg
    Wire.write(newValue & 0xFF);
    if(word)
    {
        Wire.write((newValue>>8) & 0xFF);
    }

    Wire.endTransmission();
}

