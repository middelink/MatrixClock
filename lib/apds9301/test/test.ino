/*
 Copyright © 2014 José Luis Zabalza  License LGPLv3+: GNU
 LGPL version 3 or later <http://www.gnu.org/copyleft/lgpl.html>.
 This is free software: you are free to change and redistribute it.
 There is NO WARRANTY, to the extent permitted by law.
*/


#define VERSION "1.1"

#include <inttypes.h>

#include <apds9301.h>

LuxI2C_APDS9301 lux = LuxI2C_APDS9301(0x39);

static volatile int status = LOW;

void Menu();
void ClearScreen();
void ShowGenerateInterrupt();
void ShowIntegrationTime();

void interfunc(void)
{
    status = !status;
    digitalWrite(13,status);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Start");

    pinMode(13, OUTPUT);

    attachInterrupt(0, interfunc, FALLING) ;
    Menu();
}

void loop()
{
    char c;
    static char val[6];
    static uint8_t index;
    static uint8_t command;

    if(Serial.available())
    {
        c = Serial.read();
// Getting a numerical value
        if(command)
        {
            if(c != '\r')
            {
                if(c >= '0' || c <= '9')
                {
                    val[index++] = c;
                    Serial.print(c);
                }
            }
            else
            {
// End numerical value capture
                val[index] = '\0';
                long iVal = atol(val);

// Value error condition
                if( (command == 'F' || command == 'f')
                                 &&
                    (iVal < 2 || iVal > 15))
                {
                    Serial.println("Error. 2 <= Value <= 15");
                }
                else if((command == '2' || command == '3')
                                &&
                          (iVal > 65535U))
                {
                    Serial.println("Error. Value <= 65535");
                }
                else
                {
// Value is OK. Execute command
                    ClearScreen();
                    Serial.println("----------------------------------------------");
                    if(command == 'F' || command == 'f')
                    {
                        lux.setGenerateInterruptPeriod(iVal);
                        Serial.print(" Generate interrupt period ");Serial.println(lux.getGenerateInterruptMode());
                    }
                    else if(command == '2')
                    {
                        lux.setHighThreshold(iVal);
                        Serial.print(" High threshold is ");Serial.println(lux.getHighThreshold());
                    }
                    else
                    {
                        lux.setLowThreshold(iVal);
                        Serial.print(" Low threshold is ");Serial.println(lux.getLowThreshold());
                    }
                    Serial.println("----------------------------------------------");
                }
// Go to menu
                command = index = 0;
                Menu();
            }
// out of index error condition
            if(((command == 'F' || command == 'f') && index > 2)
                    ||
               ((command == '2' || command == '3') && index > 5)
                    ||
                    c == 27)
            {
                    Serial.println("");
                    Serial.println("----------------------------------------------");
                    if(command == 'F' || command == 'f')
                        Serial.println("Error. 2 <= Value <= 15");
                    else
                        Serial.println("Error. Value <= 64545");
                    Serial.println("----------------------------------------------");
                    command = index = 0;
                    Menu();
            }
        }
        else
        {
// Getting menu option
            ClearScreen();
            switch(c)
            {
            case '0':
                lux.powerOff();
                Serial.println("----------------------------------------------");
                Serial.print("   Power is ");Serial.println((lux.isPowerOn()) ? "ON" : "OFF");
                Serial.println("----------------------------------------------");
                Menu();

                break;
            case '1':
                lux.powerOn();
                Serial.println("----------------------------------------------");
                Serial.print("   Power is ");Serial.println((lux.isPowerOn()) ? "ON" : "OFF");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case '2':
            case '3':
            {
                Serial.println("----------------------------------------------");
                Serial.println(" Input range 0 <= value <= 65535");
                Serial.println("----------------------------------------------");

                command = c;
                index = 0;
            }
                break;
            case 'F':
            case 'f':
            {
                Serial.println("----------------------------------------------");
                Serial.println(" Input range 2 <= value <= 15");
                Serial.println("----------------------------------------------");

                command = c;
                index = 0;
            }

                break;
            case '4':
                lux.setOutputInterruptOn();
                Serial.println("----------------------------------------------");
                Serial.print(" Output Interrupt is ");Serial.println((lux.getOutputInterrupt()) ? "ON" : "OFF");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case '5':
                lux.setOutputInterruptOff();
                Serial.println("----------------------------------------------");
                Serial.print(" Output Interrupt is ");Serial.println((lux.getOutputInterrupt()) ? "ON" : "OFF");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case '6':
                lux.generateInterruptEveryADCCycle();
                Serial.println("----------------------------------------------");
                ShowGenerateInterrupt();
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case '7':
                lux.generateInterruptAnyValueThresholdRange();
                Serial.println("----------------------------------------------");
                ShowGenerateInterrupt();
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case '8':
                lux.setManualIntegrationOn();
                Serial.println("----------------------------------------------");
                Serial.print(" Manual integration is ");Serial.println((lux.getManualIntegration()) ? "ON" : "OFF");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case '9':
                lux.setManualIntegrationOff();
                Serial.println("----------------------------------------------");
                Serial.print(" Manual integration is ");Serial.println((lux.getManualIntegration()) ? "ON" : "OFF");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case 'A':
            case 'a':
                lux.setADCGain(LuxI2C_APDS9301::high_gain);
                Serial.println("----------------------------------------------");
                Serial.print(" ADC Gain is "); Serial.println((lux.getADCGain()) ? "x16" : "x1");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case 'B':
            case 'b':
                lux.setADCGain(LuxI2C_APDS9301::low_gain);
                Serial.println("----------------------------------------------");
                Serial.print(" ADC Gain is "); Serial.println((lux.getADCGain()) ? "x16" : "x1");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case 'C':
            case 'c':
                lux.setIntegrationTime(LuxI2C_APDS9301::low_time);
                Serial.println("----------------------------------------------");
                ShowIntegrationTime();
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case 'D':
            case 'd':
                lux.setIntegrationTime(LuxI2C_APDS9301::medium_time);

                ShowIntegrationTime();
                Menu();
                break;
            case 'E':
            case 'e':
                lux.setIntegrationTime(LuxI2C_APDS9301::high_time);
                ShowIntegrationTime();
                Menu();
                break;
            case 'I':
            case 'i':
                Serial.println("----------------------------------------------");
                Serial.print("Part Num="); Serial.print(lux.getPartNum());
                Serial.print("  Rev Num="); Serial.println(lux.getRevNum());
                Serial.print("Power is ");Serial.println((lux.isPowerOn()) ? "ON" : "OFF");
                Serial.print("High threshold is ");Serial.println(lux.getHighThreshold());
                Serial.print("Low threshold is ");Serial.println(lux.getLowThreshold());
                Serial.print("Output Interrupt is ");Serial.println((lux.getOutputInterrupt()) ? "ON" : "OFF");
                ShowGenerateInterrupt();
                Serial.print("Manual integration is ");Serial.println((lux.getManualIntegration()) ? "ON" : "OFF");
                Serial.print("ADC Gain is "); Serial.println((lux.getADCGain()) ? "x16" : "x1");
                ShowIntegrationTime();
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case 'G':
            case 'g':
                lux.clearInterrupt();
                Serial.println("----------------------------------------------");
                Serial.println("Interrupt is clear");
                Serial.println("----------------------------------------------");
                Menu();
                break;
            case ' ':
                Serial.println("----------------------------------------------");
                Serial.print("Channel 0 ="); Serial.println(lux.getChannel0());
                Serial.print("Channel 1 ="); Serial.println(lux.getChannel1());
                Serial.print(lux.getLux());Serial.println(" luxes");
                Serial.println("----------------------------------------------");
                lux.clearInterrupt();
                Menu();
                break;
            }
        }
    }
}
void ClearScreen()
{
    uint8_t v[]={27,'[','2','J'};
    Serial.write(v,4);
}

void ShowGenerateInterrupt()
{
    uint8_t per=lux.getGenerateInterruptMode();
    switch(per)
    {
    case 0: Serial.println("Generate Interrupt each ADC cycle"); break;
    case 1: Serial.println("Generate interrupt measure are out of range"); break;
    default:Serial.print("Generate Interrupt measure are out of range for "); Serial.print(per); Serial.println(" cycles"); break;
    }
}

void ShowIntegrationTime()
{
    Serial.print("Integration time is ");
    switch(lux.getIntegrationTime())
    {
    case LuxI2C_APDS9301::high_time:Serial.println("high");break;
    case LuxI2C_APDS9301::medium_time:Serial.println("medium");break;
    case LuxI2C_APDS9301::low_time:Serial.println("low");break;
    case LuxI2C_APDS9301::manual:Serial.println("manual");break;
    }
}

void Menu()
{
    Serial.println("APDS9301 Test Aplication");
    Serial.println("========================");
    Serial.println("0.- Power Off                            1.- Power On");
    Serial.println("2.- Set threshold High                   3.- Set threshold Low");
    Serial.println("4.- Set Output Interrupt ON              5.- Set Output Interrupt OFF");
    Serial.println("6.- Generate Interrupt every cycle       7.- Generate Interrupt outside margins   ");
    Serial.println("8.- Set manual integration time On       9.- Set manual integration time Off");
    Serial.println("A.- Set ADC Gain High                    B.- Set ADC Gain Low");
    Serial.println("C.- Set Integrate time low               D.- Set Integrate time medium");
    Serial.println("E.- Set Integrate time high              F.- Set interrupt cycles");
    Serial.println("G.- clear Interrupt                      I.- Info");
    Serial.println("                       SPACE make a capture");

}
