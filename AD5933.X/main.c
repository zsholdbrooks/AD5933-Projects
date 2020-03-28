/**
  MAIN MANAGER Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    main manager.c

  @Summary
    This is the generated driver implementation file for the MAIN MANAGER driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for MAIN MANAGER.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.75
        Device            :  PIC24FJ64GB002
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.35
        MPLAB 	          :  MPLAB X v5.05
*/

/* Program Description:
 *  This project for direct usage of the PIC24 with the AD5933 is intended to be
 *  a very simple setup where a Microstick II or similar evaluation board for the PIC24
 *  can be connected to the AD5933 and 4 LEDs for feedback. As this is a project scavenged
 *  from a larger research project, it is very barebones and has been changed to focus only
 *  upon the usage of the AD5933 and the PIC24. The PIC24 is connected to LEDs on RA0, RB0,
 *  RB1, and RB2. The I2C lines for the AD5933 are connected to SCL1 and SDA1.
 * 
 * 
 *  Calibration: The CALIBRATION define will first need to be changed to 1 to enable the
 *               preprocessor if statements to take effect. The KNOWNIMPEDANCE define must
 *               be changed as well to the intended calibration resistance (in ohms) for
 *               the correct calibration factor to be calculated. A debugging session may
 *               be then run with a watch for "calculatedImpedance" which will contain the
 *               calibration factor once it hits the while loop.
 * 
 * Threshold Modifications:
 *      If the threshold needs to be changed, find the three macros below labeled
 *      HIGH_THRESHOLD_PERCENTAGE, MIDDLE_THRESHOLD_PERCENTAGE, & LOW_THRESHOLD_PERCENTAGE
 *      and replace the number next to it with the desired percent threshold.
 *      e.g. If the desired threshold is 50%, the correct multiplier would be 1.50
 */

//Section: Included Files

#define FCY 4000000UL // Needs to be specified for delay functions

#define CALIBRATION 0
#define KNOWNIMPEDANCE 100000 // 10kOhm expressed in ohms

#define Z_RISE 0
#define Z_DROP 1

#define UPPER_HIGH_THRESHOLD_PERCENTAGE 1.75   // 75% increase
#define UPPER_MIDDLE_THRESHOLD_PERCENTAGE 1.50 // 50% increase
#define UPPER_LOW_THRESHOLD_PERCENTAGE 1.25    // 25% increase

#define LOWER_LOW_THRESHOLD_PERCENTAGE 0.50    // 50% decrease
#define LOWER_MIDDLE_THRESHOLD_PERCENTAGE 0.65 // 35% decrease
#define LOWER_HIGH_THRESHOLD_PERCENTAGE 0.80   // 20% decrease

#include <xc.h>
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/i2c1.h"

#define SLAVE_I2C_GENERIC_RETRY_MAX           100
#define SLAVE_I2C_GENERIC_DEVICE_TIMEOUT      50   // define slave timeout

#define AD5933_I2C_DELAY                      50 //Microseconds
#define AD5933_BLOCK_WRITE                    0xA0
#define AD5933_BLOCK_READ                     0xA1
#define AD5933_ADDRESS_POINTER                0xB0

uint8_t readBuffer[8];

/*
 * To write to the device, an array must be prepared and passed to the function below.
 * A simple write would be dataList = {register, dataByte} as there are ack bits between
 * the successive bytes written with no restart and a stop finishing out the transaction.
 * 
 * A more complex operation would be for the AD5933 Block Write where attempting to
 * write nCount elements would result in 
 *          dataList = {AD5933_BLOCK_WRITE, nCount, 0x1, 0x2, ...., nCount + 2}
 *              where 0x1, 0x2, and on to nCount + 2 are the actuall data bytes
 */
uint8_t WriteDataToDev (uint8_t dev_addr, uint8_t *dataList, uint8_t listCount) {
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING; 
    I2C1_MasterWrite(dataList, listCount, dev_addr, &status);

    // wait for the message to be sent or status has changed.
    while(status == I2C1_MESSAGE_PENDING)
    {
        // add some delay here
        __delay_us(AD5933_I2C_DELAY);
        // timeout checking
        // check for max retry and skip this byte        
    } 
    if ((status == I2C1_MESSAGE_FAIL)) {
        return 0;
    }
    if ((status == I2C1_MESSAGE_ADDRESS_NO_ACK)) {
        return 0;
    }
    if ((status == I2C1_MESSAGE_COMPLETE)) {
        return 1;
    }
    return 0;
}

// Must initialize an array and pass it as the readList so that the data can be
// read as an array from the passed parameter list. The function returns a success
// value for completion or failure
uint8_t ReadByteFromDev (uint8_t dev_addr, uint8_t *readList) {
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
    I2C1_MasterRead(readList, 2, dev_addr, &status);
    
    // wait for the message to be sent or status has changed.
    while(status == I2C1_MESSAGE_PENDING)
    {
        // add some delay here
        __delay_us(AD5933_I2C_DELAY);
        // timeout checking
        // check for max retry and skip this byte        
    } 
    if ((status == I2C1_MESSAGE_FAIL)) {
        return 0;
    }
    if ((status == I2C1_MESSAGE_ADDRESS_NO_ACK)) {
        return 0;
    }
    if ((status == I2C1_MESSAGE_COMPLETE)) {
        return 1;
    }
    return 0;
}

uint8_t ReadBlockFromAD5933 (uint8_t dev_addr, uint8_t *readList, uint8_t listCount, uint8_t startReg) {
    uint8_t *readBuffer = readList;
    uint8_t writeBuffer[2] = {AD5933_ADDRESS_POINTER, startReg};
    uint8_t i = 0;
    uint8_t retryCount = 0;
    for (i = 0; i < listCount; i++) {
        while( !(WriteDataToDev (dev_addr, writeBuffer, 2))) {
            if (retryCount > 3) { return 0; }
            retryCount++;
        }
        retryCount = 0;
        while( !(ReadByteFromDev(dev_addr, readBuffer))) {
                if (retryCount > 3) { return 0; }
            retryCount++;
        }
        retryCount = 0;
        writeBuffer[1]++;
        readBuffer++;
    } 
    return 1;
}

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize(); 
    
    while (1)
    {
        // initialize the module
        I2C1_Initialize();
        
        TRISB = 0xEFB8;
        
        // Acts as startup sequence across LEDs to verify start
        LATA = 0x01;
        __delay_ms(250);
        LATB = 0x01;
        __delay_ms(250);
        LATB = 0x03;
        __delay_ms(250);
        LATB = 0x07;
        __delay_ms(250);
        LATB = 0x06;
        __delay_ms(250);
        LATB = 0x04;
        __delay_ms(250);
        LATB = 0x00;
        
        // Command Arrays
        uint8_t ResetArray[2] = {0x80, 0xB3};
        uint8_t InitWithStartFreqArray[2] = {0x80, 0x13};
        uint8_t StartFreqSweepArray[2] = {0x80, 0x23};
        uint8_t IncFreqArray[2] = {0x80, 0x33};
        uint8_t PowerDownArray[2] = {0x80, 0xA0};
        uint8_t StatusPointer[2] = {AD5933_ADDRESS_POINTER, 0x8F};
        
        // The data being sent below is as follows: StartFreq (0x05, 0x11, 0x9C) which is 9900Hz, 
        //      Freq increment (0x00, 0x0D, 0x1B) which corresponds to 100Hz,
        //      # of increments (0x00, 0x0A) which is 10,
        //      & excitation cycles (0x00, 0xC0) which is 192
        uint8_t SettingsPointer[2] = {AD5933_ADDRESS_POINTER, 0x82};
        uint8_t ParamSettings[12] = {AD5933_BLOCK_WRITE, 10, 0x05, 0x11, 0x9C, 0x00, 0x0D, 0x1B, 0x00, 0x0A, 0x00, 0xC0}; 
        
        uint8_t statusData[1] = {0};
        uint8_t RegResult[4] = {0, 0, 0, 0};
        double RealRegData = 0;
        double ImagRegData = 0;
        uint8_t iteration = 0;
        double calculatedImpedance = 0;
        double impedSum = 0;
        double impedAverage = 0;
        double imagSq = 0;
        double realSq = 0;
        double gainFactor = 4.792225E-9; //Calibrated for 100kOhm @ 50kHz with an excitation of 400mVpp

        double initialImpedance = 1;
                
        // Program AD5933 with settings once
        WriteDataToDev(0x0D, SettingsPointer, 2);
        WriteDataToDev(0x0D, ParamSettings, 12);
        
        // statusData & 0x04 for Sweep Complete
        // statusData & 0x02 for valid data
        while (1) {
            // Program reset to conreg1
            WriteDataToDev(0x0D, ResetArray, 2);
            // Program Init Start Freq
            WriteDataToDev(0x0D, InitWithStartFreqArray, 2);
            __delay_ms(20); // Delay for settling time cycles was 20
            // Program Start Freq
            WriteDataToDev(0x0D, StartFreqSweepArray, 2);
            while (!(statusData[0] & 0x04)) {
                //Set pointer to status reg
                WriteDataToDev(0x0D, StatusPointer, 2);
                ReadByteFromDev(0x0D, statusData); // Collect status
                while (!(statusData[0] & 0x02)) {
                    //Poll status reg and delay
                    __delay_us(200);
                    WriteDataToDev(0x0D, StatusPointer, 2);
                    ReadByteFromDev(0x0D, statusData); // Obtain Status data
                }
                //Pull Data from Data Regs
                ReadBlockFromAD5933 (0x0D, RegResult, 4, 0x94); // Read from Real and Imag
                RealRegData = (RegResult[0] << 8) + RegResult[1];
                if (RealRegData >= 0x8000) {    // Twos Complement
                    RealRegData -= 65536;
                }
                realSq = RealRegData * RealRegData;
                ImagRegData = (RegResult[2] << 8) + RegResult[3];
                if (ImagRegData >= 0x8000) {    // Twos Complement
                    ImagRegData -= 65536;
                }
                imagSq = ImagRegData * ImagRegData;
                calculatedImpedance = sqrt(realSq + imagSq);
                if (calculatedImpedance != 0) {
#if CALIBRATION
                    // Can obtain impedance calibration from breakpoint debugging in MPLAB X
                    calculatedImpedance = 1/(calculatedImpedance * KNOWNIMPEDANCE);  
#else
                    calculatedImpedance = 1/(calculatedImpedance * gainFactor);
#endif
                }
                // Interpret data by calculating impedance
                // testResults[iteration] = calculatedImpedance;
                // Add to Sum
                iteration += 1;
                impedSum += calculatedImpedance;
                if (!(statusData[0] & 0x04)) {
                    WriteDataToDev(0x0D, IncFreqArray, 2);
                }
            }
            //Program Power Down
            WriteDataToDev(0x0D, PowerDownArray, 2);
            if (iteration) {
                impedAverage = impedSum/iteration;
            }
#if CALIBRATION
            //check impedAverage for Calibration factor
            while (1);
#endif
            if (initialImpedance == 1) {
                initialImpedance = calculatedImpedance;
            }
#if Z_RISE
            else {
                if (impedAverage >= (initialImpedance * UPPER_HIGH_THRESHOLD_PERCENTAGE)) {  // if above absolute highest ratio
                    _LATB2 = 1;
                }
                else if (impedAverage >= (initialImpedance * UPPER_MIDDLE_THRESHOLD_PERCENTAGE)) {
                    _LATB1 = 1;
                }
                else if ((impedAverage >= (initialImpedance * UPPER_LOW_THRESHOLD_PERCENTAGE)) {
                    _LATB0 = 1;
                }
            }
#elif Z_DROP
            else {
               if (impedAverage <= (initialImpedance * LOWER_LOW_THRESHOLD_PERCENTAGE)) {  // if below absolute lowest ratio
                    _LATB2 = 1;
                }
                else if (impedAverage <= (initialImpedance * LOWER_MIDDLE_THRESHOLD_PERCENTAGE)){
                    _LATB1 = 1;
                }
                else if (impedAverage <= (initialImpedance * LOWER_HIGH_THRESHOLD_PERCENTAGE)){
                    _LATB0 = 1;
                }
            }
#endif

            // Power LED Blinks to signify mesasurement cycle has been completed
            _LATA0 = 0;
            __delay_ms(300);
            _LATA0 = 1;
            impedSum = 0;
            statusData[0] = 0;
            iteration = 0;
        }
        
    }
    return 1;
}
/**
 End of File
*/

