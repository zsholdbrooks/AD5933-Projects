# File Name:    AD5933Interface.py
# Author:       Zack Holdbrooks
# Last edited:  March 28, 2020, 2:30 AM
# Description:  This file is a Python script/library that is focused upon testing the AD5933 impedance
#                 sensor with the Raspberry Pi. The script relies heavily upon SMBUS2 for I2C wrappers
#                 for the Raspberry Pi.
# 
#               The main script below initially asks for a nominal and actual tested calibration resistor
#                 value to calculate the calibration factor. This program is assuming that, during the
#                 calibration, both resistors are connected to the AD5933 as shown in the datasheet 
#                 without any additional circuitry. Regardless, the program will then ask for an expected
#                 value to test. It will continue to loop through within a given calibration set until 'q'
#                 is received. The next parameters for a new calibration set will then be requested to
#                 begin another measurement cycle. You can enter '0' for the calibration value to exit.
#  



from smbus2 import *
import csv
import os
import time
from math import atan, cos, degrees, floor, radians, sin, sqrt

# Important Notes
# Use smbus2 functions write_byte_data(Address, Register, Data)
#   and read_byte(Address) to conform with AD5933 functionality
# In smbus2 documentation, "with SMBusWrapper(1) as bus:" is used and recommended
#   for bus communication for automatically opening and closing the bus
# The number of samples does not include the start frequency, thus if you set 200 increments,
#   you will obtain 201 results. Also, if 0 or 1 increments are used, the device will always
#   return 2 results due to default internal programming before it recognizes the sweep as complete

AD5933_ADDRESS = 0x0D
AD5933_CONFIGBITS = 0x00 #D10 and D9 cleared for 2.0 Vpp excitation, PGA Gain cleared for x5

# AD5933 Registers
# With the exception of the control registers, the succeeding lower bit registers will be referred
#    to by using sum of the corresponding register and the offset,
#    e.g. the register containing the temperature data D7 to D0 will appear as TEMPERATURE_REGISTER + 1 for 0x93
CONREG1 = 0x80                  #Reg Data D15 to D8
CONREG2 = 0x81                  #Reg Data D7 to D0
START_FREQ_REG = 0x82           #Reg Data for starting frequency D23 to D16, 0x83 contains D15 to D8, 0x84 contains D7 to D0
FREQ_INCREMENT_REG = 0x85       #Reg Data for frequency increment D23 to D16, 0x86 contains D15 to D8, 0x87 contains D7 to D0
NUM_FREQ_INCREMENTS_REG = 0x88  #Reg Data for frequency number of frequency increments D15 to D8, 0x89 contains D7 to D0
NUM_SETTLING_TIME_CYCLES_REG = 0x8A  #Reg data for number of settling cycles D15 to D8, 0x8B contains D7 to D0
STATUS_REGISTER = 0x8F          #If D0 = 1, it indicates valid temperature measurements,
                                #If D1 = 1, it indicates valid real/imaginary data,
                                #If D2 = 1, it indicates frequency sweep completion    
TEMPERATURE_REGISTER = 0x92     #Temp Reg D15 to D8, 0x93 contains D7 to D0
REAL_IMPED_REG = 0x94           #Real impedance measured data D15 to D8, 0x95 contains D7 to D0
IMAG_IMPED_REG = 0x94           #Imaginary impedance measured data D15 to D8, 0x95 contains D7 to D0

#AD5933 Control Register Commands
#OR them with config bits to maintain control register integrity
INITIALIZE_WITH_START_FREQ = 0x10
START_FREQ_SWEEP = 0x20
INCREMENT_FREQ = 0x30
REPEAT_FREQ = 0x40
TEMPERATURE_COMMAND = 0x90
POWER_DOWN = 0xA0
STANDBY_MODE = 0xB0

#AD5933 Commands
POINTER_COMMAND = 0xB0
BLOCK_READ = 0xA1
BLOCK_WRITE = 0xA0

#Tolerance Macros
MAXIMUM_FREQ = 100000  #100 kHz
MINIMUM_FREQ = 1000    #1 kHz
MINIMUM_INC = 0.1      #0.1 Hz minimum without auxiliary circuitry

IMPEDANCE_DATA_OUTPUT_FILE = "tests " + time.strftime("%d-%m-%Y ") + time.strftime("%H-%M-%S") + ".csv"
SETTLING_TIME_CYCLES = 100
KNOWN_CALIBRATION_IMPEDANCE = 200000
CALIBRATED_SYSTEM_PHASE = []
CALIBRATION_NEEDED = 1

#Using Two point gain factor calculation, values are changed in calibration
AD5933_LOWER_GAIN_FACTOR = 1
AD5933_UPPER_GAIN_FACTOR = 1
AD5933_GAIN_FACTOR_DIFFERENCE = 1

############################################################
################  Primary AD5933 Functions  ################
############################################################

def ObtainTemperature ():
    validTemp = 0  #Initialize loop condition
    SetAddressPointer(STATUS_REGISTER) #Set up reading from status register
    with SMBusWrapper(1) as bus:
        bus.write_byte_data(AD5933_ADDRESS, CONREG1,                 #Sends command to initiate measuring temperature
                            TEMPERATURE_COMMAND | AD5933_CONFIGBITS) #Since the control register is written to, the configuration bits must be OR'd in order to maintain configuration
        while (not validTemp):
            validTemp = bus.read_byte(AD5933_ADDRESS) & 0x01  #And operation masks so only the valid temperature bit is observed in the Status register
        u16_rawRegData = (ReadRegister(TEMPERATURE_REGISTER) & 0x3F) << 8    #Collects the MSB from the first register and shifts the bits so the LSBs can be collected
        u16_rawRegData = u16_rawRegData | ReadRegister(TEMPERATURE_REGISTER + 1)   #Collects LSB from the next temperature register and ORs with MSB shifted string
    if (u16_rawRegData < 0x3800):   #Checks for positive temperature by using approximation mask from datasheet
        return (u16_rawRegData/32.0)  #Positive conversion
    else:
        return ((u16_rawRegData-16384)/32)  #negative conversion

"""Put in condition to calibrate everytime freq range changes or other settings"""
"""Calibrate needs to set gain factor and system phase array"""
"""do del CALIBRATED_SYSTEM_PHASE[:]"""
# Handle bad numbers before entering function, Go ahead and check for Resolution errors
def ObtainImpedanceProfile (Start_Freq, Resolution, Increments):
    global CALIBRATION_NEEDED
    ProcessAndSetParameters (Start_Freq, Resolution, Increments)
    ResetCommand = AD5933_CONFIGBITS | STANDBY_MODE
    StartCommand = INITIALIZE_WITH_START_FREQ | AD5933_CONFIGBITS
    StartSweepCommand = AD5933_CONFIGBITS | START_FREQ_SWEEP
    IncFreqCommand = AD5933_CONFIGBITS | INCREMENT_FREQ
    PowerDownCommand = AD5933_CONFIGBITS | POWER_DOWN
    DataList = []
    currentFrequency = Start_Freq
    SweepComplete = 0
    ValidData = 0
    iteration = 0
    with SMBusWrapper(1) as bus:
        bus.write_byte_data(AD5933_ADDRESS, CONREG1, ResetCommand)
        bus.write_byte_data(AD5933_ADDRESS, CONREG1, StartCommand)
        time.sleep(0.2)
        bus.write_byte_data(AD5933_ADDRESS, CONREG1, StartSweepCommand)
        while (not SweepComplete):
            time.sleep(1)
            time.sleep((5 + SETTLING_TIME_CYCLES)/currentFrequency)  #Sliding delay dependent upon frequency
            time.sleep(0.0015)
            while (not ValidData):
                ValidData = bus.read_byte(AD5933_ADDRESS) & 0x02 #Continue with same pointer
            ValidData = 0
            rawRegData = CollectRealAndImagData()
            DataList.append(ProcessData(Start_Freq, Resolution, Increments, iteration, rawRegData))
            print(iteration, hex(rawRegData[0]), TwosComplementConversion(rawRegData[0]),
                  "Real", hex(rawRegData[1]), TwosComplementConversion(rawRegData[1]), "Imag")
            bus.write_byte_data(AD5933_ADDRESS, CONREG1, IncFreqCommand)
            SweepComplete = ReadRegister(STATUS_REGISTER) & 0x04  #Masks to only compare valid data
            print()
            iteration += 1
        bus.write_byte_data(AD5933_ADDRESS, CONREG1, PowerDownCommand)
    if (CALIBRATION_NEEDED):
        CALIBRATION_NEEDED = 0
        global AD5933_GAIN_FACTOR_DIFFERENCE
        AD5933_GAIN_FACTOR_DIFFERENCE = AD5933_UPPER_GAIN_FACTOR - AD5933_LOWER_GAIN_FACTOR
    return

############################################################
###############  Auxiliary AD5933 Functions  ###############
############################################################

def SetAddressPointer (value):
    with SMBusWrapper(1) as bus:
        bus.write_byte_data(AD5933_ADDRESS, POINTER_COMMAND, value)

#The ReadRegister function provides usable functionality 
def ReadRegister (register): #Reads a byte from the AD5933 according to the procedure specified in the datasheet
    SetAddressPointer (register)  #Sets register pointer so AD5933 will retrieve data from desired register
    with SMBusWrapper(1) as bus:
        value = bus.read_byte(AD5933_ADDRESS) #Obtain transmitted byte from AD5933
    return value

def AD5933_Block_Read (start_reg, number_of_bytes):   #By default, don't cares are reported as 0's
    SetAddressPointer(start_reg)   #Set start register
    write = i2c_msg.write(AD5933_ADDRESS, [BLOCK_READ, number_of_bytes])  #Creates write object where the address and block commands are set for rdwr to interpret
    read = i2c_msg.read(AD5933_ADDRESS, number_of_bytes)   #Creates read object containing the address and expected number of bytes to obtain
    with SMBusWrapper(1) as bus:
        bus.i2c_rdwr(write, read)   #Executes the I2C data transfer by first sending the block commands in write then storing received data in the read object
        return (list(read))   #Returns the data obtained from the I2C transfer in a list with each byte being a single element

def AD5933_Block_Write (start_reg, data_vals, number_of_bytes):
    SetAddressPointer(start_reg)   #Set start register
    with SMBusWrapper(1) as bus:
        bus.write_i2c_block_data(AD5933_ADDRESS, BLOCK_WRITE, [number_of_bytes] + data_vals)   #Writes first the necessary intitiation commands and the succeeding data_vals stored in a list

def SetTimeSettlingCycles (value):   #Add in accounting for multipliers
    SETTLING_TIME_CYCLES = value
    with SMBusWrapper(1) as bus:
        bus.write_byte_data(AD5933_ADDRESS, NUM_SETTLING_TIME_CYCLES_REG + 1, (value & 0xFF))
        bus.write_byte_data(AD5933_ADDRESS, NUM_SETTLING_TIME_CYCLES_REG, ((value & 0x700) >> 8))

############################################################
##############  Data Manipulation Functions  ###############
############################################################    
    
def ProcessAndSetParameters (Start_Freq, Resolution, Increments):
    convertedHex = FreqConverter(Start_Freq)  #Sets up conversion for Starting Frequency Register
    LSB = convertedHex & 0xFF
    Middle = (convertedHex & 0xFF00) >> 8
    MSB = convertedHex >> 16
    startFreqList = [MSB, Middle, LSB]
    convertedHex = FreqConverter(Resolution)  #Sets up conversion for Frequency Increment Register
    LSB = convertedHex & 0xFF
    Middle = (convertedHex & 0xFF00) >> 8
    MSB = convertedHex >> 16
    incFreqList = [MSB, Middle, LSB]
    LSB = Increments & 0xFF
    MSB = Increments >> 8
    sampleList = [MSB, LSB]
    AD5933_Block_Write(0x82, startFreqList + incFreqList + sampleList, 8)
    
def CollectRealAndImagData(): #2's complement needs to be accounted for
    rawData = AD5933_Block_Read (REAL_IMPED_REG, 4)
    return ([(rawData[0] << 8) + rawData[1], (rawData[2] << 8) + rawData[3]])

def ProcessData(Start_Freq, Freq_Inc, Max_Increments, Iteration, Raw_Reg_Data):
    global AD5933_LOWER_GAIN_FACTOR
    global AD5933_UPPER_GAIN_FACTOR
    global AD5933_GAIN_FACTOR_DIFFERENCE
    twosRawReal = TwosComplementConversion(Raw_Reg_Data[0])
    twosRawImag = TwosComplementConversion(Raw_Reg_Data[1])
    magnitude = sqrt(twosRawReal ** 2 + twosRawImag ** 2)
    if (twosRawReal > 0):
        if (twosRawImag > 0):
            calculatedPhase = degrees(atan(twosRawImag/twosRawReal))   #First quadrant
        elif (twosRawImag < 0):
            calculatedPhase = degrees(atan(twosRawImag/twosRawReal)) + 180  #Second Quadrant
        else:
            calculatedPhase = 0
    elif (twosRawReal < 0):
        if (twosRawImag > 0):
            calculatedPhase = degrees(atan(twosRawImag/twosRawReal)) + 180  #Third Quadrant
        elif (twosRawImag < 0):
            calculatedPhase = degrees(atan(twosRawImag/twosRawReal)) + 360  #Fourth Quadrant
        else:
            calculatedPhase = 180
    else:
        if (twosRawImag > 0):
            calculatedPhase = 90
        else:
            calculatedPhase = 270
    if (CALIBRATION_NEEDED):
        CALIBRATED_SYSTEM_PHASE.append(calculatedPhase)
        AD5933CalibrationFactor = 1/(magnitude * KNOWN_CALIBRATION_IMPEDANCE)
        if (Iteration == 0):
            AD5933_LOWER_GAIN_FACTOR = AD5933CalibrationFactor
        elif (Iteration == Max_Increments):
            AD5933_UPPER_GAIN_FACTOR = AD5933CalibrationFactor
        with open(IMPEDANCE_DATA_OUTPUT_FILE, "a", newline="") as csvfile:
            fieldnames = ["Frequency", "Real Data", "Real Two's Comp", "Imaginary Data", "Imag Two's Comp",
                          "Magnitude", "Phase Angle", "Gain Factor"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if Iteration == 0:
                writer.writeheader()    
            writer.writerow({"Frequency": Start_Freq + Iteration * Freq_Inc,
                             "Real Data": Raw_Reg_Data[0], "Real Two's Comp": twosRawReal,
                             "Magnitude": magnitude, "Phase Angle": calculatedPhase,
                             "Imaginary Data": Raw_Reg_Data[1], "Imag Two's Comp": twosRawImag,
                             "Gain Factor": AD5933CalibrationFactor})
        return
    else:
        #Handle potential div by 0 exception below
        adjustedTwoPointGain = AD5933_LOWER_GAIN_FACTOR + AD5933_GAIN_FACTOR_DIFFERENCE * Iteration / Max_Increments
        calculatedImped = 1/(adjustedTwoPointGain * magnitude)
        calculatedPhase = calculatedPhase - CALIBRATED_SYSTEM_PHASE[Iteration]
        calculatedResist = calculatedImped * cos(radians(calculatedPhase))
        calculatedReact = calculatedImped * sin(radians(calculatedPhase))

        print("Calculated Impedance #", Iteration, calculatedImped)
        with open(IMPEDANCE_DATA_OUTPUT_FILE, "a", newline="") as csvfile:
            fieldnames = ["Frequency", "Real Data", "Real Two's Comp", "Imaginary Data", "Imag Two's Comp",
                          "Magnitude", "Phase Angle", "Resistance", "Reactance", "Impedance", "Used Gain"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if Iteration == 0:
                writer.writeheader()    
            writer.writerow({"Frequency": Start_Freq + Iteration * Freq_Inc,
                             "Real Data": Raw_Reg_Data[0], "Real Two's Comp": twosRawReal,
                             "Magnitude": magnitude, "Phase Angle": calculatedPhase,
                             "Imaginary Data": Raw_Reg_Data[1], "Imag Two's Comp": twosRawImag,
                             "Resistance": calculatedResist, "Reactance": calculatedReact,
                             "Impedance": calculatedImped, "Used Gain": adjustedTwoPointGain})
        return

def FreqConverter(Freq):
    converted = ((Freq)*((2**27)/(4000000)))
    return (floor(converted))

def TwosComplementConversion (rawValue):
    if (rawValue < 0x8000):        #If rawValue doesn't have the MSB set, return original value
        return rawValue
    else:                          #else convert it to 2's complement by using rawValue - (2*(2^15))
        return (rawValue - 65536)

def Average(x):
    sum = 0
    length = len(x)
    for y in range(length):
        sum += x[y]
    return (sum/length)

def Aggregate (func, number):
    a = []
    for x in range(number):
        a.append(func)
    print(a)
    print("Max:",max(a))
    print("Min:",min(a))
    print("Average:", Average(a))


###################################################
################### Main Script ###################
###################################################

SetTimeSettlingCycles(128)         #Sets 128 cycles between measurements

configcondition = 2
if (configcondition == 1):
    AD5933_CONFIGBITS = 0x01           #Vpp 2.0, PGA x1
elif (configcondition == 2):
    AD5933_CONFIGBITS = 0x03           #Vpp 0.2, PGA x1
elif (configcondition == 3):
    AD5933_CONFIGBITS = 0x05           #Vpp 0.4, PGA x1
elif (configcondition == 4):
    AD5933_CONFIGBITS = 0x07           #Vpp 1.0, PGA x1
elif (configcondition == 5):
    AD5933_CONFIGBITS = 0x00           #Vpp 2.0, PGA x5
elif (configcondition == 6):
    AD5933_CONFIGBITS = 0x02           #Vpp 0.2, PGA x5
elif (configcondition == 7):
    AD5933_CONFIGBITS = 0x04           #Vpp 0.4, PGA x5
elif (configcondition == 8):
    AD5933_CONFIGBITS = 0x06           #Vpp 1.0, PGA x5

calibtitle = input("Calibration short hand?")
calibration = input("Calibration measured value?")
testvalue = input("calibration test val?")
measured = input("Measured value?")

StartFreq = 40000   # Starting frequency in Hz
FreqIncrement = 200 # Frequency increments from succeeding measurements in Hz
NumIncrements = 100 # Number of points in a frequency sweeps
# Note: End frequency = StartFreq + (FreqIncrement * NumIncrements)

while (calibration != "0"):
    CALIBRATION_NEEDED = 1     # Sets flag for calibration at beginning of each loop
    KNOWN_CALIBRATION_IMPEDANCE = float(calibration)   # Casts input from calibration request to float
    os.mkdir(calibtitle + " calibration")              # Creates a directory corresponding to calibration set

    # Loops for the same given calibration set
    while (testvalue != "q"):
        if (CALIBRATION_NEEDED):
            print("Calibration is needed")
            # Modifies data file global that is used for CSV in the ProcessData command
            IMPEDANCE_DATA_OUTPUT_FILE = calibtitle + " calibration/Calibration Data"
            ObtainImpedanceProfile(StartFreq, FreqIncrement, NumIncrements)
            # Sets up next measurement sweep
            testvalue = input("Resistor Value?")
            measured = input("Measured value?")
            continue   # Moves back to the beginning of the loop

        IMPEDANCE_DATA_OUTPUT_FILE = calibtitle + " calibration/" + testvalue + " data"
        # Heads CSV file with nominal and multimeter tested value for impedance under frequency sweep
        with open(IMPEDANCE_DATA_OUTPUT_FILE, "a", newline="") as csvfile:
            fieldnames = ["Resistor value", "Measured"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow({"Resistor value": testvalue, "Measured": measured})    
        ObtainImpedanceProfile(StartFreq, FreqIncrement, NumIncrements)
        print("Profile Obtained")
        testvalue = input("Next Resistor Value?")
        measured = input("Measured value?")
    print("Cycle Complete")
    # Sets up loop for new calibration test set
    calibtitle = input("Calibration short hand?")
    calibration = input("Calibration value?")
    testvalue = input("calibration test val?")
    measured = input("Measured value?")