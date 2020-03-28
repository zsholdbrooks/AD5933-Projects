# AD5933-Projects

This repository includes resources that I developed while using the AD5933 impedance sensor.

The python script "AD5933Interface.py" is a script intended for use on the Raspberry Pi as a
point of high level interaction without the need for a microcontroller. Once data is obtained,
it can very quickly be analyzed in Raspbian directly as a CSV. The SMBus2 library (used for I2C
communication wrappers) is the only dependency that must be installed outside of the natively
supported python libraries.

The "AD5933.X" directory contains the MPLAB X project that I developed as an example for using
the AD5933 with a microcontroller. The project is using specifically the PIC24FJ64GB002 (one of
the PIC24 chips included with the Microstick 2 development kit), but can be reconfigured as long
as the MCC pregenerated I2C libraries are regenerated for the new PIC device. Otherwise, it is a
simple bare metal microcontroller program where pins are set high after a certain percentage
change is made from the initial measured impedance.
