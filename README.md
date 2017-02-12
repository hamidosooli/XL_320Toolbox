XL_320Toolbox
========

#####Matlab Toolbox to control ROBOTIS Dynamixel XL_320 smart servo actuators with the OpenCM9.04 microcontroller.#####
######Version 1.0, 2-12-17######
Methods had been created by Dr. Federico Parietti for Dynamixel Pro

--------

First of all you will need to download the DXLTosser on the OpenCM9.04 

In ROBOTIS-v1.0.4 windows version Go to menu File > Example > DYNAMIXEL > Tosser

Download ROBOTIS software for OpenCM here http://support.robotis.com/en/software/robotis_opencm/robotis_opencm.htm

--------

Hamid Osooli, h.osooli @ std.kashanu.ac.ir

Created: 11-02-17

This version tested with MATLAB R2015a  

--------

Instructions

Add the XL_320Toolbox folder to the MATLAB path.

Open the file “XL_320_Example.m” to see some examples showing how to use the library functions to
write and read instructions to/from the Dynamixel XL_320 servos.

Thanks to the work by Dr. Federico Parietti All functions are extensively commented.

Notice that the library contains two kind of functions:
-	simple Write and Read functions: to control one Dynamixel XL_320  servo at a time; multiple servos can
still be controlled by the same code (by accessing them one by one), but this quickly increases computing
time;
-	Sync_Write and Sync-Read functions: to control multiple Dynamixel XL_320 servos at the same time; these
functions are a little more complicated, but they are essential if you need to control many servos with a faster sampling rate. 

--------

License

This work is provided with a GNU GPL v3.0 license (see attached file).

Remember to credit the author of the library when using this work.
