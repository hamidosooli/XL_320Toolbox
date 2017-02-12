%XL_320-Example  Simple script to demonstrate XL_320 class
%
%   First of all you will need to download the DXLTosser on the OPENCM
%	In ROBOTIS-v1.0.4 windows version Go to menu File > Example > DYNAMIXEL > Tosser 
%   Ensure that OpenCM9.04 microcontroller is connected via USB.
%   Actuators should be XL_320 actuators by default. 

%   Hamid Osooli, h.osooli @ std.kashanu.ac.ir
%   Created: 11-02-17

clc
clear all
close all

% Instantiate XL_320 class
xl = XL_320();

%Set the port parameters
s = serial('COM4');
set(s,'Baudrate',1000000);   %set speed of communication at 1'000'000 bps
set(s,'StopBits',1);         %specify number of bits used to indicate end of byte
set(s,'DataBits',8);         %number of data bits to transmit (we use 8 bit data)
set(s,'Parity','none');      %no parity

%Connect serial port
fopen(s);

%xl.read(1,xl.Address.PRESENT_POSITION,xl.Bytes.PRESENT_POSITION,s)
%xl.sync_read([1,2,3,4],xl.Address.PRESENT_POSITION,xl.Bytes.PRESENT_POSITION,s)
xl.write(1,xl.Address.GOAL_POSITION,600,xl.Bytes.GOAL_POSITION,s);
%xl.sync_write([1,2],xl.Address.GOAL_POSITION,[600,600],xl.Bytes.GOAL_POSITION,s);
%xl.sync_write_four([1,2,3,4],xl.Address.GOAL_POSITION,[600,600,600,600],xl.Bytes.GOAL_POSITION,s);

%Disconnect serial port
fclose(s);
delete(s)
clear s