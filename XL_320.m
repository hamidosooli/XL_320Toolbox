classdef XL_320
    %XL_320 Create XL_320 object to communicate with OpenCM9.04 and Dynamixels
    %   
    %   Methods:
    %       read                 - Sends an Instruction Packet to read the desired value at the specified address on the Dynamixel XL_320
    %       write                - Writes the desired value at the specified address on the Dynamixel XL_320
    %       sync_read            - Reads the desired values at the specified address on the Dynamixel XL_320 servos, simultaneously (works with any number of servos)
    %       sync_write           - Writes the desired values at the specified address on the Dynamixel XL_320 servos, simultaneously (works with 2 servos)
    %       sync_write_four      - Writes the desired values at the specified address on the Dynamixel XL_320 servos, simultaneously (works with 4 servos)
    %       
    %   Properties (Read-only):
    %       Address              - Structure of Dynamixel addresses.
    %       Bytes                - Size of Dynamixel addresses in Byte.
    
    %   Hamid Osooli, h.osooli @ std.kashanu.ac.ir
    %   Created: 11-09-16
    %   Methods had been created by Federico Parietti for Dynamixel Pro
   properties (Constant)
      % Dynamixel XL_320 hexadecimal register addresses
      Address = struct('MODEL_NUMBER',                   0,...
                       'VERSION_OF_FIRMWARE',            2,...
                       'ID',                             3,...
                       'BAUD_RATE',                      4,...
                       'RETURN_DELAY_TIME',              5,...
                       'CW_ANGLE_LIMIT',                 6,...
                       'CCW_ANGLE_LIMIT',                8,...
                       'CONTROL_MODE',                   11,...
                       'LIMIT_TEMPERATURE',              12,...
                       'LOWER_LIMIT_VOLTAGE',            13,...
                       'UPPER_LIMIT_VOLTAGE',            14,...
                       'MAX_TORQUE',                     15,...
                       'RETURN_LEVEL',                   17,...
                       'ALARM_SHUTDOWN',                 18,...
                       'TORQUE_ENABLE',                  24,...
                       'LED',                            25,...
                       'D_GAIN',                         27,...
                       'I_GAIN',                         28,...
                       'P_GAIN',                         29,...
                       'GOAL_POSITION',                  30,...
                       'GOAL_VELOCITY',                  32,...
                       'GOAL_TORQUE',                    35,...
                       'PRESENT_POSITION',               37,...
                       'PRESENT_SPEED',                  39,...
                       'PRESENT_LOAD',                   41,...
                       'PRESENT_VOLTAGE',                45,...
                       'PRESENT_TEMPERATURE',            46,...
                       'REGISTERED_INSTRUCTION',         47,...
                       'MOVING',                         49,...
                       'HARDWARE_ERROR_STATUS',          50,...
                       'PUNCH',                          51);
                   
      % Dynamixel XL_320 address sizes in BYTE
      Bytes = struct('MODEL_NUMBER',                   2,...
                     'VERSION_OF_FIRMWARE',            1,...
                     'ID',                             1,...
                     'BAUD_RATE',                      1,...
                     'RETURN_DELAY_TIME',              1,...
                     'CW_ANGLE_LIMIT',                 2,...
                     'CCW_ANGLE_LIMIT',                2,...
                     'CONTROL_MODE',                   1,...
                     'LIMIT_TEMPERATURE',              1,...
                     'LOWER_LIMIT_VOLTAGE',            1,...
                     'UPPER_LIMIT_VOLTAGE',            1,...
                     'MAX_TORQUE',                     2,...
                     'RETURN_LEVEL',                   1,...
                     'ALARM_SHUTDOWN',                 1,...
                     'TORQUE_ENABLE',                  1,...
                     'LED',                            1,...
                     'D_GAIN',                         1,...
                     'I_GAIN',                         1,...
                     'P_GAIN',                         1,...
                     'GOAL_POSITION',                  2,...
                     'GOAL_VELOCITY',                  2,...
                     'GOAL_TORQUE',                    2,...
                     'PRESENT_POSITION',               2,...
                     'PRESENT_SPEED',                  2,...
                     'PRESENT_LOAD',                   2,...
                     'PRESENT_VOLTAGE',                1,...
                     'PRESENT_TEMPERATURE',            1,...
                     'REGISTERED_INSTRUCTION',         1,...
                     'MOVING',                         1,...
                     'HARDWARE_ERROR_STATUS',          1,...
                     'PUNCH',                          2);
   end
   
   methods (Static)
       
      function CRC = CRC_update(CRC,Packet_Data,L_Packet_Data)
%Calculates the CRC
% By: Federico Parietti
%Inputs:
% - CRC is 0 (format: double), the initial value of the CRC
% - Packet_Data is an array containing the values (format: double) of the
%   Instruction and Parameter fields.
% - L_Packet_Data is the length of Packet_Data (format: double)
%Output:
% - CRC is the value of the CRC for this data packet


%Create CRC table and convert from hex to dec format
CRC_table = [  hex2dec('0000'), hex2dec('8005'), hex2dec('800F'), hex2dec('000A'), hex2dec('801B'), hex2dec('001E'), hex2dec('0014'), hex2dec('8011')

               hex2dec('8033'), hex2dec('0036'), hex2dec('003C'), hex2dec('8039'), hex2dec('0028'), hex2dec('802D'), hex2dec('8027'), hex2dec('0022')

               hex2dec('8063'), hex2dec('0066'), hex2dec('006C'), hex2dec('8069'), hex2dec('0078'), hex2dec('807D'), hex2dec('8077'), hex2dec('0072')

               hex2dec('0050'), hex2dec('8055'), hex2dec('805F'), hex2dec('005A'), hex2dec('804B'), hex2dec('004E'), hex2dec('0044'), hex2dec('8041')

               hex2dec('80C3'), hex2dec('00C6'), hex2dec('00CC'), hex2dec('80C9'), hex2dec('00D8'), hex2dec('80DD'), hex2dec('80D7'), hex2dec('00D2')

               hex2dec('00F0'), hex2dec('80F5'), hex2dec('80FF'), hex2dec('00FA'), hex2dec('80EB'), hex2dec('00EE'), hex2dec('00E4'), hex2dec('80E1')

               hex2dec('00A0'), hex2dec('80A5'), hex2dec('80AF'), hex2dec('00AA'), hex2dec('80BB'), hex2dec('00BE'), hex2dec('00B4'), hex2dec('80B1')

               hex2dec('8093'), hex2dec('0096'), hex2dec('009C'), hex2dec('8099'), hex2dec('0088'), hex2dec('808D'), hex2dec('8087'), hex2dec('0082')

               hex2dec('8183'), hex2dec('0186'), hex2dec('018C'), hex2dec('8189'), hex2dec('0198'), hex2dec('819D'), hex2dec('8197'), hex2dec('0192')

               hex2dec('01B0'), hex2dec('81B5'), hex2dec('81BF'), hex2dec('01BA'), hex2dec('81AB'), hex2dec('01AE'), hex2dec('01A4'), hex2dec('81A1')

               hex2dec('01E0'), hex2dec('81E5'), hex2dec('81EF'), hex2dec('01EA'), hex2dec('81FB'), hex2dec('01FE'), hex2dec('01F4'), hex2dec('81F1')

               hex2dec('81D3'), hex2dec('01D6'), hex2dec('01DC'), hex2dec('81D9'), hex2dec('01C8'), hex2dec('81CD'), hex2dec('81C7'), hex2dec('01C2')

               hex2dec('0140'), hex2dec('8145'), hex2dec('814F'), hex2dec('014A'), hex2dec('815B'), hex2dec('015E'), hex2dec('0154'), hex2dec('8151')

               hex2dec('8173'), hex2dec('0176'), hex2dec('017C'), hex2dec('8179'), hex2dec('0168'), hex2dec('816D'), hex2dec('8167'), hex2dec('0162')

               hex2dec('8123'), hex2dec('0126'), hex2dec('012C'), hex2dec('8129'), hex2dec('0138'), hex2dec('813D'), hex2dec('8137'), hex2dec('0132')

               hex2dec('0110'), hex2dec('8115'), hex2dec('811F'), hex2dec('011A'), hex2dec('810B'), hex2dec('010E'), hex2dec('0104'), hex2dec('8101')

               hex2dec('8303'), hex2dec('0306'), hex2dec('030C'), hex2dec('8309'), hex2dec('0318'), hex2dec('831D'), hex2dec('8317'), hex2dec('0312')

               hex2dec('0330'), hex2dec('8335'), hex2dec('833F'), hex2dec('033A'), hex2dec('832B'), hex2dec('032E'), hex2dec('0324'), hex2dec('8321')

               hex2dec('0360'), hex2dec('8365'), hex2dec('836F'), hex2dec('036A'), hex2dec('837B'), hex2dec('037E'), hex2dec('0374'), hex2dec('8371')

               hex2dec('8353'), hex2dec('0356'), hex2dec('035C'), hex2dec('8359'), hex2dec('0348'), hex2dec('834D'), hex2dec('8347'), hex2dec('0342')

               hex2dec('03C0'), hex2dec('83C5'), hex2dec('83CF'), hex2dec('03CA'), hex2dec('83DB'), hex2dec('03DE'), hex2dec('03D4'), hex2dec('83D1')

               hex2dec('83F3'), hex2dec('03F6'), hex2dec('03FC'), hex2dec('83F9'), hex2dec('03E8'), hex2dec('83ED'), hex2dec('83E7'), hex2dec('03E2')

               hex2dec('83A3'), hex2dec('03A6'), hex2dec('03AC'), hex2dec('83A9'), hex2dec('03B8'), hex2dec('83BD'), hex2dec('83B7'), hex2dec('03B2')

               hex2dec('0390'), hex2dec('8395'), hex2dec('839F'), hex2dec('039A'), hex2dec('838B'), hex2dec('038E'), hex2dec('0384'), hex2dec('8381')

               hex2dec('0280'), hex2dec('8285'), hex2dec('828F'), hex2dec('028A'), hex2dec('829B'), hex2dec('029E'), hex2dec('0294'), hex2dec('8291')

               hex2dec('82B3'), hex2dec('02B6'), hex2dec('02BC'), hex2dec('82B9'), hex2dec('02A8'), hex2dec('82AD'), hex2dec('82A7'), hex2dec('02A2')

               hex2dec('82E3'), hex2dec('02E6'), hex2dec('02EC'), hex2dec('82E9'), hex2dec('02F8'), hex2dec('82FD'), hex2dec('82F7'), hex2dec('02F2')

               hex2dec('02D0'), hex2dec('82D5'), hex2dec('82DF'), hex2dec('02DA'), hex2dec('82CB'), hex2dec('02CE'), hex2dec('02C4'), hex2dec('82C1')

               hex2dec('8243'), hex2dec('0246'), hex2dec('024C'), hex2dec('8249'), hex2dec('0258'), hex2dec('825D'), hex2dec('8257'), hex2dec('0252')

               hex2dec('0270'), hex2dec('8275'), hex2dec('827F'), hex2dec('027A'), hex2dec('826B'), hex2dec('026E'), hex2dec('0264'), hex2dec('8261')

               hex2dec('0220'), hex2dec('8225'), hex2dec('822F'), hex2dec('022A'), hex2dec('823B'), hex2dec('023E'), hex2dec('0234'), hex2dec('8231')

               hex2dec('8213'), hex2dec('0216'), hex2dec('021C'), hex2dec('8219'), hex2dec('0208'), hex2dec('820D'), hex2dec('8207'), hex2dec('0202') ];


%Calculate the value of the CRC
CRC = uint16(CRC);
for j_count=1:1:L_Packet_Data
    
    %First, shift CRC to the right by 8 bits
    step1 = bitshift(CRC,-8);
%     step1 = bitsrl(CRC,8);
    %Then, do bitwise XOR with the current value in Packet_Data
    step2 = bitxor( step1, uint16(Packet_Data(j_count)) );
    %Finally, do bitwise AND with FF (hex)
    index = bitand( step2, uint16(hex2dec('FF')) );
    
    %Find row and column of matrix element
    index = double(index);
    table_width = size(CRC_table,2);
    row         = floor(index/table_width);
    column      = floor(mod(index,table_width));
    
    %IMPORTANT: Matlab indexes start from 1
    row    = row+1;
    column = column+1;
    
    %Shift CRC to the left by 8 bits
    CRC = bitshift(CRC,8);
%     CRC = bitsll(CRC,8);
    %Do bitwise XOR between the shifter CRC and the table value indicated
    %by index
    CRC = bitxor( CRC, uint16(CRC_table(row,column)) );
    
end

%Return the output as a double number
CRC = double(CRC);

      end
      
      function [high_byte, low_byte] = high_low_bytes(number)
%Calculates the high and low bytes of the input number

%Negative numbers are represented with the Two's Complement
if number<0
    number = 2^16+number;
end

%Find the 2 bytes
high_byte = floor(number/256);        %quotient
low_byte  = floor( mod(number,256) ); %remainder(integer)

      end
      
      function decimal = highlow_to_decimal(high_byte, low_byte)
%Converts high byte and low byte into the corresponding decimal value

%Convert to decimal
decimal = high_byte*256+low_byte;

%Identify and compute negative numbers (Two's Complement)
if decimal>(2^15-1)
    decimal = decimal-2^16;
end

      end
      
      function [high_high_byte, high_byte, low_byte, low_low_byte] = high_high_low_low_bytes(number)
%Calculates the 4 bytes representation of the input number
%It also handles negative inputs (by using the two's complement)

%Negative numbers are represented with the Two's Complement
if number<0
    number = 2^32+number;
end

%Find the 4 bytes
high_high_byte = floor( number/(2^24) );
remainder1     = floor( mod( number,(2^24) ) );
high_byte      = floor( remainder1/(2^16) );
remainder2     = floor( mod( remainder1,(2^16) ) );
low_byte       = floor( remainder2/(2^8) );
low_low_byte   = floor( mod( remainder2,(2^8) ) );

      end
      
      function decimal = highhighlowlow_to_decimal(high_high_byte, high_byte, low_byte, low_low_byte)
%Converts high high byte, high byte, low byte and low low byte into the corresponding decimal value

%Convert to decimal
decimal = high_high_byte*(2^24)+high_byte*(2^16)+low_byte*(2^8)+low_low_byte;

%Identify and compute negative numbers (Two's Complement)
if decimal>(2^31-1)
    decimal = decimal-2^32;
end

      end
   end
   
   methods
       
      function xl = XL_320()
      %   OBJ = XL_320() constructs a XL_320 class object.
      end
      
      function [Instruction_Packet, Status_Packet, read_value] = read(xl,servo_ID,address,bytes,s)
%Sends an Instruction Packet to read the desired value at the specified
%address on the Dynamixel XL_320
% By: Federico Parietti
%Inputs:
%-> ID:      the ID of the Dynamixel XL_320 where we want to read
%-> address: the address of the value that we want to read on the control
%            table of the Dynamixel XL_320
%-> bytes:   the number of bytes in which the value is represented
%-> s:       the serial port object
%Outputs:
%-> Instruction_Packet: the Instruction Packet that has been sent to the
%                       Dynamixel XL_320
%-> Status_Packet:      the Status Packet that has been received from the
%                       Dynamixel XL_320
%-> read_value:         the read value of the desired parameter


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create the Instruction Packet

%Headers
Header1 = hex2dec('FF');
Header2 = hex2dec('FF');
Header3 = hex2dec('FD');

%Reserved byte
Reserved = hex2dec('00');

%ID of the Dynamixel XL_320
ID = servo_ID;

%Packet Length
num_param = 2+2; %address(2 bytes) + number of bytes to read (2 bytes)
L_p = num_param + 3; %number of parameters + 3 (Instruction and CRC)
[LEN_H, LEN_L] = xl.high_low_bytes(L_p);

%Instruction
Instruction = hex2dec('02'); %the Read instruction is 02

%Parameters
%Address
[address_H, address_L] = xl.high_low_bytes(address);
%Data length
[data_length_H, data_length_L] = xl.high_low_bytes(bytes);

%16 bit CRC
%Calculate the CRC value
Instruction_Packet = [ Header1
                       Header2
                       Header3
                       Reserved
                       ID
                       LEN_L
                       LEN_H
                       Instruction
                       address_L
                       address_H
                       data_length_L
                       data_length_H ];
L_Instruction_Packet = length(Instruction_Packet);
CRC = 0;
CRC = xl.CRC_update(CRC,Instruction_Packet,L_Instruction_Packet);
%Find Low and High bytes of the CRC
[CRC_H, CRC_L] = xl.high_low_bytes(CRC);

%Complete the Instruction Packet with the CRC
Instruction_Packet = [ Instruction_Packet
                       CRC_L
                       CRC_H              ];
                   
%Flush the buffer (if it's not already empty)
if s.BytesAvailable
    fread(s, s.BytesAvailable);
end        
                   
%Send the Instruction Packet

%Binary write
fwrite(s, Instruction_Packet);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Read the Status Packet

%Length of Status Packet after the Read Instruction Packet
L_Status_Packet = 3+1+1+2+1+1+bytes+2;
%(header+reserved+ID+packet length+instruction+error+parameter+CRC)

%Binary read
Status_Packet = fread(s, L_Status_Packet); %much faster if number of bytes is specified
%Note: if the 9th byte is zero, there are no errors

% %Order the received Status Packet (header at the beginning)
% flag_order = 0;
% if (Status_Packet(1)==255) && (Status_Packet(2)==255) && (Status_Packet(3)==253)
%     flag_order = 1;
% end
% while flag_order==0
%     %Shift the vector down by one (circularly)
%     Status_Packet = circshift(Status_Packet,[1 0]);
%     if (Status_Packet(1)==255) && (Status_Packet(2)==255) && (Status_Packet(3)==253)
%         flag_order = 1;
%     end
% end

%Isolate the relevant bytes (they can be 1, 2 or 4) and convert them to the
%corresponding decimal value
if bytes == 1
    
    read_value = Status_Packet(10); %it's already decimal
    
elseif bytes == 2
    
    read_value_L = Status_Packet(10);
    read_value_H = Status_Packet(11);
    read_value = xl.highlow_to_decimal(read_value_H, read_value_L);
    
elseif bytes == 4
    
    read_value_LL = Status_Packet(10);
    read_value_L  = Status_Packet(11);
    read_value_H  = Status_Packet(12);
    read_value_HH = Status_Packet(13);
    read_value = xl.highhighlowlow_to_decimal(read_value_HH, read_value_H, read_value_L, read_value_LL);

else
    read_value = NaN;
    disp('Incorrect bytes number')
end
      end
      
      function Instruction_Packet = write(xl,servo_ID,address,value,bytes,s)
%Writes the desired value at the specified address on the Dynamixel XL_320
% By: Federico Parietti
%Inputs:
%-> ID:      the ID of the Dynamixel XL_320 where we want to write
%-> address: the address on the control table of the Dynamixel XL_320
%-> value:   the value to be written at the above specified address
%-> bytes:   the number of bytes in which the value is represented
%-> s:       the serial port object
%Output:
%-> Instruction_Packet: the Instruction packet that has been sent to the
%                       Dynamixel XL_320


%Create the Instruction Packet

%Headers
Header1 = hex2dec('FF');
Header2 = hex2dec('FF');
Header3 = hex2dec('FD');

%Reserved byte
Reserved = hex2dec('00');

%ID of the Dynamixel XL_320
ID = servo_ID;

%Packet Length
num_param = 2+bytes; %address(2 bytes) + number of value bytes
L_p = num_param + 3; %number of parameters + 3 (Instruction and CRC)
[LEN_H, LEN_L] = xl.high_low_bytes(L_p);

%Instruction
Instruction = hex2dec('03'); %the Write instruction is 03

%Parameters
%Address
[address_H, address_L] = xl.high_low_bytes(address);
%Value(s) to write
if bytes==1
    values_W = value;
elseif bytes==2
    [value_H, value_L] = xl.high_low_bytes(value);
    values_W = [ value_L
                value_H ];
else %in this case the value is expressed in 4 bytes
    [value_HH, value_H, value_L, value_LL] = xl.high_high_low_low_bytes(value);
    values_W = [ value_LL
                 value_L
                 value_H
                 value_HH ];
end

%16 bit CRC
%Calculate the CRC value
Instruction_Packet = [ Header1
                       Header2
                       Header3
                       Reserved
                       ID
                       LEN_L
                       LEN_H
                       Instruction
                       address_L
                       address_H
                       values_W    ];
L_Instruction_Packet = length(Instruction_Packet);
CRC = 0;
CRC = xl.CRC_update(CRC,Instruction_Packet,L_Instruction_Packet);
%Find Low and High bytes of the CRC
[CRC_H, CRC_L] = xl.high_low_bytes(CRC);

%Complete the Instruction Packet with the CRC
Instruction_Packet = [ Instruction_Packet
                       CRC_L
                       CRC_H              ];

                   
%Send the Instruction Packet

%Binary write
fwrite(s, Instruction_Packet);


%Read the Status Packet

% %Length of Status Packet after a Write Instruction Packet
% L_Status_Packet = 11;
% 
% %Binary read
% Status_Packet = fread(s, L_Status_Packet) %much faster if number of bytes is specified
% %Note: if the 9th byte is zero, there are no errors

      end
      
      function read_value_vect = sync_read(xl,servo_IDs,address,bytes,s)
%Reads the desired values at the specified address on the Dynamixel XL_320
%servos, simultaneously (works with any number of servos)
% By: Federico Parietti
%Inputs:
%-> servo_IDs: the IDs of the Dynamixel XL_320 servos where we want to read
%              (row vector)
%-> address:   the address on the control table of the Dynamixel XL_320
%-> bytes:     the number of bytes in which the value is represented
%-> s:         the serial port object
%Output:
%-> read_value_vect: the read values of the desired parameter


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create the Instruction Packet

%Headers
Header1 = hex2dec('FF');
Header2 = hex2dec('FF');
Header3 = hex2dec('FD');

%Reserved byte
Reserved = hex2dec('00');

%ID of the Dynamixel XL_320
ID = hex2dec('FE'); %ID used for broadcast transmission

%Packet Length
num_servos = length(servo_IDs);
L_p = 1+2+2+num_servos+2;
%instruction(1)+common address(2)+common length(2)+servo ID (1 for each
%servo)+CRC(2)

%number of parameters + 3 (Instruction and CRC)
[LEN_H, LEN_L] = xl.high_low_bytes(L_p);

%Instruction
Instruction = hex2dec('82'); %the Sync Read instruction is 82

%Parameters
%Address
[common_address_H, common_address_L] = xl.high_low_bytes(address);
%Common read data length
[common_length_H, common_length_L] = xl.high_low_bytes(bytes);
%ID of first servo (from IDs vector)
%ID of second servo (from IDs vector)

%16 bit CRC
%Calculate the CRC value
Instruction_Packet = [ Header1
                       Header2
                       Header3
                       Reserved
                       ID
                       LEN_L
                       LEN_H
                       Instruction
                       common_address_L
                       common_address_H
                       common_length_L
                       common_length_H
                       servo_IDs'       ];       
L_Instruction_Packet = length(Instruction_Packet);
CRC = 0;
CRC = xl.CRC_update(CRC,Instruction_Packet,L_Instruction_Packet);
%Find Low and High bytes of the CRC
[CRC_H, CRC_L] = xl.high_low_bytes(CRC);

%Complete the Instruction Packet with the CRC
Instruction_Packet = [ Instruction_Packet
                       CRC_L
                       CRC_H              ];

%Flush the buffer (if it's not already empty)
if s.BytesAvailable
    fread(s, s.BytesAvailable);
end
                   
%Send the Instruction Packet

%Binary write
fwrite(s, Instruction_Packet);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Read the Status Packet

%Length of Status Packet after the Read Instruction Packet
%(note: there are as many Status Packets as servos called in the Read
%Instruction Packet)
L_Status_Packet = 3+1+1+2+1+1+bytes+2;
%header(3)+reserved(1)+ID(1)+packet length(2)+instruction(1)+error(1)+
%+parameter(bytes)+CRC(2)

%Read all of the Status Packets, extracting the desired values from all of
%them (same order as the servos in servo_IDs)
read_value_vect = zeros(1,num_servos);
for count=1:1:num_servos
    %Binary read
    Status_Packet = fread(s, L_Status_Packet);
    %Note: if the 9th byte is zero, there are no errors
    
    %Isolate the relevant bytes (they can be 1, 2 or 4) and convert them to the
    %corresponding decimal value
    if bytes == 1

        read_value_vect(count) = Status_Packet(10); %it's already decimal

    elseif bytes == 2

        read_value_L = Status_Packet(10);
        read_value_H = Status_Packet(11);
        read_value_vect(count) = xl.highlow_to_decimal(read_value_H, read_value_L);

    elseif bytes == 4

        read_value_LL = Status_Packet(10);
        read_value_L  = Status_Packet(11);
        read_value_H  = Status_Packet(12);
        read_value_HH = Status_Packet(13);
        read_value_vect(count) = xl.highhighlowlow_to_decimal(read_value_HH, read_value_H, read_value_L, read_value_LL);

    else
        read_value_vect(count) = NaN;
        disp('Incorrect bytes number')
    end
end

      end
      
      function Instruction_Packet = sync_write(xl,servo_IDs,address,values,bytes,s)
%Writes the desired values at the specified address on the Dynamixel XL_320
%servos, simultaneously (works with 2 servos)
% By: Federico Parietti
%Inputs:
%-> IDs:     the IDs of the Dynamixel XL_320 servos where we want to write
%-> address: the address on the control table of the Dynamixel XL_320
%-> values:  the values to be written at the above specified address (in
%            the same order as the servos IDs)
%-> bytes:   the number of bytes in which the value is represented
%-> s:       the serial port object
%Output:
%-> Instruction_Packet: the Instruction packet that has been sent to the
%                       Dynamixel XL_320


%Create the Instruction Packet

%Headers
Header1 = hex2dec('FF');
Header2 = hex2dec('FF');
Header3 = hex2dec('FD');

%Reserved byte
Reserved = hex2dec('00');

%ID of the Dynamixel XL_320
ID = hex2dec('FE'); %ID used for broadcast transmission

%Packet Length
L_p = 1+2+2+(1+bytes)*length(servo_IDs)+2;
%instruction(1)+common address(2)+common length(2)+servo ID and value
%(1+bytes for each servo)+CRC(2)

%number of parameters + 3 (Instruction and CRC)
[LEN_H, LEN_L] = xl.high_low_bytes(L_p);

%Instruction
Instruction = hex2dec('83'); %the Sync Write instruction is 83

%Parameters
%Address
[common_address_H, common_address_L] = xl.high_low_bytes(address);
%Common write data length
[common_length_H, common_length_L] = xl.high_low_bytes(bytes);
%ID of first servo (from IDs vector)
%Value to write on first servo
if bytes==1
    values_W1 = values(1);
elseif bytes==2
    [value_H, value_L] = xl.high_low_bytes(values(1));
    values_W1 = [ value_L
                  value_H ];
else %in this case the value is expressed in 4 bytes
    [value_HH, value_H, value_L, value_LL] = xl.high_high_low_low_bytes(values(1));
    values_W1 = [ value_LL
                  value_L
                  value_H
                  value_HH ];
end
%ID of second servo (from IDs vector)
%Value to write on second servo
if bytes==1
    values_W2 = values(2);
elseif bytes==2
    [value_H, value_L] = xl.high_low_bytes(values(2));
    values_W2 = [ value_L
                  value_H ];
else %in this case the value is expressed in 4 bytes
    [value_HH, value_H, value_L, value_LL] = xl.high_high_low_low_bytes(values(2));
    values_W2 = [ value_LL
                  value_L
                  value_H
                  value_HH ];
end

%16 bit CRC
%Calculate the CRC value
Instruction_Packet = [ Header1
                       Header2
                       Header3
                       Reserved
                       ID
                       LEN_L
                       LEN_H
                       Instruction
                       common_address_L
                       common_address_H
                       common_length_L
                       common_length_H
                       servo_IDs(1)
                       values_W1
                       servo_IDs(2)
                       values_W2        ];       
L_Instruction_Packet = length(Instruction_Packet);
CRC = 0;
CRC = xl.CRC_update(CRC,Instruction_Packet,L_Instruction_Packet);
%Find Low and High bytes of the CRC
[CRC_H, CRC_L] = xl.high_low_bytes(CRC);

%Complete the Instruction Packet with the CRC
Instruction_Packet = [ Instruction_Packet
                       CRC_L
                       CRC_H              ];
                   
%Send the Instruction Packet

%Binary write
fwrite(s, Instruction_Packet);

      end
      
      function Instruction_Packet = sync_write_four(xl,servo_IDs,address,values,bytes,s)
%Writes the desired values at the specified address on the Dynamixel XL_320
%servos, simultaneously (works with 4 servos)
% By: Federico Parietti
%Inputs:
%-> IDs:     the IDs of the Dynamixel XL_320 servos where we want to write
%-> address: the address on the control table of the Dynamixel XL_320
%-> values:  the values to be written at the above specified address (in
%            the same order as the servos IDs)
%-> bytes:   the number of bytes in which the value is represented
%-> s:       the serial port object
%Output:
%-> Instruction_Packet: the Instruction packet that has been sent to the
%                       Dynamixel XL_320


%Create the Instruction Packet

%Headers
Header1 = hex2dec('FF');
Header2 = hex2dec('FF');
Header3 = hex2dec('FD');

%Reserved byte
Reserved = hex2dec('00');

%ID of the Dynamixel XL_320
ID = hex2dec('FE'); %ID used for broadcast transmission

%Packet Length
L_p = 1+2+2+(1+bytes)*length(servo_IDs)+2;
%instruction(1)+common address(2)+common length(2)+servo ID and value
%(1+bytes for each servo)+CRC(2)

%number of parameters + 3 (Instruction and CRC)
[LEN_H, LEN_L] = xl.high_low_bytes(L_p);

%Instruction
Instruction = hex2dec('83'); %the Sync Write instruction is 83

%Parameters
%Address
[common_address_H, common_address_L] = xl.high_low_bytes(address);
%Common write data length
[common_length_H, common_length_L] = xl.high_low_bytes(bytes);
%ID of first servo (from IDs vector)
%Value to write on first servo
if bytes==1
    values_W1 = values(1);
elseif bytes==2
    [value_H, value_L] = xl.high_low_bytes(values(1));
    values_W1 = [ value_L
                  value_H ];
else %in this case the value is expressed in 4 bytes
    [value_HH, value_H, value_L, value_LL] = xl.high_high_low_low_bytes(values(1));
    values_W1 = [ value_LL
                  value_L
                  value_H
                  value_HH ];
end
%ID of second servo (from IDs vector)
%Value to write on second servo
if bytes==1
    values_W2 = values(2);
elseif bytes==2
    [value_H, value_L] = xl.high_low_bytes(values(2));
    values_W2 = [ value_L
                  value_H ];
else %in this case the value is expressed in 4 bytes
    [value_HH, value_H, value_L, value_LL] = xl.high_high_low_low_bytes(values(2));
    values_W2 = [ value_LL
                  value_L
                  value_H
                  value_HH ];
end
%ID of third servo (from IDs vector)
%Value to write on third servo
if bytes==1
    values_W3 = values(3);
elseif bytes==2
    [value_H, value_L] = xl.high_low_bytes(values(3));
    values_W3 = [ value_L
                  value_H ];
else %in this case the value is expressed in 4 bytes
    [value_HH, value_H, value_L, value_LL] = xl.high_high_low_low_bytes(values(3));
    values_W3 = [ value_LL
                  value_L
                  value_H
                  value_HH ];
end
%ID of fourth servo (from IDs vector)
%Value to write on fourth servo
if bytes==1
    values_W4 = values(4);
elseif bytes==2
    [value_H, value_L] = xl.high_low_bytes(values(4));
    values_W4 = [ value_L
                  value_H ];
else %in this case the value is expressed in 4 bytes
    [value_HH, value_H, value_L, value_LL] = xl.high_high_low_low_bytes(values(4));
    values_W4 = [ value_LL
                  value_L
                  value_H
                  value_HH ];
end

%16 bit CRC
%Calculate the CRC value
Instruction_Packet = [ Header1
                       Header2
                       Header3
                       Reserved
                       ID
                       LEN_L
                       LEN_H
                       Instruction
                       common_address_L
                       common_address_H
                       common_length_L
                       common_length_H
                       servo_IDs(1)
                       values_W1
                       servo_IDs(2)
                       values_W2
                       servo_IDs(3)
                       values_W3
                       servo_IDs(4)
                       values_W4        ];       
L_Instruction_Packet = length(Instruction_Packet);
CRC = 0;
CRC = xl.CRC_update(CRC,Instruction_Packet,L_Instruction_Packet);
%Find Low and High bytes of the CRC
[CRC_H, CRC_L] = xl.high_low_bytes(CRC);

%Complete the Instruction Packet with the CRC
Instruction_Packet = [ Instruction_Packet
                       CRC_L
                       CRC_H              ];
                   
%Send the Instruction Packet

%Binary write
fwrite(s, Instruction_Packet);

      end
   end
   
end