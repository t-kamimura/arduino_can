%
clear
close all

% arduinoObj = arduino('COM4', 'Uno', 'Libraries', 'CAN');
% canChannel(arduinoObj, "MCP2515", "D10", "D2", "OscillatorFrequency", 16e6, "BusSpeed", 500e3);

% read(ch)

pos = 0x00;   % 16 bit
vel = 0x00;   % 12 bit
kp = 0x00;    % 12 bit
kd = 0x00;    % 12 bit
ff = 0x00;    % 12 bit
    
  
can_msg(1) = bitshift(pos, 8);
can_msg(2) = pos & 0x00FF;
can_msg(3) = (bitshift(vel, 4)) & 0xFF;
can_msg(4) = (bitshift(bitand(vel, 0x000F), -4)) + bitand(bitshift(kp,8), 0xFF);
can_msg(5) = kp & 0xFF;
can_msg(6) = bitshift(kd,4);
can_msg(7) = (bitshift(kd & 0x000F, -4)) + (bitshift(ff,8));
can_msg(8) = ff & 0xff;
