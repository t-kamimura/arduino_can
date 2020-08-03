%
clear
close all

arduinoObj = arduino('COM4', 'Uno', 'Libraries', 'CAN');
ch = canChannel(arduinoObj,"Sparkfun CAN-Bus Shield", "BusSpeed", 1000e3);

% read(ch)

% % read PID parameter command
% data(1) = 0x30;
% data(2) = 0x00;
% data(3) = 0x00;
% data(4) = 0x00;
% data(5) = 0x00;
% data(6) = 0x00;
% data(7) = 0x00;
% data(8) = 0x00;

% position reset command
disp('position RESET')
data(1) = 0xFF;
data(2) = 0xFF;
data(3) = 0xFF;
data(4) = 0xFF;
data(5) = 0xFF;
data(6) = 0xFF;
data(7) = 0xFF;
data(8) = 0xFE;
write(ch, 1, false, data)
pause(1)
buf = read(ch,8);
motorData = buf.Data{1,1};
pos_H = motorData(2);
pos_L = motorData(3);
pos = bitshift(uint16(pos_H),8) + uint16(pos_L);
logData(1,:) = [pos_H pos_L pos];
pause(1)


% % Motor ON command
% disp('ON')
% data(1) = 0xFF;
% data(2) = 0xFF;
% data(3) = 0xFF;
% data(4) = 0xFF;
% data(5) = 0xFF;
% data(6) = 0xFF;
% data(7) = 0xFF;
% data(8) = 0xFC;
% write(ch, 1, false, data)
% pause(1)
% buf = read(ch,8);
% motorData = buf.Data{1,1};
% pos_H = motorData(2);
% pos_L = motorData(3);
% pos = bitshift(uint16(pos_H),8) + uint16(pos_L);
% logData(2,:) = [pos_H pos_L pos];
% pause(1)


% Motor command

pos_tgt = 10; % 16 bit (max. 65535)
% bias = hex2dec('FF00'); % 0b 1111 1111 0000 0000
% pos_tgt = bitor(pos_tgt,bias);
bias = 32768;
pos_tgt = pos_tgt + bias;
vel_tgt = 0; % 12 bit (max. 4095)
kp_tgt = 1; % 12 bit (max. 4095)
kd_tgt = 0; % 12 bit (max. 4095)
ff_tgt = 0; % 12 bit (max. 4095)

cmd(1) = uint8(bitshift(pos_tgt, -8)); % pos上8bitのみ抜き出し
cmd(2) = uint8(bitand(pos_tgt, hex2dec('FF'))); % pos下8bitのみ抜き出し
cmd(3) = uint8(bitshift(vel_tgt, -8)); % vel上8ビットのみ抜き出し
cmd(4) = uint8(bitshift(bitand(vel_tgt, hex2dec('F')), 4) + bitand(bitshift(kp_tgt, -8), hex2dec('F'))); % velの下4桁と，kpの上4桁を合成
cmd(5) = uint8(bitand(kp_tgt, hex2dec('FF'))); % kpの下8bit
cmd(6) = uint8(bitshift(kd_tgt, -8)); % kdの上8bit
cmd(7) = uint8(bitshift(bitand(kd_tgt, hex2dec('F')), 4) + bitand(bitshift(ff_tgt, -8), hex2dec('F'))); % kdの下4桁と，kpの上4桁を合成
cmd(8) = uint8(bitand(ff_tgt, hex2dec('FF')));

% write(ch, 1, false, cmd)
% pause(1)
% buf = read(ch,8);
% data = buf.Data{1,1};
% pos_H = data(2);
% pos_L = data(3);
% pos = bitshift(uint16(pos_H),8) + uint16(pos_L);
% pause(1)

% Motor OFF command
disp('OFF')
data(1) = 0xFF;
data(2) = 0xFF;
data(3) = 0xFF;
data(4) = 0xFF;
data(5) = 0xFF;
data(6) = 0xFF;
data(7) = 0xFF;
data(8) = 0xFD;
write(ch, 1, false, data)
pause(1)
buf = read(ch,8);
motorData = buf.Data{1,1};
pos_H = motorData(2);
pos_L = motorData(3);
pos = bitshift(uint16(pos_H),8) + uint16(pos_L);
logData(3,:) = [pos_H pos_L pos];

figure
hold on
n = 100;
xlim([0,n])
% ylim([])
for i = 1:n
    data(1) = 0xFF;
    data(2) = 0xFF;
    data(3) = 0xFF;
    data(4) = 0xFF;
    data(5) = 0xFF;
    data(6) = 0xFF;
    data(7) = 0xFF;
    data(8) = 0xFD;
    write(ch, 1, false, data)
    buf = read(ch,8);
    motorData = buf.Data{1,1};
    pos_H = motorData(2);
    pos_L = motorData(3);
    upos = bitshift(uint16(pos_H),8) + uint16(pos_L);
    if upos > 32768
        pos = int16(upos - 32768);
    else
        pos = int16(upos) - 32767;
    end
    disp(num2str(pos));
    plot(i,pos,'ob')
    pause(0.01)
end

clear arduinoObj
clear ch