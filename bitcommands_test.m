clear
close all

pos_tgt = 10; % 16 bit (max. 65535)
% bias = hex2dec('FF00'); % 0b 1111 1111 0000 0000
% pos_tgt = bitor(pos_tgt,bias);
bias = 32768;
pos_tgt = pos_tgt + bias;
vel_tgt = 0; % 12 bit (max. 4095)
kp_tgt = 1; % 12 bit (max. 4095)
kd_tgt = 0; % 12 bit (max. 4095)
ff_tgt = 0; % 12 bit (max. 4095)

% 指定した桁のビットのみを抜き出すためには
% (上8bitの場合) 8桁右ビットシフトする
% (上4bitの場合) 8桁右ビットシフトする
% (下8bitの場合) 0b 0000 0000 1111 1111 = 0x FF とビットアンドを取る
% (下4bitの場合) 0b 0000 0000 0000 1111 = 0x 0F とビットアンドを取る

cmd(1) = uint8(bitshift(pos_tgt, -8)); % pos上8bitのみ抜き出し
cmd(2) = uint8(bitand(pos_tgt, hex2dec('FF'))); % pos下8bitのみ抜き出し
cmd(3) = uint8(bitshift(vel_tgt, -8)); % vel上8ビットのみ抜き出し
cmd(4) = uint8(bitshift(bitand(vel_tgt, hex2dec('F')), 4) + bitand(bitshift(kp_tgt,-8),hex2dec('F'))); % velの下4桁と，kpの上4桁を合成
cmd(5) = uint8(bitand(kp_tgt, hex2dec('FF')));  % kpの下8bit
cmd(6) = uint8(bitshift(kd_tgt, -8));   % kdの上8bit
cmd(7) = uint8(bitshift(bitand(kd_tgt, hex2dec('F')), 4) + bitand(bitshift(ff_tgt, -8), hex2dec('F'))); % kdの下4桁と，kpの上4桁を合成
cmd(8) = uint8(bitand(ff_tgt, hex2dec('FF')));

% 復号
pos_cmd = bitshift(uint16(cmd(1)), 8) + uint16(cmd(2));
vel_cmd = bitshift(uint16(cmd(3)), 4) + uint16(bitshift(cmd(4),-4));
kp_cmd = bitshift(bitand(uint16(cmd(4)),hex2dec('F')),8) + uint16(cmd(5));
kd_cmd = bitshift(uint16(cmd(6)), 4) + uint16(bitshift(cmd(7), -4));
ff_cmd = bitshift(bitand(uint16(cmd(7)), hex2dec('F')), 8) + uint16(cmd(8));

pos_H = 255;
pos_L = 255;
upos = bitshift(uint16(pos_H), 8) + uint16(pos_L);

if upos > 32768
    pos = int16(upos - 32768);
else
    pos = int16(upos) - 32767;
end
