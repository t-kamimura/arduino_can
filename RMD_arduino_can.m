%
clear
close all

arduinoObj = arduino('COM4', 'Uno', 'Libraries', 'CAN');
ch = canChannel(arduinoObj,"Sparkfun CAN-Bus Shield", "BusSpeed", 1000e3);

% read(ch)

% read PID parameter command
cmd(1) = 0x30;
cmd(2) = 0x00;
cmd(3) = 0x00;
cmd(4) = 0x00;
cmd(5) = 0x00;
cmd(6) = 0x00;
cmd(7) = 0x00;
cmd(8) = 0x00;

figure
hold on
n = 100;
xlim([0,n])
% ylim([])
for i_n = 1:n
    write(ch, 1, false, cmd)
    buf = read(ch,8)
%     motorData = buf.Data{1,1};
%     pos(1) = motorData(2);
%     pos(2) = motorData(3);
%     pos(3) = motorData(4);
%     pos(4) = motorData(5);
%     pos(5) = motorData(6);
%     pos(6) = motorData(7);
%     pos(7) = motorData(8);
%     pos_cur = uint64(pos(7));
%     for i = 1:6
%         pos_cur = pos_cur + uint64(bitshift(pos(i),8*(7-i)));
%     end
%     disp(num2str(pos));
%     plot(i_n,pos,'ob')
    pause(0.05)
end

clear arduinoObj
clear ch