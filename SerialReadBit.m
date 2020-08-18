% serialread

% initial settings
clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0, 'defaultAxesFontSize', 16);
set(0, 'defaultAxesFontName', 'Times new roman');
set(0, 'defaultTextFontSize', 16);
set(0, 'defaultTextFontName', 'Times new roman');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s = serialport("COM3", 115200);
configureTerminator(s, "CR/LF");
flush(s);
s.UserData = struct("Data", [], "Count", 1);
global Dataset countnum
configureCallback(s, "terminator", @readSerialData)

function readSerialData(src, Dataset, countnum)
    data = readline(src);
    src.UserData.Count = src.UserData.Count + 1;

    if src.UserData.Count < 6
        disp(data)
    else
        src.UserData.Data(end + 1, :) = str2num(data);
        disp(data)
    end
    if src.UserData.Count > 990
        disp("off")
        configureCallback(src, "off");
        
        % positionを読み取る
        Dataset = src.UserData.Data;
        pos_low_1 = src.UserData.Data(:, 2);
        pos_2 = src.UserData.Data(:, 3);
        pos_3 = src.UserData.Data(:, 4);
        pos_4 = src.UserData.Data(:, 5);
        pos_5 = src.UserData.Data(:, 6);
        pos_6 = src.UserData.Data(:, 7);
        pos_7 = src.UserData.Data(:, 8);
        pos = bitshift(uint8(pos_7), 48) + bitshift(uint8(pos_6), 40) + bitshift(uint8(pos_5), 32) + bitshift(uint8(pos_4), 24) + bitshift(uint8(pos_3), 16) + bitshift(uint8(pos_2), 8) + uint8(pos_low_1);
        countnum = src.UserData.Count;
        logData(1,:) = [pos_low_1, pos_2, pos_3, pos_4, pos_5, pos_6, pos_7, pos];
    end

end