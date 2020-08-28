% ボード線図をプロットするプログラム

clear
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0, 'defaultAxesFontSize', 16);
set(0, 'defaultAxesFontName', 'times');
set(0, 'defaultTextFontSize', 16);
set(0, 'defaultTextFontName', 'times');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath(pwd, 'kp_100ki_100');
file_list = ["1.mat", "2.mat", "3.mat", "4.mat", "5.mat"];
freqset = [0.2, 0.4, 0.8, 1.6, 3.2, 6.4, 12.8];
list = [0, 0, 0, 0, 0, 0, 0];

for i_n = 1:length(file_list)
    load(file_list(i_n));
    
    time = Dataset(:, 1);
    tgt_pos = Dataset(:, 2);
    cmd_byte = Dataset(:, 3);
    temperature = Dataset(:, 4);
    cur_L = Dataset(:, 5);
    cur_H = Dataset(:, 6);
    vel_L = Dataset(:, 7);
    vel_H = Dataset(:, 8);
    pos_L = Dataset(:, 9);
    pos_H = Dataset(:, 10);

    pos_cmd = Dataset(:, 11);
    pos_msg(:, 1) = Dataset(:, 12);
    pos_msg(:, 2) = Dataset(:, 13);
    pos_msg(:, 3) = Dataset(:, 14);
    pos_msg(:, 4) = Dataset(:, 15);
    pos_msg(:, 5) = Dataset(:, 16);
    pos_msg(:, 6) = Dataset(:, 17);
    pos_msg(:, 7) = Dataset(:, 18);

    cur = (bitshift(cur_H, 8, 'int16') + cur_L);
    vel = bitshift(vel_H, 8, 'int16') + vel_L;
    pos = bitshift(pos_msg(:, 7), 48, 'int64') + bitshift(pos_msg(:, 6), 40, 'int64') + bitshift(pos_msg(:, 5), 32, 'int64') + bitshift(pos_msg(:, 4), 24, 'int64') + bitshift(pos_msg(:, 3), 16, 'int64') + bitshift(pos_msg(:, 2), 8, 'int64') + pos_msg(:, 1);

    for l = 1:length(pos)
        if pos(l)  > 36028797018963967
            pos(l) = pos(l) - 72057594037927935;
        end
    end
    
    data(i_n).freq = freqset(i_n);

    for i_t = 30:length(time)
        data(i_n).tout(i_t,1) = time(i_t);
        data(i_n).inputout(i_t,1) = tgt_pos(i_t);
        data(i_n).outputout(i_t,1) = pos(i_t);
    end
    
    data(i_n).inputAmp = max(data(i_n).inputout);
    data(i_n).outputAmp = max(data(i_n).outputout);
    list(i_n) = data(i_n).outputAmp/data(i_n).inputAmp;

    semilogx(data(i_n).freq, list(i_n), 'bo')
    hold on
end


plot(freqset, list, 'b-')
title(["RMD-X8 PRO"; "A=17.5, kp=100, ki=100, kd=50"])
xlabel("frequency [Hz]")
ylabel("magnitude [dB]")
figSave(true, "RMD-X8_PRO")

% save
function figSave(tf, name)
    try
        if tf == true
            figname = [name];
            % saveas(gcf,figname,'fig')
            saveas(gcf,figname,'png')
            % saveas(gcf,figname,'pdf')
            disp('save finish!')
        end
    catch
        disp('some error(s) occurred in saving process')
    end 
end