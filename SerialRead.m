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
        Dataset = src.UserData.Data;
        countnum = src.UserData.Count;
        plot(src.UserData.Data(:, 1), src.UserData.Data(:, 2))
        hold on
%         figure
        try
            plot(src.UserData.Data(:, 1), src.UserData.Data(:, 5))
        catch
            disp('no receibed data avairable')
        end

        xlabel("time [ms]")
        ylabel("input/output")
        legend({'input', 'output'})
        filename = ['freqRespData.mat'];
        save(filename, "Dataset")
        filename = ['freqRespFig'];        
        saveas(gcf, filename, 'png')
    end

end
