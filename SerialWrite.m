% serialWrite

% initial settings
clear
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0, 'defaultAxesFontSize', 16);
set(0, 'defaultAxesFontName', 'Times new roman');
set(0, 'defaultTextFontSize', 16);
set(0, 'defaultTextFontName', 'Times new roman');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% シリアルポートの設定
s = serialport("COM4", 9600);       % シリアルオブジェクト
configureTerminator(s, "CR/LF");    % ターミネータ（シリアル通信データの区切り）
flush(s);                           % sに溜まっているデータを消す

s.UserData = struct("Data", [], "Count", 1);

% Arduinoが起動しsetupが終わるまで待つ
tic
data = 0;
while data<1
    data = read(s,1,"uint32");
end

% これ以降シリアルの受信はコールバック処理で行う
configureCallback(s, "terminator", @readSerialData)

% 位置指令の書き込み
tic

for i = 1:200
    t1 = toc;
    data = int16(sin(t1) * 90);
    disp(data)
    write(s,data,"int16");
    flush(s)
    while toc- t1 < 0.05
    end
end

% 終了処理
clear s

% コールバック関数
function readSerialData(src, Dataset, countnum)
    data = readline(src);
    src.UserData.Count = src.UserData.Count + 1;
    disp(data)

%     if src.UserData.Count < 6
%         disp(data)
%     else
%         src.UserData.Data(end + 1, :) = str2num(data);
%         disp(data)
%     end
%     if src.UserData.Count > 154
%         disp("off")
%         configureCallback(src, "off");
%         Dataset = src.UserData.Data;
%         countnum = src.UserData.Count;
%         plot(src.UserData.Data(:, 1), src.UserData.Data(:, 2))
%         hold on
% %         figure
%         try
%             plot(src.UserData.Data(:, 1), src.UserData.Data(:, 5))
%         catch
%             disp('no receibed data avairable')
%         end
% 
%         xlabel("time [ms]")
%         ylabel("input/output")
%         legend({'input', 'output'})
%         filename = ['freqRespData.mat'];
%         save(filename, "Dataset")
%         filename = ['freqRespFig'];        
%         saveas(gcf, filename, 'png')
%     end

end
