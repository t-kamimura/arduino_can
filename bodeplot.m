clear
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
set(0, 'defaultAxesFontSize', 12);
set(0, 'defaultAxesFontName', 'times');
set(0, 'defaultTextFontSize', 16);
set(0, 'defaultTextFontName', 'times');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% numset = [ 11 12 13 14 15 16 17 ...
%             21 22 23 24 25 26 27 28 ...
%             31 32 33 34 35 36 37 38];
% freqset = [0.25 0.5 1 2 4 8 16 ...
%             0.25 0.5 1 2 4 8 16 32 ...
%             0.25 0.5 1 2 4 8 16 32];
numset = [ 11 12 13 14 15 16 17];
freqset = [0.25 0.5 1 2 4 8 16];
figure        

for i_n = 1:length(numset)
    filename = ['freqRespData',num2str(numset(i_n)),'.mat'];
    load(filename)
    
    data(i_n).freq = freqset(i_n);
    for i_t = 30:length(Dataset(:,1))
        data(i_n).tout(i_t,1) = Dataset(i_t,1);
        data(i_n).inputout(i_t,1) = Dataset(i_t,2);
        data(i_n).outputout(i_t,1) = Dataset(i_t,5);
    end
    
    inputAmp = max(data(i_n).inputout);
    outputAmp = max(data(i_n).outputout);
    
    loglog(data(i_n).freq, outputAmp/inputAmp, 'o')
    hold on
end
