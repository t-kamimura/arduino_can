clear
close all

arduinoObj = arduino('COM4', 'Uno');

n = 5;

for i = 1:n
    writeDigitalPin(arduinoObj ,'D2',0);
    pause(0.1);
    writeDigitalPin(arduinoObj ,'D2',1);
    pause(0.1);
end
writeDigitalPin(arduinoObj ,'D2',0);
fprintf('finish\n')