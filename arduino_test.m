clear
close all

arduinoObj = arduino('COM4', 'Uno');

n = 5;

for i = 1:n
    writeDigitalPin(arduinoObj ,'D7',0);
    pause(0.5);
    writeDigitalPin(arduinoObj ,'D7',1);
    pause(0.5);
end
writeDigitalPin(arduinoObj ,'D7',0);
fprintf('finish\n')