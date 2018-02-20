clear
clc

%% Load data from File

fileName = '2_19_637pm.csv'; % 627, 629, 633, 635, 637
sampleFreq = 1; %[samples/sec]

%{
% Find KV from csv File
fileID = fopen(fileName);
line = '     ';
while line(3:4) ~= 'KV'
    line = fgetl(fileID);
end
kv = str2double(line(8 : end));
fclose(fileID);
%}
kv = 370;
polesMotor = 14;

% Skip initial info lines
fileID = fopen(fileName);
dataStartLine = 0;
line = '#';
while line(1) == '#'
    line = fgetl(fileID);
    dataStartLine = dataStartLine + 1;
end
fclose(fileID);

% Load Numerical Data
dataStartLine = dataStartLine + 2; %data starts 2 lines after last '#'
data = csvread(fileName, dataStartLine - 1, 1 - 1); %row & line start at 0

throttlePosition = data(:, 1); % reading from throttle [ms]
powerDraw = data(:, 2); %[W]
voltage = data(:, 3); %[V]
current = powerDraw ./ voltage; %[A] current reading doesn't work
rotRate = data(:, 7); %[rpm]
time = sampleFreq * [0 : length(throttlePosition) - 1]';

% Derived Data
rotRate = rotRate / polesMotor;

omega = 2 * pi * rotRate / 60; %rotation rate [rad/s]
torque = powerDraw ./ omega; %torque provided [Nm]

throttle = (throttlePosition - 1.1) / (1.9 - 1.1) * 100; %level above min throttle / total throttle range [%]


%% Plot Results
figure(1)

subplot(2, 3, 1)
plot(time, throttle)
xlabel('time [s]')
ylabel('Throttle Position [%]')
title('Throttle vs. t')

subplot(2, 3, 2)
plot(time, powerDraw)
xlabel('time [s]')
ylabel('Power Draw [W]')
title('Power Draw vs. t')

subplot(2, 3, 3)
plot(time, voltage)
xlabel('time [s]')
ylabel('Voltage [V]')
title('Voltage vs. t')

subplot(2, 3, 4)
plot(time, current)
xlabel('time [s]')
ylabel('Current [A]')
title('Current vs. t')

subplot(2, 3, 5)
plot(time, rotRate)
xlabel('time [s]')
ylabel('Rotation Rate [rpm]')
title('Rotation Rate vs. t')

subplot(2, 3, 6)
plot(time, torque)
xlabel('time [s]')
ylabel('Torque [Nm]')
title('Torque vs. t')
