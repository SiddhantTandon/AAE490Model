clear
clc

% preliminary rotor moment of inertia ~1 Nm
% preliminary drag moment ~0.3 Nm

%{
Turnigy Aerodrive Motor Specs:
Max Current: 80 A
Max Power: 4032 W
Operating Voltage: 10-12S LiPoly
rpm/V: 192kv
%}


%% Load data from File

fileName = '2_19_637pm.csv'; % 627, 629, 633, 635, 637
sampleFreq = 1; %[samples/sec]

kv = 370; %Turnigy motor - 192kv
polesMotor = 14; %Turnigy motor - 14 turns?  counted 12 coils

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
time = [0 : length(throttlePosition) - 1]' / sampleFreq;

%% Derived Results
rotRate = rotRate / polesMotor;

omega = 2 * pi * rotRate / 60; %rotation rate [rad/s]

torque = zeros(length(powerDraw), 1);
for n = 1:length(powerDraw)
    if omega(n) ~= 0
        torque(n) = powerDraw(n) / omega(n); %torque provided [Nm]
    else
        torque(n) = 0;
    end
end

throttle = (throttlePosition - 1.1) / (1.9 - 1.1) * 100; %level above min throttle / total throttle range [%]

alpha = zeros(length(omega), 1);
alpha(1) = (omega(1) - 0) * sampleFreq;
for n = 2 : length(omega) - 1
    alpha(n) = (omega(n+1) - omega(n)) * sampleFreq; %change in omega / time per sample
end

Iz = torque ./ alpha;

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

figure(2)
plot(time(40:250), alpha(40:250))
hold on
plot(time(40:250), 10*torque(40:250))
hold off
xlabel('time [s]')
ylabel('Angular Acceleration [rad/s^2]')
title('Angular Acceleration vs. t')

figure(3)
plot(time, Iz)