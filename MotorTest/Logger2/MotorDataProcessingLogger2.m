clear
clc

%{
Samples taken every 0.1 seconds

File Format:
Time (minutes): Columns 1, 2, 4, 6, 8, ...
Throttle: 12
RPM: 18
Pack Voltage (volts): 26
Pack Current (amps): 28
Power (watts): 30
Prop RPM?: 62
Time (seconds): 114

Preliminary rotor moment of inertia ~1 Nm
Preliminary drag moment ~0.3 Nm

Turnigy Aerodrive Motor Specs:
Max Current: 80 A
Max Power: 4032 W
Operating Voltage: 10-12S LiPoly
rpm/V: 192kv
%}

polesMotor = 14;

%% Load Data from File
fileName = '4_4_test3.txt';
data = dlmread(fileName, '\t', 1, 0);

time = data(:, 114); %[s]
throttlePosition = data(:, 12); % reading from throttle [?]
powerDraw = data(:, 30); %[W]
voltage = data(:, 26); %[V]
current = data(:, 28); %[A] current reading doesn't work
rotRate = data(:, 18); %[rpm]

%% Derived Results
%{
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

%level above min throttle / total throttle range [%]
throttle = (throttlePosition - min(throttlePosition)) /...
           (max(throttlePosition) - min(throttlePosition)) * 100;
       
alpha = zeros(length(omega), 1);
alpha(1) = (omega(1) - 0) * time(1);
for n = 2 : length(omega) - 1
    alpha(n) = (omega(n+1) - omega(n)) * (time(n+1) - time(n)); %change in omega / time step
end

Iz = torque ./ alpha;
%}
%% Total Plots
%{
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
%}
%% Results from Specific Data Sets
%{
figure(2)
plot(time(570:1000), alpha(570:1000))
hold on
plot(time(570:1000), torque(570:1000)/1800)
hold off
xlabel('time [s]')
ylabel('Angular Acceleration [rad/s^2]')
title('Angular Acceleration vs. t')
legend('\alpha', '\tau');

figure(3)
plot(time(570:1000), powerDraw(570:1000))
hold on
plot([57, 100], [4032, 4032], ':')
hold off
xlabel('time [s]')
ylabel('Power Draw [W]')
title('Power Draw vs. t')

figure(4)
plot(time(570:1000), current(570:1000))
hold on
plot([57, 100], [80, 80], ':')
hold off
xlabel('time [s]')
ylabel('Current [A]')
title('Current vs. t')

figure(5)
plot(time(accelIndices), rotRate(accelIndices))
xlabel('time [s]')
ylabel('RPM')
title('RPM vs. t')
%}
%% Derived Results from Acceleration Periods Alone

accelIndices = [602:634, 756:783, 820:956];

timeAccel = [0 : length(accelIndices)-1]' * 0.1;
throttlePositionAccel = throttlePosition(accelIndices);
powerDrawAccel = powerDraw(accelIndices);
voltageAccel = voltage(accelIndices);
currentAccel = current(accelIndices);
rotRateAccel = rotRate(accelIndices);

rotRateAccel = rotRateAccel / polesMotor;

omegaAccel = 2 * pi * rotRateAccel / 60; %rotation rate [rad/s]

torqueAccel = zeros(length(powerDrawAccel), 1);
for n = 1:length(powerDrawAccel)
    if omegaAccel(n) ~= 0
        torqueAccel(n) = powerDrawAccel(n) / omegaAccel(n); %torque provided [Nm]
    else
        torqueAccel(n) = 0;
    end
end

%level above min throttle / total throttle range [%]
throttleAccel = (throttlePositionAccel - min(throttlePositionAccel)) /...
           (max(throttlePositionAccel) - min(throttlePositionAccel)) * 100;
       
alphaAccel = zeros(length(omegaAccel), 1);
alphaAccel(1) = (omegaAccel(1) - 0) * timeAccel(1);
for n = 2 : length(omegaAccel) - 1
    alphaAccel(n) = (omegaAccel(n+1) - omegaAccel(n)) * (timeAccel(n+1) - timeAccel(n));
end

k = 1;
for n = 1:length(powerDrawAccel)
    if (torqueAccel(n) ~= 0) && (alphaAccel(n) ~= 0)
        Iz(k) = torqueAccel(n) ./ alphaAccel(n);
        k = k + 1;
    end
end

%% Plots from Acceleration Periods Alone
figure(6)

subplot(2, 3, 1)
plot(timeAccel, throttleAccel)
xlabel('time [s]')
ylabel('Throttle Position [%]')
title('Throttle vs. t')

subplot(2, 3, 2)
plot(timeAccel, powerDrawAccel)
xlabel('time [s]')
ylabel('Power Draw [W]')
title('Power Draw vs. t')

subplot(2, 3, 3)
plot(timeAccel, voltageAccel)
xlabel('time [s]')
ylabel('Voltage [V]')
title('Voltage vs. t')

subplot(2, 3, 4)
plot(timeAccel, currentAccel)
xlabel('time [s]')
ylabel('Current [A]')
title('Current vs. t')

subplot(2, 3, 5)
plot(timeAccel, rotRateAccel)
xlabel('time [s]')
ylabel('Rotation Rate [rpm]')
title('Rotation Rate vs. t')

subplot(2, 3, 6)
plot(timeAccel, torqueAccel)
xlabel('time [s]')
ylabel('Torque [Nm]')
title('Torque vs. t')

figure(7)
plot(Iz)