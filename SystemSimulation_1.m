clear all
close all
clc

% Overall simulation for the optimization for one drone
%% Make flight, at rest w/out charging, and at rest w/ recharge a sum of multiple of sols/integer of those sols
%Can calculate the acceleration after the rotor function is called
%Only include power requirement for most costly sensor in sensor package during cruise
%Maybe add togle for solar panels if going back to hub is decided for mission

%% Sensor Type Data. Different sensor types are designated with different input values
% 1 = Hyperspectral Camera - run during cruise
% 2 = Infared Spectrometer for methane deposits - run during cruise
% 3 = Lidar - run during take off and landing
% 4 = Low Res-Camera - run during cruise
% 5 - Short range comm - run during recharge time
% 6 - Satellite/Earth link comm - run during recharge time

pwrdraw = [];           %watts of power per sensor type
fov = [];               %degrees for field of view for sensor type
maxrange = [];          %meters that sensor type can be functional
datarate = [];          %kb/s of data transferred for each sensor type
mass = [];              %kg of mass per sensor type

%% Mars enviornmental conditions

rho = .02;              %kg/m^3 density at surface
a = 240;                %m/s speed of sound
mu = 1.422*10^(-5);     %kg/(m*s) kinematic viscocity
g = 3.71;               %m/s^2 gravity on martian surface
Sflux = [];             %  solar flux on the martian surface

%% Iterated Values

OMass = [];             %Overall mass assumed for one drone
Chargetime = [];        %Seconds of charge time between flights
Maxflttime = [];        %Maximum flight time for drone in seconds
Altitude = [];          %Altitudes the drone will cruise

%% Other designated constants

MaxMissionArea = pi*25^2;   %Mission designated area km^2
MaxMissionTime = 90;        %Sols of mission time
SF = [];                    %Safety Factor inherent in design
verticlefltspeed = [];      %m/s Verticle flight speed
Sensorsonboard = [1 0 1 1 1 0]; %Designate which sensor systems are implemented on drone
%Avionics funtion provides power draw for systems watts

%% Iteration for all given values for main portion of code

for mass = OMass
    %Rotor only requires mass
    %Motor only requires motor
    for Alt = Altitude
        for MFT = Maxflttime
            %Battery requires Rotor, Motor, Avionics, Altitude, Max Flight Time
            for CT = Chargetime
                %Solar Panel/Recharge battery requries battery
                %Structures requires everything
            end
        end
    end
end

%% Grading Criterion for each iteration (1-5: 5 being most important)
% Lower mass systems 5
% Charge time lower 3
% Area covered in single flight 3
% Altitude for risk avoidance (lower is better generally) 2
% Minimize the maximum linear dimension (can determine # of drones and wether secondary or primary mission) 4




