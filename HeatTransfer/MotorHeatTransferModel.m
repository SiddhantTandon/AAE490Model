clear
clc

%{
Assumptions:
1) Motor is uniform body - model as lumped system

Conclusions:
Important variables:
    motor efficiency
    surface area for convection
    power

Irrelevant variables:
    motor operating temperature
    motor mass
    flight time
    motor effective specific heat
%}

%% Constants/Inputs

% Operational Specifications
T_motor = 100; %maximum allowable motor temperature [C]
t = 10 * 60; %flight time [s]
v_freestream = 40; %flight velocity [m/s]
v_down = 40; %downward flow speed [m/s]

% Environmental Constants
g = 3.711; %gravitational acceleration [m/s^2]
sigma = 5.67e-8; %Stefan-Boltzmann Constant [W/m^2*K^4]

T_mars = -50; %ambient temperature on Mars [C]
c_p = 730; %specific heat capacity Mars atmosphere [m/s^2*K]
rho = 0.0139; %atmospheric density [kg/m^3]
k = 0.0096; %thermal conductivity [W/m*K]
mu = 1.422e-5; %dynamic viscosity [m^2/s]
epsilon = 0.98; %motor surface emmisivity
%alpha = k / (c_p * rho); %thermal diffusivity of atmosphere [m^2/s]

% Motor Specifications
P_rotor = 5578; %power required by 1 rotor [W]
eta = 0.875; %motor efficiency [fraction]
m = 0.75; %motor mass [kg]
c_motor = 100; %approximate specific heat of motor (ranges from 15 to 420 for metals) [W/m*K]
r = 0.05; %motor radius [m]
l = 0.2; %motor height [m]
A_noFin = (2 * pi * r * l + 2 * pi * r^2); %surface area of motor (no fins) [m^2]


%% Total Wasted Power

deltaT = T_motor - T_mars; %temp. difference between motor & atmosphere
T_mars = T_mars + 273.15; %convert to K
T_motor = T_motor + 273.15; %convert to K

P_dis = (P_rotor / eta) * (1 - eta); %power lost by 1 motor


%% Possible Heat Transfer Characteristics

Pr = mu * c_p / k; %Prandtl number

% Motor temperature increase
Q_dot_accum = m * c_motor * deltaT / t; %rate energy can be absorbed by motor over flight w/o overheating [W]
%h_req = Q_dot_req / (A_noFin * deltaT); %required convective h.t. coef. (no fins) [W/m^2*K]

% Convective flow across cylinder
l_char_across = 2*r; %characteristic length = diameter
Re_across = rho * v_freestream * (l_char_across) / mu; %Reynolds number
Nu_across = 0.3 + (0.62 * sqrt(Re_across) * Pr^(1/3) / (1 + (0.4/Pr)^(2/3)) ^ (1/4)) *...
           (1 + (Re_across/282000)^(5/8)) ^ (4/5); %Nusselt number
h_across_conv = Nu_across * k / l_char_across; %possible convective h.t. coef. (no fins) [W/m^2*K]
Q_dot_across_conv = h_across_conv * A_noFin * deltaT;


% Convective flow down over cylinder
l_char_down = l;
Re_down = rho * v_down * (l_char_down) / mu; %Reynolds number
Nu_down = 0.664 * Re_down^0.5 * Pr^(1/3); %Nusselt number, laminar
%Nu_down = 0.037 * Re_down^0.8 * Pr^(1/3); %Nusselt number, turbulent
h_down_conv = Nu_down * k / l_char_down; %possible convective h.t. coef. (no fins) [W/m^2*K]
Q_dot_down_conv = h_down_conv * A_noFin * deltaT;


% Radiative heat transfer
Q_dot_rad = epsilon * sigma * A_noFin * (T_motor^4 - T_mars^4); %power dissipation via radiation


% Remaining heat to dissipate
Q_dot_remainder = P_dis - Q_dot_accum - Q_dot_rad - max(Q_dot_across_conv, Q_dot_down_conv);

%% Potential Solutions

% Heat pipes
lengthPipes = 1; %pipe length in m

linearDensity = 0.0184 / 0.3; %18.4g/300mm
Q_dotPerPipe = 30; %can transfer 30W per pipe

numPipes = round(Q_dot_remainder / Q_dotPerPipe);

massPipes = numPipes * linearDensity * lengthPipes;
Q_dot_pipe_total = numPipes * Q_dotPerPipe;

fprintf('Number of Pipes: %d\n', numPipes);
fprintf('Total Pipe Mass: %.2f kg\n', massPipes);
fprintf('Heat dissipation rate: %.2f W\n', Q_dot_pipe_total);
% DOESN'T INCLUDE MASS OF HEAT SINKS
%% Results

% Assumptions
fprintf('\nAssumed Conditions:\n')
fprintf('Motor Efficiency: %.0f%%\n', eta*100);
if Q_dot_across_conv > Q_dot_down_conv
    fprintf('Freestream Velocity: %.2f m/s\n', v_freestream);
else
    fprintf('Downward Velocity: %.2f m/s\n', v_down);
end
fprintf('Power to Rotor: %.2f W\n\n', P_rotor);

% Heating rates
fprintf('Heating Rates:\n');
fprintf('                Total power wasted: %.2f W\n', P_dis);
fprintf('    Heat absorbed by motor heating: %.2f W\n', -Q_dot_accum);
fprintf(' Heat dissipated through radiation: %.2f W\n', -Q_dot_rad);
if Q_dot_across_conv > Q_dot_down_conv
    fprintf('Heat dissipated through convection: %.2f W (across cylinder)\n', -Q_dot_across_conv);
else
    fprintf('Heat dissipated through convection: %.2f W (down cylinder)\n', -Q_dot_down_conv);
end
fprintf('------------------------------------------\n');
fprintf('   Remaining heat to be dissipated: %.2f W\n\n', Q_dot_remainder);





