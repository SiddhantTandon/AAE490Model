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

% Motor Operational Specifications
T_motor_max = 110; %maximum allowable motor temperature [C]
T_motor_min = -20; %minimum allowable motor temperature
t = 10 * 60; %flight time [s]
v_freestream = 30; %flight velocity [m/s]
v_down = 30; %downward flow speed [m/s]

% Environmental Constants
g = 3.711; %gravitational acceleration [m/s^2]
sigma = 5.67e-8; %Stefan-Boltzmann Constant [W/m^2*K^4]

T_mars = -0; %ambient temperature on Mars [C]
c_p_Mars = 730; %specific heat capacity Mars atmosphere [m/s^2*K]
rho_Mars = 0.0139; %atmospheric density [kg/m^3]
k_Mars = 0.0096; %thermal conductivity [W/m*K]
mu_Mars = 1.422e-5; %dynamic viscosity [m^2/s]

Pr = mu_Mars * c_p_Mars / k_Mars; %Prandtl number

% Motor Specifications
P_rotor = 4329; %power required by 1 rotor [W]
eta = 0.85; %motor efficiency [fraction]
m_motor = 1.25; %motor mass [kg]
c_motor = 100; %approximate specific heat of motor (ranges from 15 to 420 for metals) [W/m*K]
r = 0.04; %motor radius [m]
l = 0.075; %motor height [m]
epsilon = 0.98; %motor surface emmisivity

A_noFin = (2 * pi * r * l + 2 * pi * r^2); %surface area of motor (no fins) [m^2]


%% Total Wasted Power

%P_waste = (P_rotor / eta) * (1 - eta); %power lost by 1 motor
P_waste = 600; %set power lost by 1 motor (for consistancy)
E_waste = P_waste * t;

deltaT = T_motor_max - T_mars; %temp. difference between motor & atmosphere
T_mars = T_mars + 273.15; %convert to K
T_motor_max = T_motor_max + 273.15; %convert to K
T_motor_min = T_motor_min + 273.15; %convert to K

%% Baseline Heat Transfer Characteristics

% Motor temperature increase
%rate energy can be absorbed by motor over flight w/o overheating
Q_dot_accum = m_motor * c_motor * deltaT / t; %[W]

% Convective flow across cylinder
l_char_across = 2*r; %characteristic length = diameter
Re_across = rho_Mars * v_freestream * (l_char_across) / mu_Mars; %Reynolds number
Nu_across = 0.3 + (0.62 * sqrt(Re_across) * Pr^(1/3) / (1 + (0.4/Pr)^(2/3)) ^ (1/4)) *...
           (1 + (Re_across/282000)^(5/8)) ^ (4/5); %Nusselt number
h_across_conv = Nu_across * k_Mars / l_char_across; %possible convective h.t. coef. (no fins) [W/m^2*K]
Q_dot_across_conv = h_across_conv * A_noFin * deltaT;


% Convective flow down over cylinder
l_char_down = l;
Re_down = rho_Mars * v_down * (l_char_down) / mu_Mars; %Reynolds number
Nu_down = 0.664 * Re_down^0.5 * Pr^(1/3); %Nusselt number, laminar
%Nu_down = 0.037 * Re_down^0.8 * Pr^(1/3); %Nusselt number, turbulent
h_down_conv = Nu_down * k_Mars / l_char_down; %possible convective h.t. coef. (no fins) [W/m^2*K]
Q_dot_down_conv = h_down_conv * A_noFin * deltaT;


% Radiative heat transfer
Q_dot_rad = epsilon * sigma * A_noFin * (T_motor_max^4 - T_mars^4); %power dissipation via radiation


% Remaining heat to dissipate
Q_dot_remainder_baseline = P_waste - Q_dot_accum - Q_dot_rad - max(Q_dot_across_conv, Q_dot_down_conv);

%% Temperature Calculations

T_f_motor_insulated = E_waste / (m_motor * c_motor) + T_mars; 
T_f_motor_baseline = Q_dot_remainder_baseline * t / (m_motor * c_motor) + T_mars;
T_f_motor_cooled = Q_dot_remainder_baseline;

%% Results

% Assumptions
fprintf('Assumed Conditions:\n')
fprintf('Motor Efficiency: %.0f%%\n', eta*100);
fprintf('Maximum/Steady State motor Temperature: %.2f C\n', T_motor_max - 273.15);
fprintf('Minimum/initial motor Temperature: %.2f C\n', T_0 - 273.15);
if Q_dot_across_conv > Q_dot_down_conv
    fprintf('Freestream Velocity: %.2f m/s\n', v_freestream);
else
    fprintf('Downward Velocity: %.2f m/s\n', v_down);
end
fprintf('Power to Rotor: %.2f W\n\n', P_rotor);

% Heating rates
fprintf('Baseline Heating Rates:\n');
fprintf('                Total power wasted: %.2f W\n', P_waste);
fprintf('    Heat absorbed by motor heating: %.2f W\n', -Q_dot_accum);
fprintf(' Heat dissipated through radiation: %.2f W\n', -Q_dot_rad);
if Q_dot_across_conv > Q_dot_down_conv
    fprintf('Heat dissipated through convection: %.2f W (across cylinder)\n', -Q_dot_across_conv);
else
    fprintf('Heat dissipated through convection: %.2f W (down cylinder)\n', -Q_dot_down_conv);
end
fprintf('---------------------------------------------\n');
fprintf('   Remaining heat to be dissipated: %.2f W\n\n', Q_dot_remainder_baseline);

% Motor Temperatures
fprintf('Motor Temperatures:\n')
fprintf('      Motor Temperature with no dissipation: %.2f K\n', T_f_motor_insulated);
fprintf('Motor Temperature with baseline dissipation: %.2f K\n\n', T_f_motor_baseline);
