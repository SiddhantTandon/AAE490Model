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
T_motor_max = 70; %maximum allowable motor temperature [C]
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
P_rotor = 3181; %power required by 1 rotor [W]
eta = 0.85; %motor efficiency [fraction]
m_motor = 0.95; %motor mass [kg]
c_motor = 100; %approximate specific heat of motor (ranges from 15 to 420 for metals) [W/m*K]
r = 0.05; %motor radius [m]
l = 0.2; %motor height [m]
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


%{
%% Heat Pipes

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
%}

%% Phase Change Materials
%{
Notes:
1) Sized assuming no convective or radiative heat transfer
2) Cell array structure: {material, T_melt, H_delta_fus, c_p_liquid, c_p_solid
                            rho_solid, rho_liquid, k}
3) List of phase change materials: https://en.wikipedia.org/wiki/Phase-change_material
4) Many tunable PCM materials available
%}

% Material Properties
n = 1; %index of PCM material for analysis
materialOPtions = {'water'          , 0 , 300000, 4187, 2108, 916 , 995 , 2   ;
                   '0400-Q20 BioPCM', 20, 215000, 3200, 3500, 1075, 1125, 0.45};

PCM = materialOPtions{n, 1}; %phase change material
T_melt = materialOPtions{n, 2}; %melting point [C]
H_delta_fus = materialOPtions{n, 3}; %latent heat of fusion [J/kg]
c_p_liquid = materialOPtions{n, 4}; %heat capacity of liquid [J/kg*K]
c_p_solid = materialOPtions{n, 5}; %heat capacity of solid [J/kg*K]
rho_solid = materialOPtions{n, 6}; %density of solid [kg/m^3]
rho_liquid = materialOPtions{n, 7}; %density of liquid at maximum temp [kg/m^3]

T_melt = T_melt + 273.15; %melting point [K]
T_0 = max(T_mars, T_motor_min);
T_f = T_motor_max;

% Energy absorbtion from heating solid
if T_melt > T_0
    Q_solid_1kg = c_p_solid * (T_melt - T_0);
else
    Q_solid_1kg = 0;
end

% Energy absorbtion from melting
if T_melt <= T_f
    Q_melt_1kg = H_delta_fus;
else
    Q_melt_1kg = 0;
end

% Energy abosrtion from heating liquid
if T_melt < T_f
    Q_liquid_1kg = c_p_liquid * (T_f - T_melt);
else
    Q_liquid_1kg = 0;
end

% Total energy absorbably in change from T_0 to T_f
c_p_total = Q_solid_1kg + Q_melt_1kg + Q_liquid_1kg; %total heat capacity [J/kg]

% Mass of system
m_PCM = E_waste / c_p_total; %mass of PCM required to absorb heat [kg]
m_PCM_struc = m_PCM * 0.25; %estimate of additional structurla mass required [kg]

V_liquid = m_PCM / rho_liquid; %volume of liquid [m^3]
V_solid = m_PCM / rho_solid; %volume of solid [m^3]

percent_vol_change = (V_liquid - V_solid) / V_solid;

%alpha = materialOPtions{n, 8} / (rho_liquid * c_p_liquid);
%Bi = h_across_conv * pi * r^2 * l / (materialOPtions{n, 8} * A_noFin);


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

% Solution
fprintf('Cooling Method: Phase Change Materials\n')
fprintf('Material: %s\n', PCM);
if ((T_f - T_melt) < 5)
    fprintf('WARNING: melting temp. close to max motor operating temp.\n');
end
fprintf('Mass of PCM: %.2f kg\n', m_PCM);
fprintf('Approximate Mass of Additional Structure: %.2f kg\n', m_PCM_struc);
fprintf('Approximate Total Mass: %.2f kg\n', m_PCM_struc + m_PCM);
if V_liquid > V_solid
    fprintf('Maximum Volume of PCM: %.2f [L] (liquid volume at max temp)\n', V_liquid * 1000)
else 
    fprintf('Maximum Volume of PCM: %.2f [L] (solid volume)\n', V_solid * 1000)
end
fprintf('Percentage change in volume: %.2f%%\n', percent_vol_change*100)


