clear
clc

%{
Initially adapted from MotorHeatTransferModel.m
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
T_motor = 70; %maximum allowable motor temperature [C]
t = 10 * 60; %flight time [s]
v_freestream = 30; %flight velocity [m/s]
v_down = 30; %downward flow speed [m/s]

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
P_rotor = 3181; %power required by 1 rotor [W]
eta = 0.85; %motor efficiency [fraction]
m = 0.95; %motor mass [kg]
c_motor = 100; %approximate specific heat of motor (ranges from 15 to 420 for metals) [W/m*K]
r = 0.05; %motor radius [m]
l = 0.2; %motor height [m]
A_noFin = (2 * pi * r * l + 2 * pi * r^2); %surface area of motor (no fins) [m^2]

Pr = mu * c_p / k; %Prandtl number

%% Total Wasted Power

deltaT = T_motor - T_mars; %temp. difference between motor & atmosphere
T_mars = T_mars + 273.15; %convert to K
T_motor = T_motor + 273.15; %convert to K

P_dis = (P_rotor / eta) * (1 - eta); %power lost by 1 motor

%% LiquidCoolPump

%Cylinder Inner Heat Contact Area, Assume r and l
l_in = l * 0.8; 
r_in = r * 0.8; 
A_in = (2 * pi * r_in * l_in + 2 * pi * (r_in)^2);
L_in = 0.003; % [m] which is 1/8 inch tube thickness
Apipe = 0.8 * A_in; %Area of pipe contact with coils, assume pipe is not in contact with full wire
Dpipe = 2*r_in/6; % for 12 in and 12 out tubes fitting in A_in cylinder 

%Thermo constants
Vpipe = 26589; %m/s, design variable to iterate until Tatm2 ~= 0 degrees C
k_Al = 235; %W/m.K  Aluminum Heat transfer tubes
rho_pipe = 62; %kg/m^3 for pentane
mu_pipe = 2.24*(10^-4); %mu for pentane [Pa.s]

Re_pipe = rho * Vpipe * Dpipe / mu_pipe;
%Nu_pipe = 0.664 * Re_pipe^0.5 * Pr^(1/3); %Nusselt number, laminar
Nu_pipe = 0.037 * Re_pipe^0.8 * Pr^(1/3); %Nusselt number, turbulent
h = Nu_pipe * k / Dpipe;

%States
% T1 = T_motor; 
% T2 = T_fl_hot;
% T3 = T_fl_cold;

%Thermo
Q = P_dis;
Qcond = P_dis;
Qconv = P_dis;


%qcond = k*A_in*(T1-Tpipe)/L
Tpipe1 = -(Qcond*L_in /(k_Al*A_in)) + T_motor;
%qconv = k*A_in*(T1-Tpipe)/L
T2 = -(Qconv/(h*Apipe)) + Tpipe1;
Tpipe2 = -(Qconv/(h*Apipe)) + T2;
Tatm2 = -(Qcond*L_in /(k_Al*A_in))+ Tpipe2;
Arad = Q/(epsilon * sigma * (Tatm2^4));

m_dot = rho_pipe * Vpipe * (pi*((Dpipe/2)^2));
h_cold = -82426; %[J/kg] enthalphy of pentane at 0 C and 1 atm
h_hot = 633477; %[J/kg] enthalphy of pentane at T2 = 171.5727 C and 1 atm
W_pump = m_dot*( h_cold - h_hot);



%% Results
fprintf('Total heat dissapation unaccounted for: %.2f W\n\n', P_dis);

% LiquidCoolPump
fprintf('Fluid: Pentane, Tubing: Aluminum\n');
fprintf('Thickness of pipe: %.2f mm\n', L_in*1000);
fprintf('Velocity of fluid in pipe: %.2f m/s\n', Vpipe);
fprintf('Reynolds number in pipe: %.2f \n       Note: Re > 2300 is assumed turbulent flow\n', Re_pipe);
fprintf('Temperature of heated fluid: %.2f degrees C\n', T2);
fprintf('Temperature of Mars atmosphere in radiation calculation: %.2f degrees C\n', Tatm2);
fprintf('Area of radiator: %.2E m^2\n', Arad);
fprintf('Mass flow rate in pipes: %.2f kg/s\n', m_dot);
fprintf('Pump work: %.2E W\n        Note: Work into system is negative\n', W_pump);


