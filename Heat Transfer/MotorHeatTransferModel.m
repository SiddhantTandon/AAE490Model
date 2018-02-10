clear
clc

%{
Assumptions:
1) Motor is uniform body - model as lumped system
2) To calculate h based on Mars environment, convection across surface of
   cylinder is modeled as convection over flat plate
3) In calculating Ra, free convection near wall (in theory relationship
   only valid when Ra > 10^12
4) In calculating Ra, characteristic length is motor height
5) In calculating h, l is the characteristic length w.r.t. gravity

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
T_motor = 50; %maximum allowable motor temperature [C]
t = 10 * 60; %flight time [s]
v = 40; %flight velocity [m/s]

% Environmental Constants
g = 3.711; %gravitational acceleration [m/s^2]

T_mars = -100; %ambient temperature on Mars [C]
c_p = 730; %specific heat capacity Mars atmosphere [m/s^2*K]
rho = 0.0139; %atmospheric density [kg/m^3]
k = 1.1691e-7 * T_mars^2 + 1.3327e-5 * T_mars + 2.2469e-3; %thermal conductivity (temp in C) [W/m*K]
mu = 1.422e-5; %dynamic viscosity [m^2/s]
nu = mu / rho; %kinematic viscosity [W/s^2]
alpha = k / (c_p * rho); %thermal diffusivity of atmosphere [m^2/s]

% Motor Specifications
P_rotor = 3342; %power required to run 1 rotor [W]
eta = 0.75; %motor efficiency [fraction]
m = 0.4425; %motor mass [kg]
c_motor = 100; %approximate specific heat of motor (ranges from 15 to 420 for metals) [W/m*K]
r = 0.05; %motor radius [m]
l = 0.2; %motor height [m]

%% Calculations

l_char = 2*r; 

Pr = mu * c_p / k; %Prandtl number
Re = rho * v * (l_char) / mu; %Reynolds number

P_motor = P_rotor / eta;

deltaT = T_motor - T_mars; %temp. difference between motor & atmosphere
T_mars = T_mars + 273.15; %convert to K
T_motor = T_motor + 273.15; %convert to K

Q_dot = P_motor * (1 - eta) - m * c_motor * deltaT / t; %required rate of heat transfer away from motor [W]



A_noFin = 2 * pi * r * l + 2 * pi * r^2; %surface area of motor (no fins) [m^2]

h_reqNoFin = Q_dot / (A_noFin * deltaT); %required convective h.t. coef. (no fins) [W/m^2*K]

% Reasonable heat transfer coeffient (h) on Mars
h = (k / l) * (0.037 * Re^0.8 * Pr^(1/3));

%beta = 1 / T_mars; %thermal expansion coefficient (1/T for ideal gasses) [1/K]
%Ra = g * beta / (nu * alpha) * deltaT * x^3; %Rayleigh number






