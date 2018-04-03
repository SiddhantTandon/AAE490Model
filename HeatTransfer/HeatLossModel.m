clear
clc

%{
Assumptions:
1) Laminar flow over drone body
2) Uniform heat distribution
3) Neglect conduction

Conclusions:
Important variables:
    surface area for convection
	
    

Irrelevant variables:

%}

%% Constants/Inputs


% Environmental Constants
g = 3.711; %gravitational acceleration [m/s^2]
sigma = 5.67e-8; %Stefan-Boltzmann Constant [W/m^2*K^4]

v = 26.8; %maximum expected freestream velocity at the surface
T_mars = -120; %minimum ambient temperature on Mars [C]
c_p = 730; %specific heat capacity Mars atmosphere [m/s^2*K]
rho = 0.0139; %atmospheric density [kg/m^3]
k = 0.0096; %thermal conductivity [W/m*K]
mu = 1.422e-5; %dynamic viscosity [m^2/s]
epsilon = 0.05; %drone surface emissivity (0.05 for gold coating)

% Operational Specifications
T_drone = -20; %minimum safe operating temperature of the drone [C]
t = 60 * 60 * 12; %length of heat loss (typically length of night) [s]

%Temperature 
deltaT = T_drone - T_mars; %temp. difference between motor & atmosphere
T_mars = T_mars + 273.15; %convert to K
T_drone = T_drone + 273.15; %convert to K


%Drone Surface Area
L_drone = 1; %length of one drone side panel [m]
H_drone = 0.157; %height of drone core[m]
A_drone = L_drone * L_drone * 2 + L_drone * H_drone * 4; %surface area of the top, bottom, and sides of the main drone housing [m^2]


%% Possible Heat Transfer Characteristics

Pr = mu * c_p / k; %Prandtl number

l_char = L_drone; %Characteristic length is equal to length if the flow is laminar
Re = v * l_char * rho / mu; %Reynolds number
Nu = 0.664 * Re^0.5 * Pr^(1/3); %Nusselt number, laminar
%Nu = 0.037 * Re^0.8 * Pr^(1/3); %Nusselt number, turbulent
h_conv = Nu * k / l_char; %possible convective h.t. coef. (no fins) [W/m^2*K]
Q_conv = h_conv * A_drone * deltaT; %power dissipation via convection [J/s]

% Radiative heat transfer
Q_rad = epsilon * sigma * A_drone * (T_drone^4 - T_mars^4); %power dissipation via radiation [J/s]

%Total Heat Loss
E_loss = t * (Q_rad + Q_conv); %Total energy loss [J]




% Assumptions
fprintf('\nAssumed Conditions:\n')


% Heat Loss Rates
fprintf('Heat Loss Rates:\n');

