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
sigma = 5.67e-8; %Stefan-Boltzmann Constant [W/m^2*K^4]

v_freestream = 26.8; %maximum expected freestream velocity at the surface
T_mars = -100; %minimum ambient temperature on Mars [C]
c_p = 730; %specific heat capacity Mars atmosphere [m/s^2*K]
rho = 0.0139; %atmospheric density [kg/m^3]
k = 0.0096; %thermal conductivity [W/m*K]
mu = 1.422e-5; %dynamic viscosity [m^2/s]
epsilon = 0.05; %drone surface emissivity (0.05 for gold coating)

% Operational Specifications
T_drone = -20; %minimum safe operating temperature of the drone [C]
t_h = 13.65; %number of hours to calculate total heat loss for (typically length of night) 
t = 60 * 60 * t_h; %length of heat loss converted to seconds [s]

%Temperature 
deltaT = T_drone - T_mars; %temp. difference between motor & atmosphere
T_mars = T_mars + 273.15; %convert to K
T_drone = T_drone + 273.15; %convert to K


%Drone Surface Area
r = 0.1; %drone core radius [m]
h_drone = 0.1; %drone core height [m]
A_drone = pi * r ^ 2 * 2 + 2 * r * pi * h_drone;

%% Possible Heat Transfer Characteristics

Pr = mu * c_p / k; %Prandtl number

l_char_across = 2*r; %characteristic length of cylinder
Re_across = rho * v_freestream * (l_char_across) / mu; %Reynolds number
Nu_across = 0.3 + (0.62 * sqrt(Re_across) * Pr^(1/3) / (1 + (0.4/Pr)^(2/3)) ^ (1/4)) *...
           (1 + (Re_across/282000)^(5/8)) ^ (4/5); %Nusselt number
h_across_conv = Nu_across * k / l_char_across; %possible convective h.t. coef. (no fins) [W/m^2*K]
l_char_flat = 2*r; %characteristic length of cylinder
Re_flat = rho * v_freestream * (l_char_across) / mu;
Nu_flat = 0.664 * Re_flat^0.5 * Pr^(1/3); %Nusselt number, laminar
h_flat = Nu_flat * k / l_char_flat; %possible convective h.t. coef. (no fins) [W/m^2*K]
Q_conv = h_across_conv * h_drone * pi * 2 * r * deltaT + h_flat * 2 * pi * r ^ 2 * deltaT; %power dissipation via convection [J/s]

% Radiative heat transfer
Q_rad = epsilon * sigma * A_drone * (T_drone^4 - T_mars^4); %power dissipation via radiation [J/s]

%Rate of Heat Loss
Q_total = (Q_rad + Q_conv); %Rate of heat loss [J/s]

%Total Heat Loss
E_loss = t * (Q_rad + Q_conv); %Total energy loss [J]


%%Print Statements for Results

% Assumptions
fprintf('\nAssumed Conditions:\n');
fprintf('    1) Laminar flow over drone body\n');
fprintf('    2) Uniform heat distribution\n');
fprintf('    3) Neglect conduction\n\n');

%Environmental Conditions
fprintf('Environmental Conditions:\n');
fprintf('    Surface Temperature: %.2f kelvin\n', T_mars);
fprintf('    Drone Temperature: %.2f kelvin\n', T_drone);
fprintf('    Wind Speed: %.2f m/s\n\n', v_freestream)

%Drone Body Configuration
fprintf('Drone Body Configuration:\n');
fprintf('    Drone body height: %.2f m\n',h_drone);
fprintf('    Drone body radius: %.2f m\n',r);
fprintf('    Drone body surface area: %.4f m^2\n',A_drone);
fprintf('    Drone body emissivity coefficient of %.2f \n\n',epsilon);

% Heat Loss Rates
fprintf('Heat Loss Rates:\n');
fprintf('    Heat loss rate due to radiation: %.2f W\n',Q_rad);
fprintf('    Heat loss rate due to convection: %.2f W\n',Q_conv);
fprintf('    Total heat loss rate: %.2f W\n\n',Q_total);

% Total Heat Loss
fprintf('Total heat loss for %.2f hours: %.2f J\n', t_h,E_loss);


