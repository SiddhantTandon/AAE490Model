clear;close all;clc

% Sizing code for radiatior  

% Material Properties
epsilon = 0.7;          % Emmisivity  (Reference:  https://www.engineeringtoolbox.com/emissivity-coefficients-d_447.html)
k = 100;                % [W/(m*K)] Thermal Condictivity of fin material (Reference:  https://www.engineeringtoolbox.com/thermal-conductivity-d_429.html)
rho_material = 1800;    % [km/m^3] Density of fin material

% Fin Temperature
T_fin = 273 + 40;    % [K]
T_env = 0;           % [K]

% Geometry
t_fin = 3/1000;     % [m] Fin thickness
L_fin = 0.2;        % [m] Length that a single fin extends from the center of the radiator
L_arm = 0.7;        % [m] Length of rotor arm
perim_arm = 0.1;    % [m] external perimeter of the structural arm itself, assuming 2 parallel rectangular cross sections of 3 cm by 1 cm (only considering outside perimeter)

% Constants
sigma = 5.67e-8;


% Calculations 
A_ext_arm = L_arm * perim_arm;          % [m^2]
A_one_fin = 2 * L_fin * L_arm;          % [m^2] Multiply by 2 to account for 2 sides of a single fin
V_fin = 4 * t_fin * L_fin * L_arm;      % [m^3]   
m_fin = rho_material * V_fin            % [kg]


Q_dot_arm = A_ext_arm * sigma * T_env^4;
Q_dot_isotherm = 4 * A_one_fin * sigma * T_fin^4    % [W] Simplified Const. Temperature fin heat transfer rate 

Q_dot_fin = 4 * 2*k*t_fin*L_fin * (sigma * epsilon/(5 * k * t_fin))^0.5 * (T_fin^5 - T_env^5)^0.5    % [W] Heat transfer rate using fin equation 












