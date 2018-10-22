function [ Q_dot, E_req_flight, m_all_motors ] = Motor( P_climb, P_descend, P_hover, Alt_cruise, v_climb, v_descend, t_cruise, t_hover, weight, rho, radius, a, tipMach, s, beta_cruise, beta_accel, V_inf, accel_forward, w_max )
%% Description
% Determines motor sizing and performance metrics
 
%% Inputs
% P_climb        - Power required to climb [W]
% P_descend      - Power required to descend [W]
% P_hover        - Power required to hover [W]
% v_climb        - Climb velocity [m/s]
% v_descend      - Descend velocity [m/s]
% Alt_cruise     - Cruise/Flight Altitude [m]
% weight         - Weight of the Vehicle [N]
% rho            - Atmospheric Density [kg/m^3]
% radius         - Radius of propellers [m]
% a              - Speed of Sound [m/s]
% tipMach        - Mach number at propeller tips [-]
% Cdp            - Coefficient of Proppeller Drag [-]
% s              - Rotor Solidity
% beta_cruise    - Angle of tilt from horizontal of rotor disk in cruise
% [deg]
% beta_accel     -  Angle of tilt from horizontal of rotor disk in accel
% [deg]
% V_inf          - Cruise Velocity [m/s]
% accel_forward  - Forward Acceleration [m/s^2]
% t_cruise       - Time spent in cruise [s]
% t_hover        - Time spent in hover [s]
% w_max          - Angular rotation rate at max loading [rpm]

%% Outputs
% Q_dot        - Rate of heat generation for each flight regime (W)
%                [Climb, Descend, Forward Flight, Hover]
% E_req_flight - Total amount of energy required for entire flight [J]

%% Constants
numProp = 4;
motor_eff = 0.80; % Rough Estimation (Can be found experimentally)

%% --- Begin Code ---
%% Hover
P_forward = Power_Forward_Flight(weight, rho, radius, a, tipMach, s, beta_cruise, beta_accel, V_inf, accel_forward);
P = [P_climb P_descend P_forward P_hover];

% Mechanical and Electrical Power 
P_mech = numProp * P;                  
P_elec = P_mech / motor_eff;
P_elec_one_motor = P_elec / numProp;
Q_dot = P_elec * (1 - motor_eff); % Assuming all extra power is converted to heat.

% Motor Mass
T_req = max(P_mech) / (w_max * 9.5492965964254);
m_all_motors = motorMass( T_req, P_elec_one_motor, numProp );

% Total Energy Required
t_climb = Alt_cruise / v_climb;
t_descend = Alt_cruise / v_descend;
t = [ t_climb t_descend t_cruise, t_hover ];

E_req_flight = dot(P_elec, t);

%% --- End Code ---

end

