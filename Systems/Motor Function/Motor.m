function [ Q_dot, E_req_flight, m_all_motors ] = Motor( t )
%% Description
% Determines motor sizing and performance metrics
 
%% Inputs
% t            - Time spent in each flight regime [s]
%                [Climb, Descend, Forward Flight, Hover]

%% Outputs
% Q_dot        - Rate of heat generation for each flight regime (W)
%                [Climb, Descend, Forward Flight, Hover]
% E_req_flight - Total amount of energy required for entire flight [J]

%% Constants
numProp = 4;
motor_eff = 0.85; % Rough Estimation (Can be found experimentally)

%% --- Begin Code ---
%% Hover
% Calculate Power to Hover
% These functions still need to be defined.
P_climb   = Power_Climb();
P_descend = Power_Descend();
P_forward = Power_ForwardFlight();
P_hover   = Power_Hover();

P = [P_climb P_descend P_forward P_hover];

% Mechanicla and Electrical Power 
P_mech = numProp * P;                  
P_elec = P_mech / motor_eff;
Q_dot = P_elec * (1 - motor_eff); % Assuming all extra power is converted to heat.

% Motor Mass
m_all_motors = motorMass( T_req, P_elec_one_motor, num_prop );

% Total Energy Required
E_req_flight = dot(P_elec, t);

%% --- End Code ---

end

