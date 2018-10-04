function [ ] = Motor( )
%% Description
% Determines motor sizing and performance metrics
 
%% Inputs


%% Outputs

%% Constants
numProp = 4;

%% --- Begin Code ---
%% Hover
% Calculate Power to Hover
[P_hover, ~] = Power_Hover(T, rho, r, w, Cdp, s);

% Mechanicla and Electrical Power 
P_mech_hover = numProp * P_hover;                  
P_elec_hover = P_mech_hover / motor_eff;
dQ = P_elec_hover * (1 - motor_eff); % Assuming extra power is converted to heat.

%% Climb

%% Descent


%% --- End Code ---

end

