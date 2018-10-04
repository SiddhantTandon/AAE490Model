function [P_actual, N_prop] = Power_Hover(T, rho, r, w, Cdp, s)
%% Description
% Determines the power required by the motors to maintain hovering
 
%% Inputs
% T        - Required Thrust [?]
% rho      - Gas Density [kg / m^3]
% w        - Angular Rate [rad/s]
% Cdp      - Propeller Drag Coefficient [-] (?)
% s        - Span [m^2] (?)

%% Outputs
% P_actual - Actual power required to maintain hover
% N_prop   - Propulsive Efficiency

%% ---Begin Code---

% Aerodynamic Correction Factor
% Remove hard code?
k = 1.2;

% Ideal Power Required to Hover
P_induced = T * sqrt( T / (2*rho*pi*(r^2)) );
P_ideal = k * P_induced;

% Power Required to Overcome Drag on Blades
P_drag = (Cdp * s * rho / 8) * (w*r)^3 * (pi*r^2);

% Actual Power Required to Hover
P_actual = P_ideal + P_drag;

% Calculate Figure of Merit
N_prop = P_ideal / P_actual;
    
%% ---End Code---

end