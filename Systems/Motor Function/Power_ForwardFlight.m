function [ P_forward, P_forward_accel ] = Power_ForwardFlight( )
%% Description
% Determines the power required by the motors to fly forward and
% accellerate forward
 
%% Inputs
% W                - Weight of Quadcopter [N]
% r                - Radius of Propeller [m]

%% Outputs
% P_forward        - 
% P_forward_accell - 

%% Constants
k_span_drag = 4.6;       % Spanwise Drag Factor [-]
f  =  0.15*pi*r.^2; % Equivalent flat plate area [m^2]
Ab = s*pi*r.^s;     % Blade Area [m^2]

%% ---Begin Code---
% Power Required to Accelerate Forward




% Steady State Power Required for Forward Flight
Vx    = V_inf .* cosd(beta_cruise);
Vy    = V_inf .* sind(beta_cruise);
tipV = tipMach * a - Vx;

% Velocities at Rotors
T = W ./ cosd(beta_cruise);
lambda_i0 = sqrt(T ./ (2*rho*pi*r.^2)) ./ tipV;
ux = Vx ./ Vtip;
uy = Vy ./ Vtip;
ux_bar = ux ./ lambdai0;
uz_bar = uy ./ lambdai0;
p = [1, 2.*uz_bar, (ux_bar.^2 + uz_bar.^2), 0, -1];
lambda_i_bar = roots(p);
lambda_i = lambda_i_bar .* lambda_i0;
vi = real(lambda_i(4) .* tipV);

% Power Required
P1 = k .* vi .* T;
P2 = (1/8) .* Cdp .* rho .* Ab .* tipV.^3;
P3 = 1 + k_span_drag .* (V_inf ./ tipV).^2;
P4 = (1/2) .* rho .* f .* V_inf.^3;
P_forward = P1 + P2 + P3 + P4;

%% ---End Code---

end

function [  ] = func( t ) 

V_inf = t * accel_forward;
Vx = V_inf * cosd( beta_accel );
Vy = V_inf * sind( beta_accel );
Vtip = tipMach * a - Vx

T = (weight / cosd(beta_accel));
lambda_i0 = sqrt(T / (2*rho*pi*radius^2)) / Vtip;
ux = Vx / Vtip;
uz = Vz / Vtip;
ux_bar = ux / lambda_i0;
uz_bar = uz / lambda_i0;
p = [1, 2*uz_bar, (ux_bar^2 + uz_bar^2), 0, -1];
lambda_i_bar = roots(p);
lambda_i = lambda_i_bar * lambda_i0;
vi = lambda_i * Vtip;
vi_2 = (T / (2*rho*pi*radius^2)) * (1/sqrt(Vx^2 + (Vz+real(vi(4)))^2));
vi = real(vi(4));

% Calculate power required
P_1 = k*vi*T;
P_2 = (1/8)*Cdp*rho*Ab*Vtip^3;
P_3 = 1 + k_span_drag*(V_inf/Vtip)^2;
P_4 = 0.5*rho*V_inf^3*f;
P_forward_accel = (P_1 + P_2*P_3 + P_4);


end