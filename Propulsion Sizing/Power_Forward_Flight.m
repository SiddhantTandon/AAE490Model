function [P_forward] = Power_Forward_Flight(weight, rho, radius, k, a, tipMach, Cdp, s, beta, V_inf)

% Constants
k_span_drag = 4.6; % factor due to spanwise drag, value taken from textbook Ch 5, pg 133
f = 0.016*pi*radius^2; % equivalent flat plate area, related to parasitic drag, value taken from Ch 5, pg 136
Ab = s*pi*radius^2; % [m^2] blade area

% Calculate velocities at the rotor based on inclination angle (beta) of rotor disk
Vx = V_inf * cosd(beta);
Vz = V_inf * sind(beta);
Vtip = (tipMach * a)-Vx;

% Calculate induced velocity based on weight and rotor inclination angle
T = weight / cosd(beta);
lambda_i0 = sqrt(T / (2*rho*pi*radius^2)) / Vtip;
ux = Vx / Vtip;
uz = Vz / Vtip;
ux_bar = ux / lambda_i0;
uz_bar = uz / lambda_i0;
p = [1, 2*uz_bar, (ux_bar^2 + uz_bar^2), 0, -1];
lambda_i_bar = roots(p);
lambda_i = lambda_i_bar * lambda_i0;
vi = lambda_i * Vtip;
% vi_2 = (T / (2*rho*pi*radius^2)) * (1/sqrt(Vx^2 + (Vz+real(vi(4)))^2));
vi = real(vi(4));

% Calculate power required
P_1 = k*vi*T;
P_2 = (1/8)*Cdp*rho*Ab*Vtip^3;
P_3 = 1 + k_span_drag*(V_inf/Vtip)^2;
P_4 = 0.5*rho*V_inf^3*f;
P_forward = P_1 + P_2*P_3 + P_4;