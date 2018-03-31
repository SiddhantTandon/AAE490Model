function [P_forward, P_forward_accel, t_forward_accel_hr] = Power_Forward_Flight(weight, rho, radius, k, a, tipMach, Cdp, s, beta_cruise, beta_accel, V_inf, accel_forward)

% Constants
k_span_drag = 4.6;          % factor due to spanwise drag, value taken from textbook Ch 5, pg 133
f = 0.016*pi*radius^2;      % equivalent flat plate area, related to parasitic drag, value taken from Ch 5, pg 136
Ab = s*pi*radius^2;         % [m^2] blade area

%% Calculate power required per rotor to accelerate forward
% Calculate power and battery capacity required to accelerate
V_inf_0 = 0; %m/s
V_inf_final = V_inf; %m/s
t_accel = (V_inf_final - V_inf_0)/accel_forward;
t_step = 0.1;
i = 1;
for t = 0.1:t_step:t_accel
    V_inf = t*accel_forward;
    % Calculate velocities at the rotor based on inclination angle (beta) of rotor disk
    Vx = V_inf * cosd(beta_accel);
    Vz = V_inf * sind(beta_accel);
    Vtip = (tipMach * a)-Vx;
    
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
    P_forward_accel(i) = (P_1 + P_2*P_3 + P_4);
    
    i = i+1;
end

P_forward_accel = max(P_forward_accel);                             % max power required during accel/decel [W]
t_forward_accel = (2*(V_inf_final - V_inf_0)/accel_forward);          % total time required to accel and decel [s]
t_forward_accel_hr = t_forward_accel/3600;                              % total time required to accel and decel [hr]

%% Calculate power required per rotor to fly forward
% Calculate velocities at the rotor based on inclination angle (beta) of rotor disk
Vx = V_inf * cosd(beta_cruise);
Vz = V_inf * sind(beta_cruise);
Vtip = (tipMach * a)-Vx;

T = weight / cosd(beta_cruise);

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
P_forward = (P_1 + P_2*P_3 + P_4);
