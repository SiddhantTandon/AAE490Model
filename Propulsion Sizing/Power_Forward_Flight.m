function [P_forward, cap_batt_forward_accel, P_mech_forward_accel, P_elec_forward_accel] = Power_Forward_Flight(weight, rho, radius, k, a, tipMach, Cdp, s, beta, V_inf, accel_forward, Idraw, V_batt, motor_eff, numProp)

% Constants
k_span_drag = 4.6; % factor due to spanwise drag, value taken from textbook Ch 5, pg 133
f = 0.016*pi*radius^2; % equivalent flat plate area, related to parasitic drag, value taken from Ch 5, pg 136
Ab = s*pi*radius^2; % [m^2] blade area

vehicle_mass = weight/3.71;



% Calculate power and battery capacity required to accelerate
V_inf_0 = 0.2; %m/s
V_inf_final = V_inf; %m/s
i = 1;
for V_inf = V_inf_0:0.2:V_inf_final
    % Calculate velocities at the rotor based on inclination angle (beta) of rotor disk
    Vx = V_inf * cosd(beta);
    Vz = V_inf * sind(beta);
    Vtip = (tipMach * a)-Vx;

    % Calculate induced velocity based on weight and rotor inclination angle
%     C_D = 2;
%     A_body = 1;
%     D = 0.5*C_D*A_body*rho*V_inf^2;
%     W = vehicle_mass*3.71;
%     Tx = D + vehicle_mass*accel_forward;
%     Ty = W;
%     T_total = sqrt(Tx^2 + Ty^2);
%     beta = asind(Tx/T_total);
%     T= T_total/numProp;

    beta = 20;
    T = (weight / cosd(beta));
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
    P_forward(i) = (P_1 + P_2*P_3 + P_4);
    
    P_mech_forward_accel(i) = numProp * P_forward(i);
    P_elec_forward_accel(i) = P_mech_forward_accel(i)/motor_eff;     % Convert from mechanical to electrical power
    cap_batt_forward_accel(i) = capacityBattery( P_elec_forward_accel(i), V_batt, 0.2/3600, Idraw );
    
    i = i+1;
end

cap_batt_forward_accel = sum(cap_batt_forward_accel); %A*hr
P_mech_forward_accel = max(P_mech_forward_accel);
P_elec_forward_accel = max(P_elec_forward_accel);
t_accel = (V_inf_final - V_inf_0)/accel_forward; %s
dist_accel = 0.5*(V_inf_final + V_inf_0)*t_accel; %m


% Calculate velocities at the rotor based on inclination angle (beta) of rotor disk
Vx = V_inf * cosd(beta);
Vz = V_inf * sind(beta);
Vtip = (tipMach * a)-Vx;

% Calculate induced velocity based on weight and rotor inclination angle
% C_D = 2;
% A_body = 1;
% D = 0.5*C_D*A_body*rho*V_inf^2;
% W = vehicle_mass*3.71;
% Tx = D;
% Ty = W;
% T_total = sqrt(Tx^2 + Ty^2);
% beta = asind(Tx/T_total);
% T= T_total/numProp;

beta = 2;
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
P_forward = (P_1 + P_2*P_3 + P_4);
