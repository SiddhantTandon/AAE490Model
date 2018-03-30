function [Time_climb_hr, P_climb, omega_climb, cap_batt_climb_accel_total, P_mech_climb_accel, P_elec_climb_accel, t_climb_accel] = Power_Climb(weight, rho, radius, a, tipMach, Cdp, s, Vclimb ,h, A_body, Cd_body, Idraw, V_batt, motor_eff, accel_climb, numProp)

% Calculate Power and battery capacity needed to accelerate to climb speed
Vclimb_0 = 0; % Initial vertical velocity of vehicle [m/s]
Vclimb_final = Vclimb; % Final vertical velocity of vehicle [m/s]


i = 1;
for Vclimb = Vclimb_0:0.2:Vclimb_final % 
    Drag = 0.5*rho*A_body*Vclimb^2*Cd_body;
    vehicle_mass = weight/3.71;
    Tclimb = vehicle_mass*accel_climb + Drag + weight;
    v_i_0 = sqrt(Tclimb/(2*rho*pi*radius^2));
    v_i_climb = -Vclimb/2 + sqrt((Vclimb/2)^2 + v_i_0^2);
    Vtip_climb = tipMach * a;
    omega_climb = Vtip_climb / radius;
    Pp_climb = (Cdp * s * rho / 8) * (omega_climb*radius)^3 * (pi*radius^2);
    P_climb(i) = (Tclimb*(Vclimb + v_i_climb) + Pp_climb);
    
    P_mech_climb_accel(i) = numProp * P_climb(i);
    P_elec_climb_accel(i) = P_mech_climb_accel(i)/motor_eff;     % Convert from mechanical to electrical power
    cap_batt_climb_accel(i) = capacityBattery( P_elec_climb_accel(i), V_batt, 0.2/3600, Idraw );
    
    i = i + 1;
end

i = 1;
for Vclimb = Vclimb_final:-0.2:Vclimb_0
    Drag = 0.5*rho*A_body*Vclimb^2*Cd_body;
    vehicle_mass = weight/3.71;
    Tclimb = -1*vehicle_mass*accel_climb + Drag + weight;
    v_i_0 = sqrt(Tclimb/(2*rho*pi*radius^2));
    v_i_climb = -Vclimb/2 + sqrt((Vclimb/2)^2 + v_i_0^2);
    Vtip_climb = tipMach * a;
    omega_climb = Vtip_climb / radius;
    Pp_climb = (Cdp * s * rho / 8) * (omega_climb*radius)^3 * (pi*radius^2);
    P_climb_2(i) = (Tclimb*(Vclimb + v_i_climb) + Pp_climb);
    
    P_mech_climb_accel_2(i) = numProp * P_climb_2(i);
    P_elec_climb_accel_2(i) = P_mech_climb_accel_2(i)/motor_eff;     % Convert from mechanical to electrical power
    cap_batt_climb_accel_2(i) = capacityBattery( P_elec_climb_accel_2(i), V_batt, 0.2/3600, Idraw );
    
    i = i + 1;
end

cap_batt_climb_accel_total = 4 * (sum(cap_batt_climb_accel) + sum(cap_batt_climb_accel_2)); %A*hr
P_mech_climb_accel = max([max(P_mech_climb_accel), max(P_mech_climb_accel_2)]);
P_elec_climb_accel = max([max(P_elec_climb_accel), max(P_elec_climb_accel)]);
t_climb_accel = 2*(Vclimb_final - Vclimb_0)/accel_climb; %s
dist_accel = 0.5*(Vclimb_final + Vclimb_0)*t_climb_accel; %m
% figure(1);
% plot(Vclimb_0:0.2:Vclimb_final, P_climb)
% figure(2);
% plot(Vclimb_0:0.2:Vclimb_final, cap_batt_climb_accel)
% Caluclate Power to climb
Vclimb = Vclimb_final;
Time_climb = (h - dist_accel)/Vclimb; %s
Time_climb_hr = Time_climb/3600; %hr
Tclimb = weight + 0.5*rho*A_body*Vclimb^2*Cd_body;
v_i_0 = sqrt(Tclimb/(2*rho*pi*radius^2));
v_i_climb = -Vclimb/2 + sqrt((Vclimb/2)^2 + v_i_0^2);
Vtip_climb = tipMach * a;
omega_climb = Vtip_climb / radius;
Pp_climb = (Cdp * s * rho / 8) * (omega_climb*radius)^3 * (pi*radius^2);
P_climb = (Tclimb*(Vclimb + v_i_climb) + Pp_climb);
