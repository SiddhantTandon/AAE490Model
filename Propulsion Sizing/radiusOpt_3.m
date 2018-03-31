function [Mbat, Mrotors, Mmotors, cap_batt, mass_panel, area_panel, omega, P_mech_total, P_elec_total, fig_merit] = radiusOpt_3(solidity, tipMach, Cdp, mass_total, radius_vector, numProp, flightTime, V_batt, motor_eff,  sun_time, solar_flux, h, drone_vert_rate, v_cruise, beta_cruise, beta_accel, accel_vert, accel_forward, mass_blade)
% This is a supplementary MATLAB code used to calculate the power required
% for hover on Mars to size the required battery at various rotor radii.
% Maximizing the remaining mass budget, we can find an optimal radius,
% along with the corresponding battery mass and motor rpm.
% (http://www.mae.cornell.edu/mae/news/loader.cfm?csModule=security/getfile&amp;pageid=282149)

flightTime_hr = flightTime/60;    % Convert flight time from [min] to [hr]

% Design Variables
    %number_blades = 4;
    
    %rho_blade = 1600;              % density of blade material (carbon fiber for now) [kg/m^3]
    rho_cruise = rhoMars(h);        % air density on mars [kg/m^3]
    rho_vert = rhoMars(h/2);        % use average air density for climb/descent
    %t_blade = 0.02;                % average thickness of rotor blades [m]
    Mrotors = 8;                    % Estimated mass of all rotors
    k = 1.2;                        % Aerodynamic correction factor
    Idraw = 349.625;                % Current required by the motors [A]
    A_body = 1; %m^2                % Estimated frontal area of vehicle [m^2]
    Cd_body = 2;                    % Estimated drag coefficient of vehcile (rectangular box)

    % Constants
    a = 240;                        % speed of sound, m/s
    g_m = 3.71;                     % Gravity on mars, m/s^2
    weight =  mass_total * g_m /numProp;  % [N] weight that each rotor must support in hover 
    
    % Calculate Power to Hover
    [fig_merit, P_hover, omega] = Power_Hover(weight, rho_cruise, radius_vector, k, a, tipMach, Cdp, solidity);
    
    % Caluclate Power to Climb
    [P_climb, P_climb_accel, omega_climb, Time_climb_hr, t_climb_accel_hr] = Power_Climb(weight, rho_vert, radius_vector, a, tipMach, Cdp, solidity, drone_vert_rate, h, A_body, Cd_body, Idraw, V_batt, motor_eff, accel_vert, numProp);

    % Calculate Power to Descend
    [P_descend, Time_descend_hr, t_descend_accel_hr] = Power_Descend(weight, rho_vert, radius_vector, a, tipMach, Cdp, solidity, drone_vert_rate, h, A_body, Cd_body, numProp, accel_vert);

    % Calculate Power for Forward Flight
    [P_forward, P_forward_accel, t_forward_accel_hr] = Power_Forward_Flight(weight, rho_cruise, radius_vector, k, a, tipMach, Cdp, solidity, beta_cruise, beta_accel, v_cruise, accel_forward);
    
    % Total Hover Power
    P_mech_total_hover = numProp * P_hover;                         % multiply power to account for multiple rotors
    P_mech_one_motor_hover = P_mech_total_hover/numProp;
    P_elec_total_hover = P_mech_total_hover/motor_eff;              % Convert from mechanical to electrical power
    P_elec_one_motor_hover = P_mech_one_motor_hover/motor_eff;      % Convert from mechanical to electrical power
    
    % Total Climb Power
    P_mech_total_climb = numProp * P_climb;
    P_mech_one_motor_climb = P_mech_total_climb/numProp;
    P_elec_total_climb = P_mech_total_climb/motor_eff;              % Convert from mechanical to electrical power
    P_elec_one_motor_climb = P_mech_one_motor_climb/motor_eff;      % Convert from mechanical to electrical power
    
    % Total Climb Accel/Decel Power
    P_mech_total_climb_accel = numProp * P_climb_accel;
    P_mech_one_motor_climb_accel = P_climb_accel;
    P_elec_total_climb_accel = P_mech_total_climb_accel/motor_eff;              % Convert from mechanical to electrical power
    P_elec_one_motor_climb_accel = P_mech_one_motor_climb_accel/motor_eff;      % Convert from mechanical to electrical power
    
    % Total Descent Power
    P_mech_total_descend =  numProp * P_descend;
    P_mech_one_motor_descend = P_mech_total_descend/numProp;
    P_elec_total_descend = P_mech_total_descend/motor_eff;                      % Convert from mechanical to electrical power
    P_elec_one_motor_descend = P_mech_one_motor_descend/motor_eff;              % Convert from mechanical to electrical power
    
    % Total Descent Accel/Decel Power
    P_mech_total_descend_accel = numProp * P_hover;                             % Estimate max descent accel power as power required to hover
    P_mech_one_motor_descend_accel = P_mech_total_descend_accel/numProp;
    P_elec_total_descend_accel = P_mech_total_descend_accel/motor_eff;
    P_elec_one_motor_descend_accel = P_mech_one_motor_descend_accel/motor_eff;
    
    % Total Forward Flight Power
    P_mech_total_forward =  numProp * P_forward;
    P_mech_one_motor_forward = P_mech_total_forward/numProp;
    P_elec_total_forward = P_mech_total_forward/motor_eff;     % Convert from mechanical to electrical power
    P_elec_one_motor_forward = P_mech_one_motor_forward/motor_eff;     % Convert from mechanical to electrical power
    
    
    P_mech_total_forward_accel = numProp * P_forward_accel;
    P_mech_one_motor_forward_accel = P_mech_total_forward_accel/numProp;
    P_elec_total_forward_accel = P_mech_total_forward_accel/motor_eff; 
    P_elec_one_motor_forward_accel = P_mech_one_motor_forward_accel/motor_eff;
    
    torque = P_mech_total_climb/omega_climb;
    
%     P_elec_total(j) = P_mech_total(j)/motor_eff;     % Convert from mechanical to electrical power
%     P_elec_one_motor(j) = P_mech_one_motor(j)/motor_eff;     % Convert from mechanical to electrical power
    P_mech_total = max([P_mech_total_hover, P_mech_total_climb, P_mech_total_descend, P_mech_total_forward, P_mech_total_climb_accel, P_mech_total_descend_accel, P_mech_total_forward_accel]);
    P_elec_total = P_mech_total/motor_eff;
    
%     cap_batt_hover(j) = capacityBattery( P_elec_total_hover(j), V_batt, flightTime_hr, Idraw );     % Estimated battery capacity [A*hr]
    cap_batt_climb = capacityBattery( P_elec_total_climb, V_batt, Time_climb_hr, Idraw );     % Estimated battery capacity [A*hr]
    cap_batt_climb_accel = capacityBattery( P_elec_total_climb_accel, V_batt, t_climb_accel_hr, Idraw );
    cap_batt_descend = capacityBattery( P_elec_total_descend, V_batt, Time_descend_hr, Idraw);     % Estimated battery capacity [A*hr]
    cap_batt_descend_accel = capacityBattery( P_elec_total_descend_accel, V_batt, t_descend_accel_hr, Idraw);
    cap_batt_forward = capacityBattery( P_elec_total_forward, V_batt, flightTime_hr, Idraw); 
    cap_batt_forward_accel = capacityBattery( P_elec_total_forward_accel, V_batt, t_forward_accel_hr, Idraw); 
    cap_batt = 1.2*(cap_batt_forward + cap_batt_climb + cap_batt_climb_accel + cap_batt_descend + cap_batt_descend_accel + cap_batt_forward_accel);
    
    Mbat = massBattery(cap_batt, V_batt);                          % Mass of the battery required

    Power_consumption_array = 1.2*[P_elec_one_motor_hover, P_elec_one_motor_climb, P_elec_one_motor_climb_accel, P_elec_one_motor_descend, P_elec_one_motor_descend_accel, P_elec_one_motor_forward, P_elec_one_motor_forward_accel];
    Max_power_elec = max(Power_consumption_array);
    Mmotors = (Max_power_elec*7e-5 + 0.2135) * numProp * 2;                     % Rough correlation for motor mass 
                                                              % ^^^  Multiplied by 2 to achieve a more accurate estimation 
    % Find solar panel specs   
    energy_batt = cap_batt * V_batt;          % [W*hr]          
    P_panel = energy_batt / (sun_time);                  % [W]
    [ area_panel, mass_panel ] = sizeSolarPanel(P_panel, solar_flux);  % [kg]


% figure(1);
% plot(radius_vector,n);
% xlabel('Radius (m)'); ylabel('Mass Remaining Ratio');

% [~,i]=max(n);
% fig_merit = fig_merit(i);
% Mbat = Mbat(i);
% Mrotors = Mrotors(i);
% Mmotors = Mmotors(i);
% mass_panel = mass_panel(i);
% area_panel = area_panel(i);
% cap_batt = cap_batt(i);
% radius_rotor = radius_vector(i);
% torque = torque(i);
% omega = omega(i);
% P_mech_total = P_mech_total(i);
% P_elec_total = P_elec_total(i);
% energy_batt = energy_batt(i);
end
