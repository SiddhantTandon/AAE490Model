function [Mbat, Mrotors, Mmotors,cap_batt, mass_panel, area_panel, radius_rotor, omega, P_mech_total, P_elec_total] = radiusOpt_3(solidity, tipMach, Cdp, mass_total, radius_vector, numProp, flightTime, V_batt, motor_eff,  sun_time, solar_flux, h, drone_vert_rate, v_cruise, beta_cruise, beta_accel, accel_climb, accel_forward, mass_blade)
% This is a supplementary MATLAB code used to calculate the power required
% for hover on Mars to size the required battery at various rotor radii.
% Maximizing the remaining mass budget, we can find an optimal radius,
% along with the corresponding battery mass and motor rpm.
% (http://www.mae.cornell.edu/mae/news/loader.cfm?csModule=security/getfile&amp;pageid=282149)

flightTime_hr = flightTime/60;    % Convert flight time from [min] to [hr]
powerFactor = 1;                % Increase power by a factor to account for forward motion  


% Design Variables
    % number_blades = 4;
    
    %rho_blade = 1600; % density of blade material (carbon fiber for now) [kg/m^3]
    rho_cruise = rhoMars(h);   % air density on mars [kg/m^3]
    rho_vert = rhoMars(h/2);    % use average air density for climb/descent
    %t_blade = 0.02;     % average thickness of rotor blades [m]
%     Mrotors = 16.64;%20.332;%14.418;
    Mrotors = 8;        % [kg]
    %fig_merit = 0.75;   % Figure of Merit (= P_ideal/P_hover)
    k = 1.2;            % Aerodynamic correction factor
    Idraw = 349.625;   % Current required by the motors [A]
    A_body = 1; %m^2
    Cd_body = 2;
% Constants
    a = 240;     % speed of sound, m/s
    g_m = 3.71;  % Gravity on mars, m/s^2
    weight =  mass_total * g_m /numProp;  % [N] weight that each rotor must support in hover 
    
for j=1:length(radius_vector)    % Iterate through many rotor radii 
    
    %chord = solidity*pi*radius_vector(j)/number_blades;
    %Mrotors(j) = 0.8*(0.04*chord)*chord*rho_blade*radius_vector(j)*number_blades*numProp;
    %Mrotors(j) = mass_blade * number_blades * numProp;
    
    
    % Calculate Power to Hover
    [fig_merit(j), P_hover, omega(j)] = Power_Hover(weight, rho_cruise, radius_vector(j), k, a, tipMach, Cdp, solidity);
    
    % Caluclate Power to Climb
    [Time_climb_hr, P_climb, omega_climb(j), cap_batt_climb_accel, P_mech_climb_accel, P_elec_climb_accel] = Power_Climb(weight, rho_vert, radius_vector(j), a, tipMach, Cdp, solidity, drone_vert_rate, h, A_body, Cd_body, Idraw, V_batt, motor_eff, accel_climb, numProp);

    % Calculate Power to Descend
    [Time_descend_hr, P_descend, omega_descend(j)] = Power_Descend(weight, rho_vert, radius_vector(j), a, tipMach, Cdp, solidity, drone_vert_rate, h, A_body, Cd_body);

    % Calculate Power for Forward Flight
    [P_forward, cap_batt_forward_accel, P_mech_forward_accel, P_elec_forward_accel] = Power_Forward_Flight(weight, rho_cruise, radius_vector(j), k, a, tipMach, Cdp, solidity, beta_cruise, beta_accel, v_cruise, accel_forward, Idraw, V_batt, motor_eff, numProp);
    
    % Total Hover Power
    P_mech_total_hover(j) = numProp * powerFactor * P_hover;     % multiply power to account for multiple propellers and for forward flight assuming 50% used for hover, 70% used for forward flight
    P_mech_one_motor_hover(j) = P_mech_total_hover(j)/numProp;
    P_elec_total_hover(j) = P_mech_total_hover(j)/motor_eff;     % Convert from mechanical to electrical power
    P_elec_one_motor_hover(j) = P_mech_one_motor_hover(j)/motor_eff;     % Convert from mechanical to electrical power
    
    % Total Climb Power
    P_mech_total_climb(j) = numProp * P_climb;
    P_mech_one_motor_climb(j) = P_mech_total_climb(j)/numProp;
    P_elec_total_climb(j) = P_mech_total_climb(j)/motor_eff;     % Convert from mechanical to electrical power
    P_elec_one_motor_climb(j) = P_mech_one_motor_climb(j)/motor_eff;     % Convert from mechanical to electrical power
    
    P_elec_one_motor_climb_accel(j) = P_elec_climb_accel/numProp;
    
    % Total Descent Power
    P_mech_total_descend(j) =  numProp * P_descend;
    P_mech_one_motor_descend(j) = P_mech_total_descend(j)/numProp;
    P_elec_total_descend(j) = P_mech_total_descend(j)/motor_eff;     % Convert from mechanical to electrical power
    P_elec_one_motor_descend(j) = P_mech_one_motor_descend(j)/motor_eff;     % Convert from mechanical to electrical power
    
    % Total Forward Flight Power
    P_mech_total_forward(j) =  numProp * P_forward;
    P_mech_one_motor_forward(j) = P_mech_total_forward(j)/numProp;
    P_elec_total_forward(j) = P_mech_total_forward(j)/motor_eff;     % Convert from mechanical to electrical power
    P_elec_one_motor_forward(j) = P_mech_one_motor_forward(j)/motor_eff;     % Convert from mechanical to electrical power
    
    P_elec_one_motor_forward_accel(j) = P_elec_forward_accel/numProp;
    
    torque(j) = P_mech_total_climb(j)/omega_climb(j);
    
%     P_elec_total(j) = P_mech_total(j)/motor_eff;     % Convert from mechanical to electrical power
%     P_elec_one_motor(j) = P_mech_one_motor(j)/motor_eff;     % Convert from mechanical to electrical power
    P_mech_total(j) = max([P_mech_total_hover(j), P_mech_total_climb(j), P_mech_total_descend(j), P_mech_total_forward(j), P_mech_climb_accel, P_mech_forward_accel]);
    P_elec_total(j) = P_mech_total(j)/motor_eff;
    
%     cap_batt_hover(j) = capacityBattery( P_elec_total_hover(j), V_batt, flightTime_hr, Idraw );     % Estimated battery capacity [A*hr]
    cap_batt_climb(j) = capacityBattery( P_elec_total_climb(j), V_batt, Time_climb_hr, Idraw );     % Estimated battery capacity [A*hr]
    cap_batt_descend(j) = capacityBattery( P_elec_total_descend(j), V_batt, Time_descend_hr, Idraw);     % Estimated battery capacity [A*hr]
    cap_batt_forward(j) = capacityBattery( P_elec_total_forward(j), V_batt, flightTime_hr, Idraw); 
    cap_batt(j) = (cap_batt_forward(j) + cap_batt_climb(j) + cap_batt_descend(j) + cap_batt_climb_accel + cap_batt_forward_accel);
    
    Mbat(j) = 1.2 *massBattery( cap_batt(j), V_batt );                          % Mass of the battery required
%     Mrotors(j) = numProp * rho_blade * t_blade * solidity * pi * radius_vector(j)^2;       % Mass of rotor blades
    
    Power_consumption_array = 1.2*[P_elec_one_motor_hover(j), P_elec_one_motor_climb(j), P_elec_one_motor_descend(j), P_elec_one_motor_forward(j), P_elec_one_motor_climb_accel(j)];
    Max_power_elec(j) = max(Power_consumption_array);
    Mmotors(j) = (Max_power_elec(j)*7e-5 + 0.2135) * numProp * 2;                     % Rough correlation for motor mass 
                                                              % ^^^  Multiplied by 2 to achieve a more accurate estimation 
    % Find solar panel specs   
    energy_batt(j) = cap_batt(j) * V_batt;          % [W*hr]          
    P_panel(j) = energy_batt(j) / (sun_time);                  % [W]
    [ area_panel(j), mass_panel(j) ] = sizeSolarPanel(P_panel(j), solar_flux);  % [kg]

    Mrem(j) = mass_total - Mbat(j) - Mrotors(j) - Mmotors(j) - mass_panel(j);        % Remaining mass for structure/payload
    
    if Mrem(j)>0
        n(j) = Mrem(j)/mass_total;
    else
        n(j) = 0;
    end
end

% figure(1);
% plot(radius_vector,n);
% xlabel('Radius (m)'); ylabel('Mass Remaining Ratio');

[~,i]=max(n);
fig_merit = fig_merit(i);
fprintf('Fig Merit: %.2f\n',fig_merit);
Mbat = Mbat(i);
Mrotors = Mrotors(i);
Mmotors = Mmotors(i);
mass_panel = mass_panel(i);
area_panel = area_panel(i);
cap_batt = cap_batt(i);
radius_rotor = radius_vector(i);
torque = torque(i);
omega = omega(i);
P_mech_total = P_mech_total(i);
P_elec_total = P_elec_total(i);
energy_batt = energy_batt(i);
end
