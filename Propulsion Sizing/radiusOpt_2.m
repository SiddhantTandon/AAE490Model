function [Mbat, Mrotors, Mmotors,cap_batt, mass_panel, area_panel, radius_rotor, omega, P_mech_total, P_elec_total] = radiusOpt_2(solidity, tipMach, Cdp, mass_total, radius_vector, numProp, flightTime, V_batt, motor_eff,  sun_time, solar_flux, h, drone_vert_rate, v_cruise, beta)
% This is a supplementary MATLAB code used to calculate the power required
% for hover on Mars to size the required battery at various rotor radii.
% Maximizing the remaining mass budget, we can find an optimal radius,
% along with the corresponding battery mass and motor rpm.
% (http://www.mae.cornell.edu/mae/news/loader.cfm?csModule=security/getfile&amp;pageid=282149)

flightTime_hr = flightTime/60;    % Convert flight time from [min] to [hr]
powerFactor = 1;                % Increase power by a factor to account for forward motion  


% Design Variables
    rho_blade = 1600; % density of blade material (carbon fiber for now) [kg/m^3]
    rho_cruise = rhoMars(h);   % air density on mars [kg/m^3]
    rho_vert = rhoMars(h/2);    % use average air density for climb/descent
    t_blade = 0.04;     % average thickness of rotor blades [m]
    fig_merit = 0.75;   % Figure of Merit (= P_ideal/P_hover)
    k = 1.2;            % Aerodynamic correction factor
    Idraw = 349.625;   % Current required by the motors [A]
    A_body = 4; %m^2
    Cd_body = 2;
% Constants
    a = 240;     % speed of sound, m/s
    g_m = 3.71;  % Gravity on mars, m/s^2
    weight =  mass_total * g_m /numProp;  % [N] weight that each rotor must support in hover 
    
for j=1:length(radius_vector)    % Iterate through many rotor radii 
    
    % Calculate Power to Hover
    [fig_merit(j), P_hover, omega(j)] = Power_Hover(weight, rho_cruise, radius_vector(j), k, a, tipMach, Cdp, solidity);
%     % Calculate induced power Pi per propeller
%     Pid = thrust * sqrt( thrust / (2*rho*pi*(radius_vector(j)^2)) );      % Ideal hover power required 
%     Pi =  k*Pid; % Corrected this equation by removing FM
% 
%     % Calculate the power necessary to overcome drag on the blades Pp
%     Vtip = tipMach * a; %Velocity at the tip
%     omega(j) = Vtip / radius_vector(j);
%     Pp = (Cdp * solidity * rho / 8) * (omega(j)*radius_vector(j))^3 * (pi*radius_vector(j)^2);
%     P_hover = Pi+Pp;
    
%     FM(j) = Pid/P_hover;
    
    
    % Caluclate Power to climb
    [Time_climb_hr, P_climb, omega_climb(j)] = Power_Climb(weight, rho_vert, radius_vector(j), a, tipMach, Cdp, solidity, drone_vert_rate, h, A_body, Cd_body);
%     Vclimb = 2; %m/s
%     Time_climb = h/Vclimb; %s
%     Time_climb_hr = Time_climb/3600; %hr
%     Cd_body = 2;
%     A_body = 0.05; %m^2
%     Tclimb = thrust + 0.5*rho*A_body*Vclimb^2*Cd_body;
%     v_i = -Vclimb/2 + sqrt((Vclimb/2)^2 + Tclimb);
%     Vtip_climb = 0.8 * a;
%     omega_climb(j) = Vtip_climb / radius_vector(j);
%     Pp_climb = (Cdp * solidity * rho / 8) * (omega_climb(j)*radius_vector(j))^3 * (pi*radius_vector(j)^2);
%     P_climb = Tclimb*(Vclimb + v_i) + Pp_climb;
    
    % Calculate Power to Descend
    [Time_descend_hr, P_descend, omega_descend(j)] = Power_Descend(weight, rho_vert, radius_vector(j), a, tipMach, Cdp, solidity, drone_vert_rate, h, A_body, Cd_body);

%     Vdescend = 2; %m/s
%     Time_descend = h/Vdescend;
%     Time_descend_hr = Time_descend/3600;
%     Tdescend = weight - 0.5*rho*A_body*Vclimb^2*Cd_body; % subtract drag from weight to get descent thrust req
%     v_i = Vdescend/2 - sqrt((Vdescend/2)^2 - Tdescend);
%     Vtip_descend = 0.6 * a;
%     omega_descend(j) = Vtip_descend / radius_vector(j);
%     Pp_descend = (Cdp * solidity * rho / 8) * (omega_descend(j)*radius_vector(j))^3 * (pi*radius_vector(j)^2);
%     P_descend = Tdescend*(Vdescend + v_i) + Pp_descend;
    
    % Calculate Power for Forward Flight
    [P_forward] = Power_Forward_Flight(weight, rho_cruise, radius_vector(j), k, a, tipMach, Cdp, solidity, beta, v_cruise);
    
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
    
    torque(j) = P_mech_total_climb(j)/omega_climb(j);
    
%     P_elec_total(j) = P_mech_total(j)/motor_eff;     % Convert from mechanical to electrical power
%     P_elec_one_motor(j) = P_mech_one_motor(j)/motor_eff;     % Convert from mechanical to electrical power
    P_mech_total(j) = max([P_mech_total_hover(j), P_mech_total_climb(j), P_mech_total_descend(j), P_mech_total_forward(j)]);
    P_elec_total(j) = max([P_elec_total_hover(j), P_elec_total_climb(j), P_elec_total_descend(j), P_elec_total_forward(j)]);
    
%     cap_batt_hover(j) = capacityBattery( P_elec_total_hover(j), V_batt, flightTime_hr, Idraw );     % Estimated battery capacity [A*hr]
    cap_batt_climb(j) = capacityBattery( P_elec_total_climb(j), V_batt, Time_climb_hr, Idraw );     % Estimated battery capacity [A*hr]
    cap_batt_descend(j) = capacityBattery( P_elec_total_descend(j), V_batt, Time_descend_hr, Idraw);     % Estimated battery capacity [A*hr]
    cap_batt_forward(j) = capacityBattery( P_elec_total_forward(j), V_batt, flightTime_hr, Idraw); 
    cap_batt(j) = cap_batt_forward(j) + cap_batt_climb(j) + cap_batt_descend(j);
    
    Mbat(j) = massBattery( cap_batt(j), V_batt );                          % Mass of the battery required
    %Mrotors(j) = numProp * rho_blade * t_blade * solidity * pi * radius_vector(j)^2;       % Mass of rotor blades
    Mrotors = 16.64;
    
    Power_consumption_array = [P_elec_one_motor_hover(j), P_elec_one_motor_climb(j), P_elec_one_motor_descend(j), P_elec_one_motor_forward(j)];
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
%fprintf('Fig Merit: %.2f\n',fig_merit);
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