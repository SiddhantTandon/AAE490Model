function [Mbat, Mrotors, Mmotors,cap_batt, mass_panel, area_panel, radius_rotor, omega, P_mech_total, P_elec_total] = radiusOpt(solidity, tipMach, Cdp, mass_total, radius_vector, numProp, flightTime, V_batt, motor_eff,  sun_time, solar_flux, h)
% This is a supplementary MATLAB code used to calculate the power required
% for hover on Mars to size the required battery at various rotor radii.
% Maximizing the remaining mass budget, we can find an optimal radius,
% along with the corresponding battery mass and motor rpm.
% (http://www.mae.cornell.edu/mae/news/loader.cfm?csModule=security/getfile&amp;pageid=282149)

flightTime_hr = flightTime/60;    % Convert flight time from [min] to [hr]
powerFactor = 1.4;                % Increase power by a factor to account for forward motion  


% Design Variables
    rho_blade = 1600/2; % density of blade material (carbon fiber for now) [kg/m^3]
    rho = rhoMars(h);   % air density on mars [kg/m^3]
    t_blade = 0.02;     % average thickness of rotor blades [m]
    fig_merit = 0.75;   % Figure of Merit (= P_ideal/P_hover)
    k = 1.2;            % Aerodynamic correction factor

% Constants
    a = 240;     % speed of sound, m/s
    g_m = 3.71;  % Gravity on mars, m/s^2
    thrust =  mass_total * g_m /numProp;  % [N] Thrust force that each motor must generate 
    
for j=1:length(radius_vector)    % Iterate through many rotor radii 
    % Calculate induced power Pi per propeller
    Pid = thrust * sqrt( thrust / (2*rho*pi*(radius_vector(j)^2)) );      % Ideal hover power required 
    Pi =  k*Pid / fig_merit;

    % Calculate the power necessary to overcome drag on the blades Pp
    Vtip = tipMach * a; %Velocity at the tip
    omega(j) = Vtip / radius_vector(j);
    Pp = (Cdp * solidity * rho / 8) * (omega(j)*radius_vector(j))^3 * (pi*radius_vector(j)^2);
    
    P_mech_total(j) = numProp * powerFactor * (Pi + Pp);     % multiply power to account for multiple propellers and for forward flight assuming 50% used for hover, 70% used for forward flight
    P_mech_one_motor(j) = P_mech_total(j)/numProp;
    torque(j) = P_mech_total(j)/omega(j);
    
    P_elec_total(j) = P_mech_total(j)/motor_eff;     % Convert from mechanical to electrical power
    P_elec_one_motor(j) = P_mech_one_motor(j)/motor_eff;     % Convert from mechanical to electrical power
    
    cap_batt(j) = capacityBattery( P_elec_total(j), V_batt, flightTime_hr );     % Estimated battery capacity [A*hr]
    Mbat(j) = massBattery( cap_batt(j), V_batt );                          % Mass of the battery required
    Mrotors(j) = numProp * rho_blade * t_blade * solidity * pi * radius_vector(j)^2;       % Mass of rotor blades
    Mmotors(j) = (P_elec_one_motor(j)*7e-5 + 0.2135) * numProp * 2;                     % Rough correlation for motor mass 
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
% plot(R,n);
% xlabel('Radius (m)'); ylabel('Mass Remaining Ratio');

[~,i]=max(n);
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
