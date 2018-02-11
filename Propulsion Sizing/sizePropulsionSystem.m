clear;close all;clc

% DESCRIPTION: Calculate available payload mass and total mass of a single
% drone based on a number of design parameters Also estimate masses of
% indivudial components of the propulsion system for a single drone 


% Vehicle Design Inputs
    mass_total =  40;     % Total vehicle mass including payload [kg] 
    numProp = 4;          % Total number of propeller/motor combinations (4 for quadcopter) 

% Aerodynamic/rotor parameters
    solidity = 0.18;                    % Blade Solidity
    num_blades = 2;  % ((This doesn't do anything yet))
    tipMach = 0.6;                      % Tip Mach Number, chosen to be fixed value
    Cd_blade_avg = 0.038;               % Average Drag Coefficient for blade
    radius_vector = linspace(0,8,1000); % potential rotor radii [m]

% Electronics Parameters
    V_batt = 40;          % Battery voltage [V]
    motor_eff = 0.85;     % Efficency factor between mechancial power and electrical power (= P_mech/P_elec)
    
% Mission Profile Parameters
    A_cover = pi * 25000^2;     % Required total coverage area [m^2] 
    h_cruise = 250;             % Cruise altitude above the Martian surface [m]
    v_cruise = 40;              % Planned cruise velocity [m/s]
    sensor_fov = 57;            % Left to right field of view angle (full sweep) of the sensor [deg]
    num_drones = 6;             % Total number of drones used for surveying 
    num_days = 90;              % Number of Martian sols required to complete surveying area  
    drone_vert_rate = 2;        % [m/s]  Estimated ascent/descent rate of drone to/from cruise altitude

% Solar Flux Parameters
    sun_time = 12.33;  % Hours of useful sunlight on Martian surface [hr]
    solar_flux = 400;  % Average solar flux on Martian surface (assumed constant) [W/m^2]      Source: http://ccar.colorado.edu/asen5050/projects/projects_2001/benoit/solar_irradiance_on_mars.htm     
  
%%%%%%%%%%%%%%%%%%%%%%%% CALCULATIONS %%%%%%%%%%%%%%%%%%%%%%%%

% Find required flight time 
    t_cruise = cruiseTime(A_cover, h_cruise, v_cruise, sensor_fov, num_drones, num_days);     % Required Flight Time [min]
    t_flight = t_cruise + 2*(h_cruise/drone_vert_rate)/60;    % Add fixed number of minutes for climb/descent
    
    
% This section was adapted from Cornell's Martian RHOVER Feasibility Study (http://www.mae.cornell.edu/mae/news/loader.cfm?csModule=security/getfile&amp;pageid=282149)    
    % Return the individual rotor radius that minimizes propulsion/power system weight
    [mass_batt, mass_rotor, mass_motors, cap_batt, mass_panel, area_panel, radius_rotor, omega, P_mech_total, P_elec_total] = radiusOpt(solidity, tipMach, Cd_blade_avg, mass_total, radius_vector, numProp, t_flight, V_batt, motor_eff, sun_time, solar_flux, h_cruise); 
    
    P_mech_one_motor = P_mech_total/numProp;    % Caculate mech. and elec. powers per motor [W]
    P_elec_one_motor = P_elec_total/numProp;
    
    excess_heat = P_elec_total - P_mech_total;

    mass_avail = mass_total - mass_batt - mass_rotor - mass_motors - mass_panel;        % Available mass left over after considering propulsion/power system 
    
 
%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Total mass of single drone (Input): %.1f kg\n',mass_total)
fprintf('Mass available for structure, payload, and avionics: %.2f kg\n',mass_avail)
fprintf('Mass of all batteries: %.2f kg\n',mass_batt)
fprintf('Mass of all motors: %.2f kg\n',mass_motors)
fprintf('Mass of all rotors: %.2f kg\n',mass_rotor)
fprintf('Mass of all solar panels: %.2f kg\n',mass_panel)
fprintf('Area of all solar panels: %.2f m^2\n',area_panel)
fprintf('Radius of each rotor: %.3f m\n',radius_rotor)
fprintf('Mechanical power required from one motor: %.0f W\n',P_mech_one_motor)
fprintf('Electrical power required by one motor: %.0f W\n',P_elec_one_motor)
fprintf('Required heat dissipation for all motors: %.0f W\n',excess_heat)
fprintf('Cruise time of each drone per day: %.2f min\n',t_cruise)
fprintf('Total flight time of each drone per day: %.2f min\n\n',t_flight)
