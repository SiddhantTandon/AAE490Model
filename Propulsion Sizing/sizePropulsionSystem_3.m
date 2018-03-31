clear;close all;clc

% DESCRIPTION: Calculate available payload mass and total mass of a single
% drone based on a number of design parameters Also estimate masses of
% indivudial components of the propulsion system for a single drone 

% Vehicle Design Inputs
    mass_total = 60;     % Total vehicle mass including payload [kg] 

% Aerodynamic/rotor parameters
    numProp = 4;          % Total number of propeller/motor combinations (4 for quadcopter) 
    solidity = 0.32;                    % Blade Solidity
    tipMach = 0.7;                      % Tip Mach Number, chosen to be fixed value
    Cd_blade_avg = 0.077;               % Average Drag Coefficient for blade
    radius_vector = 0.65;               % Blade radius [m]
    mass_blades = 8;                    % Estimated Mass of all blades [kg]
    
% Electronics Parameters
    V_batt = 43.7377;                   % Battery voltage [V]
    V_motor = 43.2;                     % Motor Voltage [V]
    motor_eff = 0.85;                   % Efficency factor between mechancial power and electrical power (= P_mech/P_elec)
    
% Mission Profile Parameters
    A_cover = pi * 25000^2;     % Required total coverage area [m^2] 
    h_cruise = 300;             % Cruise altitude above the Martian surface [m]
    v_cruise = 30;              % Planned cruise velocity [m/s]
    sensor_fov = 57;            % Left to right field of view angle (full sweep) of the sensor [deg]
    num_drones = 6;             % Total number of drones used for surveying 
    num_days = 90;              % Number of Martian sols required to complete surveying area  
    drone_vert_rate = 10;       % [m/s]  Estimated ascent/descent rate of drone to/from cruise altitude
    accel_vert = 1;             % [m/s^2] vertical acceleration
    accel_forward = 2;          % [m/s^2] horizontal acceleration
    beta_cruise = 10;           % [deg]  Angle of tilt from horizontal of rotor disk in forward flight
    beta_accel = 15;            % [deg]  Angle of tilt from horizontal of rotor disk in forward flight

% Solar Flux Parameters 
    mission_lat = 18;           % Geographic latitude of the mission on Mars Surface [deg] (Range between -90 and 90 deg)
    solar_lon = 275;            % [deg] (NOT MARTIAN SOLS) Angular position of Mars around the Sun based on time of year (0 deg corresponds to northern vernal equinox)
    
  
%%%%%%%%%%%%%%%%%%%%%%%% CALCULATIONS %%%%%%%%%%%%%%%%%%%%%%%%

% Calculate solar flux available with the given conditions  
    [ solar_flux, sun_time ] = solarFlux(mission_lat, solar_lon);     % Average solar flux on Martian surface (assumed constant) [W/m^2] and % Hours of useful sunlight on Martian surface [hr]
    
% Find required flight time
    t_accel_decel_forward = 2*v_cruise/accel_forward; % time to accelerate and decelerate to/from cruise velocity [s]
    t_accel_decel_vert = 4*drone_vert_rate/accel_vert; % time to accelerate and decelerate to/from vertical velocity (includes climb and descent) [s]
    distance_accel_vert = 0.5 * drone_vert_rate * t_accel_decel_vert; % distance travelled during vertical acceleration/deceleration (includes climb and descent) [m]
    
    
    t_cruise = cruiseTime(A_cover, h_cruise, v_cruise, sensor_fov, num_drones, num_days, t_accel_decel_forward);     % Required Flight Time [min]
    t_flight = t_cruise + t_accel_decel_forward/60 + t_accel_decel_vert/60 + 2*((h_cruise - 0.5*distance_accel_vert)/drone_vert_rate)/60;    % Add fixed number of minutes for climb/descent [min]
    
    
% This section was adapted from Cornell's Martian RHOVER Feasibility Study (http://www.mae.cornell.edu/mae/news/loader.cfm?csModule=security/getfile&amp;pageid=282149)    
    % Return the individual rotor radius that minimizes propulsion/power system weight
    [mass_batt, mass_rotor, mass_motors, cap_batt, mass_panel, area_panel, omega, P_mech_total, P_elec_total, fig_merit] = radiusOpt_3(solidity, tipMach, Cd_blade_avg, mass_total, radius_vector, numProp, t_flight, V_batt, motor_eff, sun_time, solar_flux, h_cruise, drone_vert_rate, v_cruise, beta_cruise, beta_accel, accel_vert, accel_forward, mass_blades); 
    [voluBat, num_series, num_parallel, total_cells, R] = volBattery(V_batt, P_elec_total, t_flight);
    
    P_mech_one_motor = P_mech_total/numProp;    % Caculate mech. and elec. powers per motor [W]
    P_elec_one_motor = P_elec_total/numProp;
    
    I = P_elec_total/V_motor; %Current thru motor [Amps]
    V_battchk = I*R + V_motor; %should be equal to V_batt, adjust V_batt to equal V_battchk
    
    excess_heat = P_elec_total - P_mech_total;

    mass_avail = mass_total - mass_batt - mass_rotor - mass_motors - mass_panel;        % Available mass left over after considering propulsion/power system 
    

%%%%%%%%%%%%%%%%%%%%%%%% OUTPUT %%%%%%%%%%%%%%%%%%%%%%%
fprintf('Total mass of single drone (Input): %.1f kg\n',mass_total)
fprintf('Mass available for structure, payload, and avionics: %.2f kg\n',mass_avail)
fprintf('Percent Mass available: %.2f percent\n', 100*(mass_avail/mass_total));
fprintf('Mass of all batteries: %.2f kg\n',mass_batt)
fprintf('Mass of all motors: %.2f kg\n',mass_motors)
fprintf('Mass of all rotors: %.2f kg\n',mass_rotor)
fprintf('Mass of all solar panels: %.2f kg\n',mass_panel)
fprintf('Area of all solar panels: %.2f m^2\n',area_panel)
fprintf('Volume of Battery: %.5f m^3\n', voluBat);
fprintf('Radius of each rotor: %.3f m\n',radius_vector)
fprintf('Figure of Merit of Rotor: %.3f \n',fig_merit);
fprintf('Mechanical power required from one motor: %.0f W\n',P_mech_one_motor)
fprintf('Electrical power required by one motor: %.0f W\n',P_elec_one_motor)
fprintf('Required heat dissipation for all motors: %.0f W\n',excess_heat)
fprintf('Cruise time of each drone per day: %.2f min\n',t_cruise)
fprintf('Total flight time of each drone per day: %.2f min\n\n',t_flight)
