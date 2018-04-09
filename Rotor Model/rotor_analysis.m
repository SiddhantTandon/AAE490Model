clc;clear;

% Import data
load('high_fid_blade.mat');
airfoil = dlmread('NACA 4404 Data.txt')';
max_alfa_allow = airfoil(1,end);

% Constants
rho = 0.02; % kg/m^3
a = 240; % m/s
mu = 1.422 * 10^-5; % kg/m/s
g = 3.71; % m/s^2
M_crit = 0.8;

% Input data
vehicle_mass = 60; % kg
pitches = -5:5;
rpms = 2110:100:2810;
cruises = 20:1:35;
climbs = 1:1:10;
beta = 10; % flight inclination (deg)

% Instance storage
analyzed_blades = [];

%% Primary analysis loop
for blade = 1:length(success_blades)
    fprintf('Analyzing blade %d...\n', blade);
    blade_data = success_blades(blade);
    c = blade_data.chords;
    blade_number = blade_data.blade_number;
    rotor_number = blade_data.rotor_number;
    thetas = blade_data.pitches;
    initial_thrust = blade_data.thrust;
    radius = blade_data.radius;
    min_radius = blade_data.min_radius;
    
    A_disk = pi * radius^2;
    sections = length(c);
    
    max_accel_data = [0,0,0,0]; % Thrust, Torque, Pitch, Rpm
    max_thr_hover = [0,0,0,0]; % Thrust, Torque, Pitch, Rpm
    v_max_data = [0,0,0,0,0]; % Thrust, Torque, Pitch, Rpm, Cruise Velocity
    max_thr_cruise = zeros(length(cruises),5);
    max_thr_cruise(:,5) = cruises; % Thrust, Torque, Pitch, Rpm, Cruise Velocity
    max_thr_climb = zeros(length(climbs),5);
    max_thr_climb(:,5) = climbs; % Thrust, Torque, Pitch, Rpm, Climb Velocity
    
    %% Determine hover performance
    fprintf('    Checking hover performance...\n');
    for pitch = pitches
        for rpm = rpms
            v_tip = rpm*2*pi*radius/60;
            thrust1 = initial_thrust;
            thrust2 = -5;
            while abs(thrust1 - thrust2) > 1.5
                vi = sqrt(thrust1 * blade_number / (2*rho*A_disk)); % Hover induced velocity
                for count = 1:sections % iterate through sections
                    r_pos = min_radius + (count - .5)*(radius-min_radius)/sections;
                    v_inf = r_pos/radius*v_tip; % average velocity of section
                    phi = atan(vi/v_inf) * 180 / pi;
                    alfa = thetas(count) - phi + pitch;
                    
                    v = vi/sin(phi / 180 * pi);
                    Mach_section = v / a;
                    Re = v*c(count)*rho/mu; % average Reynolds of section
                    if Re < 500
                        Re = 1000;
                    end
                    Re = round(Re,-3);

                    if alfa > max_alfa_allow % Fill with dummy data
                        Ts(count) = 0;
                        Rs(count) = 0;
                    else
                        % Find data set at predicted Reynolds number and AoA
                        for data_set = airfoil % iterate through data
                            if data_set(1) == (round(alfa)*2)/2 && data_set(2) == Re
                                break
                            end
                        end
                        % Extract data for individual section of blade
                        cl = data_set(3)/sqrt(1-Mach_section^2);
                        cd = data_set(4)/sqrt(1-Mach_section^2);
                        cts(count) = cl*cosd(phi) - cd*sind(phi);
                        crs(count) = cd*cosd(phi) + cl*sind(phi);
                        Ts(count) = cts(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                        Rs(count) = crs(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                    end
                end
                
                % Check thrust
                thrust2 = sum(Ts); % total thrust
                thrust1 = (thrust1 + thrust2) / 2;
            end
            
            % Check for extrema
            if thrust2 > max_thr_hover(1)
                torque = blade_number * sum((min_radius + ((1:sections) - .5)*(radius-min_radius)/sections) .* Rs);
                max_thr_hover = [thrust2, torque, pitch, rpm];
            end
            
        end
    end
    
    %% Determine climb performance
    fprintf('    Checking climb performance...\n');
    climb_index = 1;
    for climb = climbs
        fprintf('        Climb rate: %.0f m/s\n', climb);
        for pitch = pitches
            for rpm = rpms
                v_tip = rpm*2*pi*radius/60;
                thrust1 = initial_thrust;
                thrust2 = -5;
                while abs(thrust1 - thrust2) > 1.5
                    vi = sqrt(thrust1 * blade_number / (2*rho*A_disk)); % Hover induced velocity
                    vi = -climb/2 + sqrt((climb/2)^2 + vi^2); % Climb induced velocity
                    for count = 1:sections % iterate through sections
                        r_pos = min_radius + (count - .5)*(radius-min_radius)/sections;
                        v_inf = r_pos/radius*v_tip; % average velocity of section
                        phi = atan(vi/v_inf) * 180 / pi;
                        alfa = thetas(count) - phi + pitch;

                        v = vi/sin(phi / 180 * pi);
                        Mach_section = v / a;
                        Re = v*c(count)*rho/mu; % average Reynolds of section
                        if Re < 500
                            Re = 1000;
                        end
                        Re = round(Re,-3);

                        if alfa > max_alfa_allow % escape data loop if alfa out of range
                            Ts(count) = 0;
                            Rs(count) = 0;
                        else
                            % Find data set at predicted Reynolds number and AoA
                            for data_set = airfoil % iterate through data
                                if data_set(1) == (round(alfa)*2)/2 && data_set(2) == Re
                                    break
                                end
                            end
                            % Extract data for individual section of blade
                            cl = data_set(3)/sqrt(1-Mach_section^2);
                            cd = data_set(4)/sqrt(1-Mach_section^2);
                            cts(count) = cl*cosd(phi) - cd*sind(phi);
                            crs(count) = cd*cosd(phi) + cl*sind(phi);
                            Ts(count) = cts(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                            Rs(count) = crs(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                        end
                    end

                    % Check thrust
                    thrust2 = sum(Ts); % total thrust
                    thrust1 = (thrust1 + thrust2) / 2;
                end

                % Check for extrema
                if thrust2 > max_thr_climb(climb_index,1)
                    torque = blade_number * sum((min_radius + ((1:sections) - .5)*(radius-min_radius)/sections) .* Rs);
                    max_thr_climb(climb_index,(1:4)) = [thrust2, torque, pitch, rpm];
                end

            end
        end
        climb_index = climb_index + 1;
    end
        
    %% Determine cruise performance
    fprintf('    Checking cruise performance...\n');
    thrust_req = vehicle_mass * g / cosd(beta) / blade_number / rotor_number;
    cruise_index = 1;
    for v_cruise = cruises
        fprintf('        Cruise speed: %.0f m/s\n', v_cruise);
        for pitch = pitches
            v_tip = (M_crit * a) - v_cruise;
            ux = v_cruise * cosd(beta) / v_tip;
            uz = v_cruise * sind(beta) / v_tip;
            thrust1 = vehicle_mass * g / cosd(beta) / rotor_number / blade_number;
            thrust2 = -5;
            while abs(thrust1 - thrust2) > 1.5
                % Calculate induced velocity in cruise
                lambda_i0 = sqrt(thrust1 * blade_number / (2*rho*pi*radius^2)) / v_tip;
                ux_bar = ux / lambda_i0;
                uz_bar = uz / lambda_i0;
                p = [1, 2*uz_bar, (ux_bar^2 + uz_bar^2), 0, -1];
                lambda_i_bar = roots(p);
                lambda_i = lambda_i_bar * lambda_i0;
                vi = lambda_i * v_tip;
                vi = real(vi(4));

                for count = 1:sections % iterate through sections
                    r_pos = min_radius + (count - .5)*(radius-min_radius)/sections;
                    v_inf = r_pos/radius*v_tip; % average velocity of section
                    phi = atan(vi/v_inf) * 180 / pi;
                    alfa = thetas(count) - phi + pitch;

                    v = vi/sin(phi / 180 * pi);
                    Mach_section = v / a;
                    Re = v*c(count)*rho/mu; % average Reynolds of section
                    if Re < 500
                        Re = 1000;
                    end
                    Re = round(Re,-3);

                    if alfa > max_alfa_allow % Fill with dummy data
                        Ts(count) = 0;
                        Rs(count) = 0;
                    else
                        % Find data set at predicted Reynolds number and AoA
                        for data_set = airfoil % iterate through data
                            if data_set(1) == (round(alfa)*2)/2 && data_set(2) == Re
                                break
                            end
                        end
                        % Extract data for individual section of blade
                        cl = data_set(3)/sqrt(1-Mach_section^2);
                        cd = data_set(4)/sqrt(1-Mach_section^2);
                        cts(count) = cl*cosd(phi) - cd*sind(phi);
                        crs(count) = cd*cosd(phi) + cl*sind(phi);
                        Ts(count) = cts(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                        Rs(count) = crs(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                    end
                end

                % Check thrust
                thrust2 = sum(Ts); % total thrust
                thrust1 = (thrust1 + thrust2) / 2;
            end
            
            torque = blade_number * sum((min_radius + ((1:sections) - .5)*(radius-min_radius)/sections) .* Rs);
            
            % Check speed extrema
            if (thrust2 > thrust_req) && (v_cruise > v_max_data(1))
                v_max_data = [v_cruise, torque, pitch, (v_tip / a)];
            end
            
            % Check thrust extrema
            if thrust2 >= max_thr_cruise(cruise_index,1)
                max_thr_cruise(cruise_index,(1:4)) = [thrust2, torque, pitch, (v_tip*60/(radius*2*pi))];
            end
        end
        cruise_index = cruise_index + 1;
    end
    
    %% Store data
    blade_data.v_max_data = v_max_data;
    blade_data.max_thr_hover = max_thr_hover;
    blade_data.max_thr_cruise = max_thr_cruise;
    blade_data.max_thr_climb = max_thr_climb;
    analyzed_blades = [analyzed_blades, blade_data];
end

fprintf('Done!\n');
