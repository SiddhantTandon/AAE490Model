clc;clear;

% Import data
load('success_blades.mat');
airfoil = dlmread('NACA 4404 Data.txt')';

% Constants
rho = 0.02; % kg/m^3
a = 240; % m/s
mu = 1.422 * 10^-5; % kg/m/s
g = 3.71; % m/s^2
M_crit = 0.8;

% Input data
vehicle_mass = 60; % kg
pitches = -5:5;
tip_Mach_nums = 0.65:0.05:M_crit;
beta = 20; % flight inclination (deg)

% Instance storage
analyzed_blades = [];

%% Primary analysis loop
for blade = 366:length(success_blades)
    fprintf('Analyzing blade %d...\n', blade);
    blade_data = success_blades(blade);
    c = blade_data.chords;
    blade_number = blade_data.blade_number;
    rotor_number = blade_data.rotor_number;
    thetas = blade_data.pitches;
    initial_thrust = blade_data.thrust;
    r = blade_data.radius;
    r_min = blade_data.min_radius;
    
    A_disk = pi * r^2;
    sections = length(c);
    
    max_accel_data = [0,0,0];
    v_max_data = [0, (pitches(end)+1), 0];
            
    for pitch = pitches
        for Mach_tip = tip_Mach_nums
            v_tip = Mach_tip * a;
            % Determine hover performance
            thrust1 = initial_thrust;
            thrust2 = -5;
            trip = 0;
            while abs(thrust1 - thrust2) > 1.5
                vi = sqrt(thrust1 * blade_number / (2*rho*A_disk)); % Hover induced velocity
                for count = 1:sections % iterate through sections
                    r_pos = r_min + (count - .5)*(r-r_min)/sections;
                    v_inf = r_pos/r*v_tip; % average velocity of section
                    phi = atan(vi/v_inf) * 180 / pi;
                    alfa = thetas(count) - phi + pitch;
                    
                    v = vi/sin(phi / 180 * pi);
                    Mach_section = v / a;
                    Re = v*c(count)*rho/mu; % average Reynolds of section
                    if Re < 500
                        Re = 1000;
                    end
                    Re = round(Re,-3);

                    % Find data set at predicted Reynolds number and AoA
                    for data_set = airfoil % iterate through data
                        if data_set(1) == (round(alfa)*2)/2 && data_set(2) == Re
                            break
                        end
                    end
                    if alfa > airfoil(1,end) % escape data loop if alfa out of range
                        trip = 1;
                        break
                    end
                    
                    % Extract data for individual section of blade
                    cls(count) = data_set(3)/sqrt(1-Mach_section^2);
                    Ls(count) = cls(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                end
                
                if trip % escape due to range failure
                    break
                end
                
                % Check thrust
                thrust2 = sum(Ls); % total thrust
                thrust1 = (thrust1 + thrust2) / 2;
            end
            
            if trip % escape due to range failure
                continue
            end
            
            % Check for extrema
            accel = thrust2 * blade_number * rotor_number / vehicle_mass;
            if accel > max_accel_data(1)
                max_accel_data = [accel, pitch, Mach_tip];
            end
            
        end
        
        % Determine cruise performance
        for v_cruise = 20:5:35
            if v_max_data(1) > v_cruise % reduce loop checking for low speed flight
                continue
            end
            v_tip = (M_crit * a) - v_cruise;
            ux = v_cruise * cosd(beta) / v_tip;
            uz = v_cruise * sind(beta) / v_tip;
            thrust_req = vehicle_mass * g / cosd(beta) / blade_number / rotor_number;
            thrust1 = vehicle_mass * g / cosd(beta) / rotor_number / blade_number;
            thrust2 = -5;
            trip = 0;
            while abs(thrust1 - thrust2) > 1.5
                % Calculate induced velocity in cruise
                lambda_i0 = sqrt(thrust1 * blade_number / (2*rho*pi*r^2)) / v_tip;
                ux_bar = ux / lambda_i0;
                uz_bar = uz / lambda_i0;
                p = [1, 2*uz_bar, (ux_bar^2 + uz_bar^2), 0, -1];
                lambda_i_bar = roots(p);
                lambda_i = lambda_i_bar * lambda_i0;
                vi = lambda_i * v_tip;
                vi = real(vi(4));
                
                for count = 1:sections % iterate through sections
                    r_pos = r_min + (count - .5)*(r-r_min)/sections;
                    v_inf = r_pos/r*v_tip; % average velocity of section
                    phi = atan(vi/v_inf) * 180 / pi;
                    alfa = thetas(count) - phi + pitch;
                    
                    v = vi/sin(phi / 180 * pi);
                    Mach_section = v / a;
                    Re = v*c(count)*rho/mu; % average Reynolds of section
                    if Re < 500
                        Re = 1000;
                    end
                    Re = round(Re,-3);

                    % Find data set at predicted Reynolds number and AoA
                    for data_set = airfoil % iterate through data
                        if data_set(1) == (round(alfa)*2)/2 && data_set(2) == Re
                            break
                        end
                    end
                    
                    if alfa > airfoil(1,end) % escape data loop if alfa out of range
                        trip = 1;
                        break
                    end
                        
                    % Extract data for individual section of blade
                    cls(count) = data_set(3)/sqrt(1-Mach_section^2);
                    Ls(count) = cls(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                end
                
                if trip % escape data loop due to range failure
                    break
                end
                % Check thrust
                thrust2 = sum(Ls); % total thrust
                thrust1 = (thrust1 + thrust2) / 2;
            end
            if trip % escape data loop due to range failure
                continue
            end
            
            % Check extrema
            if (thrust2 >= thrust_req) && ((v_cruise > v_max_data(1)) || ((v_cruise == v_max_data(1)) && (pitch < v_max_data(2))))
                v_max_data = [v_cruise, pitch, (v_tip / a)];
            end
        end
    end
    
    % Store data
    blade_data.v_max_data = v_max_data;
    blade_data.max_accel_data = max_accel_data;
    analyzed_blades = [analyzed_blades, blade_data];
end

fprintf('Done!\n');
