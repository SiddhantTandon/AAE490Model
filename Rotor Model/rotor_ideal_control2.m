clc;clear;

% Import data
load('ana_high_fid_blade.mat');
airfoil = dlmread('NACA 4404 Data.txt')';
max_alfa_allow = airfoil(1,end);

% Constants
rho = 0.02; % kg/m^3
a = 240; % m/s
mu = 1.422 * 10^-5; % kg/m/s
g = 3.71; % m/s^2
Mach_crit = 0.8;

% Input data
vehicle_mass = 60; % kg
pitches = -5.0:0.5:5.0;
rpms = 2110:100:2810;
v_cruise = 20;
climb = 10;
beta = 10; % flight inclination (deg)

% Instance storage
rpm_matrix = rpms;
for index = 2:length(pitches)
    rpm_matrix = [rpm_matrix; rpms];
end
pitch_matrix = pitches';
for index = 2:length(rpms)
    pitch_matrix = [pitch_matrix, pitches'];
end

%% Primary analysis loop
for blade = 1:length(analyzed_blades)
    fprintf('Analyzing blade %d...\n', blade);
    blade_data = analyzed_blades(blade);
    c = blade_data.chords;
    blade_number = blade_data.blade_number;
    rotor_number = blade_data.rotor_number;
    thetas = blade_data.pitches;
    initial_thrust = blade_data.thrust;
    radius = blade_data.radius;
    min_radius = blade_data.min_radius;
    
    thr_req = vehicle_mass * g / blade_number / rotor_number;
    A_disk = pi * radius^2;
    sections = length(c);
    
    hover_thrs = [];
    climb_thrs = [];
    cruise_thrs = [];
    hover_torqs = [];
    climb_torqs = [];
    cruise_torqs = [];
    hover_cds = [];
    climb_cds = [];
    cruise_cds = [];
    
    % Loop through inputs
    pitch_index = 1;
    for pitch = pitches
        fprintf('    Checking pitch: %.1f (deg)...\n', pitch);
        rpm_index = 1;
        for rpm = rpms
            fprintf('        Checking rpm: %d...\n', rpm);
            v_tip = rpm*2*pi*radius/60;
            %% Determine hover performance
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
            
            hover_thrs(pitch_index, rpm_index) = thrust2;
            hover_torqs(pitch_index, rpm_index) = blade_number * sum((min_radius + ((1:sections) - .5)*(radius-min_radius)/sections) .* Rs);
            hover_cds(pitch_index, rpm_index) = mean(crs);
            
            
            %% Determine climb performance
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

            climb_thrs(pitch_index, rpm_index) = thrust2;
            climb_torqs(pitch_index, rpm_index) = blade_number * sum((min_radius + ((1:sections) - .5)*(radius-min_radius)/sections) .* Rs);
            climb_cds(pitch_index, rpm_index) = mean(crs);

            
            %% Determine cruise performance
            if ((v_tip + v_cruise) / a) > Mach_crit
                cruise_thrs(pitch_index, rpm_index) = 0;
                cruise_torqs(pitch_index, rpm_index) = 0;
            else
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

                cruise_thrs(pitch_index, rpm_index) = thrust2;
                cruise_torqs(pitch_index, rpm_index) = blade_number * sum((min_radius + ((1:sections) - .5)*(radius-min_radius)/sections) .* Rs);
                cruise_cds(pitch_index, rpm_index) = mean(crs);
            end
           
            rpm_index = rpm_index + 1;
        end
        pitch_index = pitch_index + 1;
    end
    
    %% Determine optimal performance
    fprintf('    Optimizing...\n');
    hover_conditions = [];
    climb_conditions = [];
    cruise_conditions = [];
    for pitch_index = 1:length(pitches)
       for rpm_index = 1:length(rpms)
           if (hover_thrs(pitch_index,rpm_index) > thr_req) && (hover_thrs(pitch_index,rpm_index) < (2*thr_req))
               hover_conditions = [hover_conditions;[pitch_index,rpm_index]];
           end
           
           if (climb_thrs(pitch_index,rpm_index) > thr_req) && (climb_thrs(pitch_index,rpm_index) < (2*thr_req))
               climb_conditions = [climb_conditions;[pitch_index,rpm_index]];
           end
           
           if (cruise_thrs(pitch_index,rpm_index) > (thr_req/cosd(beta))) && (cruise_thrs(pitch_index,rpm_index) < (2*thr_req/cosd(beta)))
               cruise_conditions = [cruise_conditions;[pitch_index,rpm_index]];
           end
       end
    end
    hover_min_torque = 10000; % Abitrarily large
    hover_min_torque_con = [0,0];
    climb_min_torque = 10000; % Abitrarily large
    climb_min_torque_con = [0,0];
    cruise_min_torque = 10000; % Abitrarily large
    cruise_min_torque_con = [0,0];
    % Search hover
    for hover_con_index = 1:size(hover_conditions,1)
        hover_con = hover_conditions(hover_con_index,:);
        if hover_torqs(hover_con(1),hover_con(2)) < hover_min_torque
            hover_min_torque = hover_torqs(hover_con(1),hover_con(2));
            hover_min_torque_con = hover_con;
        end
    end
    hover_con = hover_min_torque_con;
    % Search climb
    for climb_con_index = 1:size(climb_conditions,1)
        climb_con = climb_conditions(climb_con_index,:);
        if climb_torqs(climb_con(1),climb_con(2)) < climb_min_torque
            climb_min_torque = climb_torqs(climb_con(1),climb_con(2));
            climb_min_torque_con = climb_con;
        end
    end
    climb_con = climb_min_torque_con;
    % Search cruise
    for cruise_con_index = 1:size(cruise_conditions,1)
        cruise_con = cruise_conditions(cruise_con_index,:);
        if cruise_torqs(cruise_con(1),cruise_con(2)) < cruise_min_torque
            cruise_min_torque = cruise_torqs(cruise_con(1),cruise_con(2));
            cruise_min_torque_con = cruise_con;
        end
    end
    cruise_con = cruise_min_torque_con;
    
    %% Store data
    analyzed_blades(blade).hover_con = [hover_thrs(hover_con(1),hover_con(2)), hover_torqs(hover_con(1),hover_con(2)), hover_cds(hover_con(1),hover_con(2)), pitches(hover_con(1)), rpms(hover_con(2))];
    analyzed_blades(blade).climb_con = [climb_thrs(climb_con(1),climb_con(2)), climb_torqs(climb_con(1),climb_con(2)), climb_cds(climb_con(1),climb_con(2)), pitches(climb_con(1)), rpms(climb_con(2))];
    analyzed_blades(blade).cruise_con = [cruise_thrs(cruise_con(1),cruise_con(2)), cruise_torqs(cruise_con(1),cruise_con(2)), cruise_cds(cruise_con(1),cruise_con(2)), pitches(cruise_con(1)), rpms(cruise_con(2))];
end

fprintf('Done!\n');