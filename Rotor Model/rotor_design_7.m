clc; clear;
% Import data file as matrix
airfoil = dlmread('NACA 4404 Data.txt')';
max_alfa_allow = airfoil(1,end);
 
%% Input Variables
sections = 14; % Number of blade sections to iterate (must be integer greater than 1)
chord_fid = 10; % Fidelity of chord variation (must be integer greater than 1)
var_allow = 1; % Allowable chord variation (must be integer greater than or equal to zero)
vehicle_mass = 60; % kg
tip_radii = 0.65; % tip radii to iterate
rpms = 2470; % rpms to iterate
blade_number = 4;
rotor_number = 4;
 
% Constants
rho = 0.02; % kg/m^3
a = 240; % m/s
mu = 1.422 * 10^-5; % kg/m-s
g = 3.71; % m/s^2
 
weight = vehicle_mass * g;
thrust_per_rotor = weight/rotor_number;
thrust_per_blade = thrust_per_rotor/blade_number;
 
% Define storage matrices
thetas = zeros(1,sections); % angle of attack
cls = zeros(1,sections); % coefficient of lift
cds = zeros(1,sections); % coefficient of drag
Ts = zeros(1,sections); % Lift
As = zeros(1,sections); % Area
success_blades = []; % Storage matrix for successful designs
fail_blades = []; % Storage matrix for failed designs

% Construct chord matrix
% chord_matrix(1,:) = 1:chord_fid;
% for section = 2:sections
%     new_matrix = [];
%     for blade = 1:length(chord_matrix(section-1,:))
%         Determine chord bounds
%         min_chord = (chord_matrix(section-1,blade)) - var_allow;
%         if min_chord < 1
%             min_chord = 1;
%         end
%         max_chord = (chord_matrix(section-1,blade)) + var_allow;
%         if max_chord > chord_fid
%             max_chord = chord_fid;
%         end
%         Construct new additions
%         new_add = [];
%         for chord = min_chord:max_chord
%             new_add = [new_add, [chord_matrix(:,blade); chord]];
%         end
%         new_matrix = [new_matrix, new_add];
%     end
%     chord_matrix = new_matrix;
% end
% base_cs = chord_matrix / chord_fid;
base_cs = [4,5,6,7,8,9,10,10,10,10,10,10,10,10]' / chord_fid; % Final model

%% Begin primary design loop
for radius = tip_radii % radius iteration loop
    fprintf('Checking radius r = %.2f m...\n', radius);
    min_radius = 0.1 * radius;
    cs = (radius/3) * base_cs;
    A_disk = pi*radius^2;
    for rpm = rpms % rpm iteration loop
        fprintf('    Checking rpm = %d...\n', rpm);
        v_tip = rpm*2*pi*radius/60;
        c_check = 1;
        for c = cs
            if ~rem(c_check,1000)
                fprintf('        Checking chord distribution %d...\n', c_check);
            end
            % Begin convergence loop to predict behavior
            thrust1 = thrust_per_blade;
            thrust2 = -5;
            while abs(thrust1 - thrust2) > 1.5
                vi_approx1 = sqrt(thrust1 * blade_number / (2*rho*A_disk)); % initial approximation for induced velocity
                for count = 1:sections % iterate through sections
                    As(count) = c(count)*(radius-min_radius)/sections;
                    r_pos = min_radius + (count - .5)*(radius-min_radius)/sections;
                    v_inf = r_pos/radius*v_tip; % average velocity of section
                    phi = atan(vi_approx1/v_inf) * 180 / pi;
                    v = vi_approx1/sin(phi / 180 * pi);
                    Mach_section = v / a;
                    Re = v*c(count)*rho/mu; % average Reynolds of section
                    if Re < 500
                        Re = 1000;
                    end
                    Re = round(Re,-3);

                    % Find max ct
                    maxct = 0;
                    for data_set = airfoil % iterate through data
                        if data_set(2) == Re
                            ct = data_set(3)*cosd(phi) - data_set(4)*sind(phi);
                            if ct > maxct
                                maxct = ct;
                                maxct_alfa = data_set(1);
                            end
                        end
                    end
                    % Pitch down
                    for data_set = airfoil % iterate through data
                        if maxct_alfa > (max_alfa_allow - 5.0)
                            if data_set(2) == Re && (data_set(1) == maxct_alfa - 5.0)
                                break
                            end
                        else
                            if data_set(2) == Re && (data_set(1) == maxct_alfa - 2.0)
                                break
                            end
                        end
                    end

                    % Extract data for individual section of blade
                    alfas(count) = data_set(1);
                    thetas(count) = data_set(1) + phi;
                    cl = data_set(3)/sqrt(1-Mach_section^2);
                    cd = data_set(4)/sqrt(1-Mach_section^2);
                    cts(count) = cl*cosd(phi) - cd*sind(phi);
                    crs(count) = cd*cosd(phi) + cl*sind(phi);
                    Ts(count) = cts(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                    Rs(count) = crs(count)*.5*rho*v^2*c(count)*(radius-min_radius)/sections;
                end
                
                % Check thrust
                thrust2 = sum(Ts); % total thrust
                thrust1 = (thrust1 + thrust2) / 2;
            end
            
            A = sum(As); % total area
            solidity = blade_number * A / A_disk;
            torque = blade_number * sum((min_radius + ((1:sections) - .5)*(radius-min_radius)/sections) .* Rs);
            A = sum(As); % total reference area of blades
            solidity = blade_number * A / A_disk;

            % Store data
            storage.chords = c';
            storage.thrust = thrust2;
            storage.blade_number = blade_number;
            storage.rotor_number = rotor_number;
            storage.AoA = alfas;
            storage.pitches = thetas;
            storage.solidity = solidity;
            storage.torque = torque;
            storage.CD0 = mean(crs); % Redefine for general case
            storage.radius = radius;
            storage.min_radius = min_radius;
            storage.rpm_design = rpm;
            
            if (thrust2 > thrust_per_blade) && (thrust2 < (2 * thrust_per_blade))
                success_blades = [success_blades, storage];
            else
                fail_blades = [fail_blades, storage];
            end
            c_check = c_check + 1;
        end
    end
end
fprintf('\nTotal Successful Blades: %d\n',length(success_blades));
fprintf('Total Failed Blades: %d\n',length(fail_blades));