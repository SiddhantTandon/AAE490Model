clc
clear
% Import data file as matrix 'airfoil'
airfoil = dlmread('NACA 4404 Data.txt');
airfoil = airfoil';
 
% Input Variables
sections = 6; % number of sections the blade is divided into
vehicle_mass = 25; % kg not including rotor mass
mass_max = 85; % kg
 
% Constants
rho = .02; %kg/m^3 atmospheric density
a = 240; %m/s speed of sound
mu = 1.422*10^-5; %kg/m/s atmosphericc viscosity
material_rho = 1600; %kg/m^3    effective density of material taking empty space into account
thickness = .03; %x/c    average thickness of airfoil
g = 3.71; % m/s^2 gravitational constant
 
weight = vehicle_mass * g;
blade_number = 4;
rotor_number = 4;
thrust_per_rotor = weight/rotor_number;
thrust_per_blade = thrust_per_rotor/blade_number;
 
% Define Output matrices
base_cs = dec2base(0:sections^sections-1,sections) - '0'; % matrix of all possible combinations of integers from 0 to sections
base_cs = base_cs + 1;
base_cs = base_cs';
alphas = zeros(1,sections); % angle of attack
phis = zeros(1,sections); % pitching angle from induced velocity
cls = zeros(1,sections); % coefficient of lift
cds = zeros(1,sections); % coefficient of drag
cms = zeros(1,sections); % coefficient of moment
Ls = zeros(1,sections); % Lift
Ds = zeros(1,sections); % Drag
Ts = zeros(1,sections); % Thrust
Qs = zeros(1,sections); % Torque
Ms_pitch = zeros(1,sections); % Pitching Moment
As = zeros(1,sections); % Area
masses = zeros(1,sections); % mass
inertias = zeros(1,sections); % moment of inertia
powers = zeros(1,sections); % required power

success_blades = [];
 
z = 0;
% Iterate through blade radii
for r = 0.48 % tested radius
    r_min = 0.1 * r; % point where blade starts from center
    cs = (r/2)/sections * base_cs; % matrix of all chord combos
    A_disk = pi*r^2; % Area of disk
    vi = sqrt(thrust_per_rotor / (2*rho*A_disk)); % induced velocity
    % Iterate through tip Mach number
    for Mach = 0.75
        v_tip = Mach * a;
        % Iterate chord combinations
        for c = cs
            % eliminate combos that aren't optimal
            if c(1) > c(sections)
                continue
            end
            if c(1) > c(sections-1)
                continue
            end
            if c(2) > c(sections)
                continue
            end
            if c(1) > c(2) && c(2) < c(3)
                continue
            end
            if c(2) > c(3) && c(3) < c(4)
                continue
            end
            if c(3) > c(4) && c(4) < c(5)
                continue
            end
            %{
            if c(4) > c(5) && c(5) < c(6)
                continue
            end
            if c(5) > c(6) && c(6) < c(7)
                continue
            end
            %}
            % iterate through sections
            count = 1;
            while count <= sections
                As(count) = c(count)*(r-r_min)/sections;
                masses(count) = As(count)*thickness*c(count)*material_rho;
                r_sec = (r_min + (count - .5)*(r-r_min)/sections)/r; % percent centerpoint of section is along radius
                v_inf = r_sec*v_tip; % average velocity of section
                phi = atan(vi/v_inf); % pitch angle caused by induced velocity
                v = vi/sin(phi); % total velocity including induced and section velocities as components of vector
                Re = v*c(count)*rho/mu; % average Reynolds of section
                if Re < 500
                    Re = 1000;
                end
                Re = round(Re,-3); % round Reynolds number to 1000s place
                maxclcd = 0;
                y = 1;
                n = 1;
                for x = airfoil % iterate through airfoil data
                    if x(2) < Re+1 && x(2) > Re-1 % find max cl/cd at given Reynolds number
                        clcd = x(3)/x(4);
                        if clcd > maxclcd
                            maxclcd = clcd;
                            n = y;
                        end
                    end
                    y = y + 1;
                end

                % Calculations for individual section of blade
                alphas(count) = airfoil(1,n);
                phis(count) = phi;
                cls(count) = airfoil(3,n);
                cds(count) = airfoil(4,n);
                cms(count) = airfoil(6,n);
                Ls(count) = cls(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                Ds(count) = cds(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                Ts(count) = Ls(count)*cos(phi) - Ds(count)*sin(phi);
                Qs(count) = (Ls(count)*cos(phi) - Ds(count)*sin(phi))*r_sec*r;
                Ms_pitch(count) = cms(count)*.5*rho*v^2*c(count)^2*(r-r_min)/sections;
                count = count + 1;
            end
            % Converting individual sections into totals for single blade
            cd = mean(cds); % coefficient of drag of entire blade
            L = sum(Ls); % total lift
            D = sum(Ds); % 2D drag
            T = sum(Ts); % total thrust generated
            Q = sum(Qs); % total torque
            A = sum(As); % total area
            mass = sum(masses); % total mass
            % Calculating power of 1 rotor
            solidity = A*blade_number/A_disk; % percentage of disk filled with blade
            P_i = (thrust_per_rotor + mass*g)*vi; % power required to generate induced velocity
            P_p = cd*solidity*rho*v_tip^3*A_disk/8; % power required to overcome drag
            P = 1.2*P_i + P_p; % Total power to hover
            if T > (thrust_per_blade + mass*g)*1.5
                z = z + 1;
                M_pitch = sum(Ms_pitch); % pitching moment
                inertia = sum(inertias); % moment of inertia of blade
                % Store data
                storage.chords = c;
                storage.angles = alphas;
                storage.phi = phis*180/pi;
                storage.solidity = solidity;
                storage.CD0 = cd;
                storage.mass = mass;
                storage.area = A;
                storage.power = [P_i P_p P];
                storage.max_thrust = T*blade_number;
                storage.PT_ratio = P/T/blade_number;
                success_blades = [success_blades, storage];
            end
        end
    end
end
fprintf('Total Possible Blades: %d\n',z);
