clc
clear
% Import data file as matrix
airfoil = dlmread('NACA 4404 Data.txt');
airfoil = airfoil';
 
% Input Variables
sections = 6;
vehicle_mass = 60; % kg
mass_max = .85; % kg
P_max = 1000; % W
 
% Constants
rho = .02; %kg/m^3
a = 240; %m/s
mu = 1.422*10^-5; %kg/m/s
material_rho = 1600; %kg/m^3    effective density of material taking empty space into account
thickness = .04; %x/c    average thickness of airfoil
g = 3.71; % m/s^2
 
weight = vehicle_mass * g;
blade_number = 4;
rotor_number = 4;
thrust_per_rotor = weight/rotor_number;
thrust_per_blade = thrust_per_rotor/blade_number;
 
% Define Output matrices
base_cs = dec2base(0:sections^sections-1,sections) - '0';
base_cs = base_cs + 1;
base_cs = base_cs';
alphas = zeros(1,sections); % angle of attack
cls = zeros(1,sections); % coefficient of lift
cds = zeros(1,sections); % coefficient of drag
cms = zeros(1,sections); % coefficient of moment
Ls = zeros(1,sections); % Lift
Ds = zeros(1,sections); % Drag
Ms_pitch = zeros(1,sections); % Pitching Moment
Ms_drag = zeros(1,sections); % Drag Moment
As = zeros(1,sections); % Area
Res = zeros(1,sections); % Reynolds number
ts = zeros(1,sections); % thickness
masses = zeros(1,sections); % mass
inertias = zeros(1,sections); % moment of inertia
powers = zeros(1,sections); % required power

success_blades = [];
 
z = 0;
for r = 0.3:0.05:0.7
    r_min = 0.1 * r;
    cs = (r/2)/sections * base_cs;
    A_disk = pi*r^2;
    vi = sqrt(thrust_per_rotor / (2*rho*A_disk));
    for Mach = 0.5:0.05:0.8
        v_tip = Mach * a;
        for c = cs
            count = 1;
            while count <= sections
                As(count) = c(count)*(r-r_min)/sections;
                ts(count) = c(count)*thickness;
                masses(count) = As(count).*ts(count)*material_rho;
                inertias(count) = masses(count)*(count-.5)/sections;
                count = count + 1;
            end
            if sum(masses) > mass_max
                continue
            end
            count = 1;
            while count <= sections % iterate through sections
                v_inf = (r_min + (count - .5)*(r-r_min)/sections)/r*v_tip; % average velocity of section
                phi = atan(vi/v_inf);
                v = vi/sin(phi);
                Re = v*c(count)*rho/mu; % average Reynolds of section
                if Re < 500
                    Re = 1000;
                end
                round(Re,-3);
                maxclcd = 0;
                y = 1;
                n = 1;
                for x = airfoil % iterate through data
                    if x(2) < Re+1 && x(2) > Re-1 % find max cl/cd at Reynolds number
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
                cls(count) = airfoil(3,n);
                cds(count) = airfoil(4,n);
                cms(count) = airfoil(6,n);
                Ls(count) = cls(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                Ds(count) = cds(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                Ms_pitch(count) = cms(count)*.5*rho*v^2*c(count)^2*(r-r_min)/sections;
                Ms_drag(count) = Ds(count)*(count-.5)/sections;
                count = count + 1;
            end
            % Converting individual sections into totals
            L = sum(Ls); % total lift
            D = sum(Ds); % 2D drag
            A = sum(As); % total area
            AR = r^2/A; % aspect ratio
            Di = (sum(cls)/sections)^2/pi/AR; % minimum induced drag
            D_total = D + Di; % total drag
            P = (L + D_total)^1.5/(2*rho*A_disk)^.5;
            if L > thrust_per_blade && P < P_max
                z = z + 1;
                solidity = A / A_disk;
                cd = mean(cds);
                M_pitch = sum(Ms_pitch); % pitching moment
                M_drag = sum(Ms_drag); % moment from drag
                inertia = sum(inertias); % moment of inertia of blade
                % Store data
                storage.chords = c;
                storage.angles = alphas;
                storage.solidity = solidity;
                storage.mass = mass;
                storage.CD0 = cd;
                storage.radius = r;
                storage.rpm = v_tip / (2*pi*r) * 60;
                success_blades = [success_blades, storage];
            end
        end
    end
end
fprintf('Total Possible Blades: %d\n',z);
