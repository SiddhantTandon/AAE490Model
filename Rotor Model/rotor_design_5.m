clc
clear
% Import data file as matrix
airfoil = dlmread('NACA 4404 Data.txt');
airfoil = airfoil';
 
% Input Variables
sections = 5;
vehicle_mass = 60; % kg
mass_max = 1; % kg
 
% Constants
rho = .02; %kg/m^3
a = 240; %m/s
mu = 1.422*10^-5; %kg/m/s
material_rho = 1600; %kg/m^3    effective density of material taking empty space into account
thickness = .02; %x/c    average thickness of airfoil
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
thetas = zeros(1,sections); % angle of attack
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

success_blades = [];
fail_check = [0,0,0];
 
z = 0;
for r = 0.7 % 0.5:0.05:0.7
    r_min = 0.1 * r;
    cs = (r/2)/sections * base_cs;
    A_disk = pi*r^2;
    vi = sqrt(thrust_per_rotor / (2*rho*A_disk));
    for Mach = 0.7 % 0.6:0.05:0.8
        v_tip = Mach * a;
        CT = thrust_per_rotor / (0.5 * rho * v_tip^2 * A_disk * sqrt(1-Mach^2));
        for c = cs
            for count = 1:sections
                As(count) = c(count)*(r-r_min)/sections;
                ts(count) = c(count)*thickness;
                masses(count) = As(count)*ts(count)*material_rho;
                inertias(count) = masses(count)*(count-.5)/sections;
                count = count + 1;
            end
            mass = sum(masses);
            if mass > mass_max
                fail_check(1) = fail_check(1) + 1;
                continue
            end
            
            A = sum(As); % total area
            solidity = blade_number * A / A_disk;
            theta_tip = 2 * CT / (solidity * 5.7) + 0.5 * sqrt(CT);
            
            success = 1;
            for count = 1:sections % iterate through sections
                r_pos = r_min + (count - .5)*(r-r_min)/sections;
                v_inf = r_pos/r*v_tip; % average velocity of section
                phi = atan(vi/v_inf);
                v = vi/sin(phi);
                Re = v*c(count)*rho/mu; % average Reynolds of section
                if Re < 500
                    Re = 1000;
                end
                Re = round(Re,-3);
                
                % Calculate ideal twisted position
                if r_pos < (0.4*r)
                    theta = theta_tip / 0.4;
                else
                    theta = theta_tip * r / r_pos;
                end
                
                % Determine angle of attack
                alfa = theta - phi;
                
                % Find conditions at ideal twist and observed Reynolds number
                index = 1;
                for x = airfoil % iterate through data
                    if x(2) == Re && x(1) == round(alfa*2)/2;
                        break
                    end
                    index = index + 1;
                end
                if index > size(airfoil,2)
                    success = 0;
                    fail_check(2) = fail_check(2) + 1;
                    break
                end

                % Extract data for individual section of blade
                thetas(count) = theta;
                alfas(count) = alfa;
                cls(count) = airfoil(3,index)/sqrt(1-Mach^2);
                cds(count) = airfoil(4,index)/sqrt(1-Mach^2);
                cms(count) = airfoil(6,index)/sqrt(1-Mach^2);
                Ls(count) = cls(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                Ds(count) = cds(count)*.5*rho*v^2*c(count)*(r-r_min)/sections;
                Ms_pitch(count) = cms(count)*.5*rho*v^2*c(count)^2*(r-r_min)/sections;
                Ms_drag(count) = Ds(count)*(count-.5)/sections;
            end
            if ~success
                continue
            end
            % Converting individual sections into totals
            L = sum(Ls); % total lift
            D = sum(Ds); % 2D drag
            AR = r^2/A; % aspect ratio
            Di = (sum(cls)/sections)^2/pi/AR; % minimum induced drag
            D_total = D + Di; % total drag
            if L > thrust_per_blade && P < P_max
                z = z + 1;
                cd = mean(cds);
                M_pitch = sum(Ms_pitch); % pitching moment
                M_drag = sum(Ms_drag); % moment from drag
                inertia = sum(inertias); % moment of inertia of blade
                % Store data
                storage.chords = c';
                storage.pitches = thetas;
                storage.solidity = solidity;
                storage.mass = mass;
                storage.CD0 = cd;
                storage.radius = r;
                storage.rpm = v_tip / (2*pi*r) * 60;
                success_blades = [success_blades, storage];
            elseif L > thrust_per_blade
                fail_check(3) = fail_check(3) + 1;
            end
        end
    end
end
fprintf('Total Possible Blades: %d\n',z);
