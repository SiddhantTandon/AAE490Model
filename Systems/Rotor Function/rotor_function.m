function [P_hover_selected, P_climb_selected, P_des_selected, rpm_hover_selected, rpm_climb_selected, rpm_des_selected, mass_selected] = rotor_function(airfoil, blades, vehicle_mass, climb_v, des_v)
%{
climb_v = 2;
des_v = -2;
vehicle_mass = 30;
% Import airfoil data file as matrix 'airfoil'
airfoil = dlmread('NACA 4404 Data Analysis.txt');
airfoil = airfoil';
% Import blade data file as matrix 'blades'
blades = dlmread('Blade Designs.txt');
num_blades = round(size(blades,1)/4,0);
%}
fidelity = 1; % number of points evaluated

% Matrices for plots
rpm_P = zeros(num_blades,fidelity);
rpm_T = zeros(num_blades,fidelity);
rpm_P_climb = zeros(num_blades,fidelity);
rpm_T_climb = zeros(num_blades,fidelity);
rpm_P_des = zeros(num_blades,fidelity);
rpm_T_des = zeros(num_blades,fidelity);
P_hover = zeros(num_blades);
P_climb = zeros(num_blades);
P_des = zeros(num_blades);
rpm_hover = zeros(num_blades);
rpm_climb = zeros(num_blades);
rpm_des = zeros(num_blades);
Q_hover = zeros(num_blades);
Q_climb = zeros(num_blades);
Q_des = zeros(num_blades);

a = 240;
rho = .02;
mu = 1.422e-5;
g = 3.71;

tip_M = linspace(.4,.75,fidelity);

weight = vehicle_mass*g;
weight = weight*ones(1,size(tip_M,2));

z = 1;
while z <= num_blades
    chords = blades(4*z-3,:); % 4 included because 4 lines of data per blade in txt file
    alphas = blades(4*z-2,:);
    phis = blades(4*z,:);

    r = blades(4*z-1,1);
    
    A = blades(4*z-1,2);
    mass = blades(4*z,3);
    
    [P_hover, P_climb, P_des, rpm_hover, rpm_climb, rpm_des, Q_hover, Q_climb, Q_des] = blade_analysis( chords, alphas, phis, airfoil, r, A, tip_M, a, rho, mu, weight, climb_v, des_v, z );
    
    z = z+1;
end

% Blade selected by minimizing hover power required
[P_hover_selected, blade_selected] = min(P_hover);
blade_selected = blade_selected(1);
% Outputs of function in terms of 1 rotor (4 blades)
P_hover_selected = P_hover_selected(1);
P_climb_selected = P_climb(blade_selected);
P_des_selected = P_des(blade_selected);
rpm_hover_selected = rpm_hover(blade_selected);
rpm_climb_selected = rpm_climb(blade_selected);
rpm_des_selected = rpm_des(blade_selected);
Q_hover_selected = 4*Q_hover(blade_selected);
Q_climb_selected = 4*Q_climb(blade_selected);
Q_des_selected = 4*Q_des(blade_selected);
mass_selected = 4*blades(4*blade_selected - 1,3);

%{
z = 1;
figure(1)
plot(rpm,weight/4,rpm,weight*1.5/4)
hold on;
while z <= num_blades
    plot(rpm,rpm_T(z,:)*4)
    z = z + 1;
end
title('Thrust of 1 Rotor vs RPM')
xlabel('RPM')
ylabel('Thrust [N]')
legend('Hover', 'FoS','Blade 1','Blade 2','Blade 3','Blade 4','location','southeast')

z = 1;
figure(2)
while z <= num_blades
    plot(rpm,rpm_P(z,:))
    hold on;
    z = z + 1;
end
title('Mechanical Power Required for 1 Rotor vs RPM')
xlabel('RPM')
ylabel('Power [W]')
legend('Blade 1','Blade 2','Blade 3','Blade 4','location','southeast')

z = 1;
figure(3)
plot(rpm,weight/4,rpm,weight*1.5/4)
hold on;
while z <= num_blades
    plot(rpm,rpm_T_climb(z,:)*4)
    z = z + 1;
end
title('Thrust of 1 Rotor vs RPM  in Climb')
xlabel('RPM')
ylabel('Thrust [N]')
legend('Hover', 'FoS','Blade 1','Blade 2','Blade 3','Blade 4','location','southeast')

z = 1;
figure(4)
while z <= num_blades
    plot(rpm,rpm_P_climb(z,:))
    hold on;
    z = z + 1;
end
title('Mechanical Power Required for 1 Rotor vs RPM in Climb')
xlabel('RPM')
ylabel('Power [W]')
legend('Blade 1','Blade 2','Blade 3','Blade 4','location','southeast')

z = 1;
figure(5)
plot(rpm,weight/4,rpm,weight*1.5/4)
hold on;
while z <= num_blades
    plot(rpm,rpm_T_des(z,:)*4)
    z = z + 1;
end
title('Thrust of 1 Rotor vs RPM  in Descent')
xlabel('RPM')
ylabel('Thrust [N]')
legend('Hover', 'FoS','Blade 1','Blade 2','Blade 3','Blade 4','location','southeast')

z = 1;
figure(6)
while z <= num_blades
    plot(rpm,rpm_P_des(z,:))
    hold on;
    z = z + 1;
end
title('Mechanical Power Required for 1 Rotor vs RPM in Descent')
xlabel('RPM')
ylabel('Power [W]')
legend('Blade 1','Blade 2','Blade 3','Blade 4','location','southeast')
%}
