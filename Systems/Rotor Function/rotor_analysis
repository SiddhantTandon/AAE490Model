clc;clear;

% Import airfoil data file as matrix 'airfoil'
airfoil = dlmread('NACA 4404 Data Analysis.txt');
airfoil = airfoil';
% Import blade data file as matrix 'blades'
blades = dlmread('Test Blade Design.txt');
num_blades = round(size(blades,1)/4,0);
fidelity = 25; % number of points evaluated

% Matrices for plots
rpm_P = zeros(num_blades,fidelity);
rpm_T = zeros(num_blades,fidelity);
rpm_P_vert = zeros(num_blades,fidelity);
rpm_T_vert = zeros(num_blades,fidelity);
P_hover = zeros(num_blades);

vehicle_mass = 30;
a = 240;
rho = .02;
mu = 1.422e-5;
g = 3.71;
weight = vehicle_mass*g;
tip_M = linspace(.5,.8,fidelity);
weight = weight*ones(1,size(tip_M,2));

z = 1;
while z <= num_blades
    chords = blades(4*z-3,:);
    alphas = blades(4*z-2,:);
    phis = blades(4*z,:);

    r = blades(4*z-1,1);
    A_disk = pi*r^2;
    sections = size(chords,2);
    r_min = .1*r;
    A = blades(4*z-1,2);
    mass = blades(4*z,3);
    solidity = A*4/A_disk;
    rpm = tip_M*a/(2*pi*r)*60;

    cls = zeros(1,sections); % coefficient of lift
    cds = zeros(1,sections); % coefficient of drag
    cms = zeros(1,sections); % coefficient of moment
    Ls = zeros(1,sections); % Lift
    Ds = zeros(1,sections); % Drag
    Ts = zeros(1,sections); % Thrust
    Qs = zeros(1,sections); % Torque
    Ms_pitch = zeros(1,sections); % Pitching Moment
    Res = zeros(1,sections); % Reynolds number

    cls_vert = zeros(1,sections); % coefficient of lift
    cds_vert = zeros(1,sections); % coefficient of drag
    cms_vert = zeros(1,sections); % coefficient of moment
    Ls_vert = zeros(1,sections); % Lift
    Ds_vert = zeros(1,sections); % Drag
    Ts_vert = zeros(1,sections); % Thrust
    Qs_vert = zeros(1,sections); % Torque
    Ms_pitch_vert = zeros(1,sections); % Pitching Moment
    Res_vert = zeros(1,sections); % Reynolds number


    % Flight in hover

    n = 1;
    for M = tip_M
        % solve for induced velocity using iterative process of solving for
        % thrust, which is used to solve for a new induced velocity
        vi = 25;
        dvi = 100;
        while dvi > .1
            % iterate through sections
            count = 1;
            while count <= sections
                r_sec = (r_min + (count - .5)*(r-r_min)/sections)/r; % percent centerpoint of section is along radius
                v_inf = r_sec*M*a; % average velocity of section
                phi = atand(vi/v_inf); % pitch angle caused by induced velocity
                v = vi/sind(phi); % total velocity including induced and section velocities as components of vector
                Re = v*chords(count)*rho/mu; % average Reynolds of section
                if Re < 500
                    Re = 1000;
                end
                Re = round(Re,-3); % round Reynolds number to 1000s place
                Res(count) = Re;
                d_alpha = phis(count) - phi;
                alpha = alphas(count) + d_alpha; % Adjust angle of attack based on changed induced velocity
                alpha = round(alpha,1);
                if alpha > 8 || alpha < 3
                    alpha = round(alpha*2,0)/2;
                end
                for x = airfoil
                    if Re == x(2) && alpha == x(1)
                        cls(count) = x(3);
                        cds(count) = x(4);
                        %fprintf('Information obtained from file.\n\n');
                        continue
                    end
                end
                Ls(count) = cls(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
                Ds(count) = cds(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
                Ts(count) = Ls(count)*cosd(phi) - Ds(count)*sind(phi);
                count = count + 1;
            end
            T = sum(Ts); % total thrust generated
            vi_new = sqrt(T*4/(2*rho*A_disk));
            dvi = vi_new - vi;
            vi = vi_new;
        end
        
        % Solve for actual characteristics using induced velocity in hover
        % iterate through sections
        count = 1;
        while count <= sections
            r_sec = (r_min + (count - .5)*(r-r_min)/sections)/r; % percent centerpoint of section is along radius
            v_inf = r_sec*M*a; % average velocity of section
            phi = atand(vi/v_inf); % pitch angle caused by induced velocity
            v = vi/sind(phi); % total velocity including induced and section velocities as components of vector
            Re = v*chords(count)*rho/mu; % average Reynolds of section
            if Re < 500
                Re = 1000;
            end
            Re = round(Re,-3); % round Reynolds number to 1000s place
            alpha = alphas(count) + phis(count) - phi; % Adjust angle of attack based on changed induced velocity
            alpha = round(alpha,1);
            if alpha > 8 || alpha < 3
                alpha = round(alpha*2,0)/2;
            end
            Res(count) = Re;
            for x = airfoil
                if Re == x(2) && alpha == x(1)
                    cls(count) = x(3);
                    cds(count) = x(4);
                    cms(count) = x(6);
                    %fprintf('Information obtained from file.\n\n');
                    continue
                end
            end
            Ls(count) = cls(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ds(count) = cds(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ts(count) = Ls(count)*cosd(phi) - Ds(count)*sind(phi);
            Qs(count) = (Ls(count)*cosd(phi) - Ds(count)*sind(phi))*r_sec*r;
            Ms_pitch(count) = cms(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
            
            % Climb/Descent
            
            vert_v = -10; % velocity in vertical direction
            vi_vert = vi + vert_v;
            phi = atand(vi_vert/v_inf); % pitch angle caused by induced velocity
            v = vi_vert/sind(phi); % total velocity including induced and section velocities as components of vector
            Re = v*chords(count)*rho/mu; % average Reynolds of section
            if Re < 500
                Re = 1000;
            end
            Re = round(Re,-3); % round Reynolds number to 1000s place
            alpha = alphas(count) + phis(count) - phi; % Adjust angle of attack based on changed induced velocity
            alpha = round(alpha,1);
            if alpha > 8 || alpha < 3
                alpha = round(alpha*2,0)/2;
            end
            Res_vert(count) = Re;
            for x = airfoil
                if Re == x(2) && alpha == x(1)
                    cls_vert(count) = x(3);
                    cds_vert(count) = x(4);
                    cms_vert(count) = x(6);
                    %fprintf('Information obtained from file.\n\n');
                    continue
                end
            end
            Ls_vert(count) = cls_vert(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ds_vert(count) = cds_vert(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ts_vert(count) = Ls_vert(count)*cosd(phi) - Ds_vert(count)*sind(phi);
            Qs_vert(count) = (Ls_vert(count)*cosd(phi) - Ds_vert(count)*sind(phi))*r_sec*r;
            Ms_pitch_vert(count) = cms_vert(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
            count = count + 1;
        end
        % Converting individual sections into totals for single blade
        
        % Hover
        
        cd = mean(cds); % coefficient of drag of entire blade
        T = sum(Ts); % total thrust generated
        Q = sum(Qs); % total torque
        % Calculating power of 1 rotor
        P_i = (4*T)*vi; % power required to generate induced velocity
        P_p = cd*solidity*rho*(M*a)^3*A_disk/8; % power required to overcome drag
        P = 1.2*P_i + P_p; % Total power to hover
        
        rpm_P(z,n) = P;
        rpm_T(z,n) = T;
        
        % Climb/Descent
        
        cd_vert = mean(cds_vert);
        T_vert = sum(Ts_vert); % total thrust generated
        Q_vert = sum(Qs_vert); % total torque
        % Calculating power of 1 rotor
        P_i_vert = (4*T)*vi; % power required to generate induced velocity
        P_p_vert = cd*solidity*rho*(M*a)^3*A_disk/8; % power required to overcome drag
        P_vert = 1.2*P_i_vert + P_p_vert; % Total power to hover        


        rpm_P_vert(z,n) = P_vert;
        rpm_T_vert(z,n) = T_vert;
        
        n = n+1;
    end
    
    % Find the power required to hover for each blade
    dif = 10000;
    count = 1;
    for power = rpm_P(z,:)
        if abs(rpm_T(z,count)*16 - weight(1)) < dif
            dif = abs(rpm_T(z,count)*16 - weight(1));
            P_hover(z) = power;
        end
        count = count+1;
    end
    z = z+1;
end

figure(1)
plot(rpm,rpm_T(1,:)*4,rpm,rpm_T(2,:)*4,rpm,weight/4,rpm,weight*1.5/4)
title('Thrust of 1 Rotor vs RPM')
xlabel('RPM')
ylabel('Thrust [N]')
legend('Blade 1','Blade 2','Hover', 'FoS','location','southeast')

figure(2)
plot(rpm,rpm_P(1,:),rpm,rpm_P(2,:))
title('Mechanical Power Required for 1 Rotor vs RPM')
xlabel('RPM')
ylabel('Power [W]')
legend('Blade 1','Blade 2','location','southeast')

figure(3)
plot(rpm,rpm_T_vert(1,:)*4,rpm,rpm_T_vert(2,:)*4,rpm,weight/4,rpm,weight*1.5/4)
title('Thrust of 1 Rotor vs RPM  in 10 m/s Climb')
xlabel('RPM')
ylabel('Thrust [N]')
legend('Blade 1','Blade 2','Hover', 'FoS','location','southeast')

figure(4)
plot(rpm,rpm_P_vert(1,:),rpm,rpm_P_vert(2,:))
title('Mechanical Power Required for 1 Rotor vs RPM in 10 m/s Climb')
xlabel('RPM')
ylabel('Power [W]')
legend('Blade 1','Blade 2','location','southeast')
