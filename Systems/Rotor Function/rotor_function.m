function [output] = rotor_function(airfoil blades vehicle_mass climb_v des_v)

% Import airfoil data file as matrix 'airfoil'
%airfoil = dlmread('NACA 4404 Data Analysis.txt');
%airfoil = airfoil';
% Import blade data file as matrix 'blades'
%blades = dlmread('Test Blade Design.txt');
%num_blades = round(size(blades,1)/4,0);
fidelity = 25; % number of points evaluated

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

vehicle_mass = 30;
a = 240;
rho = .02;
mu = 1.422e-5;
g = 3.71;
weight = vehicle_mass*g;
tip_M = linspace(.4,.75,fidelity);
weight = weight*ones(1,size(tip_M,2));

z = 1;
while z <= num_blades
    chords = blades(4*z-3,:); % 4 included because 4 lines of data per blade in txt file
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

    cls_climb = zeros(1,sections); % coefficient of lift
    cds_climb = zeros(1,sections); % coefficient of drag
    cms_climb = zeros(1,sections); % coefficient of moment
    Ls_climb = zeros(1,sections); % Lift
    Ds_climb = zeros(1,sections); % Drag
    Ts_climb = zeros(1,sections); % Thrust
    Qs_climb = zeros(1,sections); % Torque
    Ms_pitch_climb = zeros(1,sections); % Pitching Moment
    Res_climb = zeros(1,sections); % Reynolds number
    
    cls_des = zeros(1,sections); % coefficient of lift
    cds_des = zeros(1,sections); % coefficient of drag
    cms_des = zeros(1,sections); % coefficient of moment
    Ls_des = zeros(1,sections); % Lift
    Ds_des = zeros(1,sections); % Drag
    Ts_des = zeros(1,sections); % Thrust
    Qs_des = zeros(1,sections); % Torque
    Ms_pitch_des = zeros(1,sections); % Pitching Moment
    Res_des = zeros(1,sections); % Reynolds number


    % Flight in hover

    n = 1;
    for M = tip_M
        % solve for induced velocity using iterative process of solving for
        % thrust, which is used to solve for a new induced velocity
        vi = 20; % starting point, means nothing
        vi_x = [0 0];
        d = 1;
        while d <= 2
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
            vi_x(d) = sqrt(T*4/(2*rho*A_disk));
            d = d + 1;
        end
        vi = mean(vi_x);
        
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
                    continue
                end
            end
            Ls(count) = cls(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ds(count) = cds(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ts(count) = Ls(count)*cosd(phi) - Ds(count)*sind(phi);
            Qs(count) = (Ls(count)*cosd(phi) - Ds(count)*sind(phi))*r_sec*r;
            Ms_pitch(count) = cms(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
            
            % Climb
            
            vi_climb = vi + climb_v;
            phi = atand(vi_climb/v_inf); % pitch angle caused by induced velocity
            v = vi_climb/sind(phi); % total velocity including induced and section velocities as components of vector
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
            Res_climb(count) = Re;
            for x = airfoil
                if Re == x(2) && alpha == x(1)
                    cls_climb(count) = x(3);
                    cds_climb(count) = x(4);
                    cms_climb(count) = x(6);
                    %fprintf('Information obtained from file.\n\n');
                    continue
                end
            end
            Ls_climb(count) = cls_climb(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ds_climb(count) = cds_climb(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ts_climb(count) = Ls_climb(count)*cosd(phi) - Ds_climb(count)*sind(phi);
            Qs_climb(count) = (Ls_climb(count)*cosd(phi) - Ds_climb(count)*sind(phi))*r_sec*r;
            Ms_pitch_climb(count) = cms_climb(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
            
            % Descent
            
            vi_des = vi + des_v;
            phi = atand(vi_des/v_inf); % pitch angle caused by induced velocity
            v = vi_des/sind(phi); % total velocity including induced and section velocities as components of vector
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
            Res_des(count) = Re;
            for x = airfoil
                if Re == x(2) && alpha == x(1)
                    cls_des(count) = x(3);
                    cds_des(count) = x(4);
                    cms_des(count) = x(6);
                    %fprintf('Information obtained from file.\n\n');
                    continue
                end
            end
            Ls_des(count) = cls_des(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ds_des(count) = cds_des(count)*.5*rho*v^2*chords(count)*(r-r_min)/sections;
            Ts_des(count) = Ls_des(count)*cosd(phi) - Ds_des(count)*sind(phi);
            Qs_des(count) = (Ls_des(count)*cosd(phi) - Ds_des(count)*sind(phi))*r_sec*r;
            Ms_pitch_des(count) = cms_des(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
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
        
        % Climb
        
        cd_climb = mean(cds_climb);
        T_climb = sum(Ts_climb); % total thrust generated
        Q_climb = sum(Qs_climb); % total torque
        % Calculating power of 1 rotor
        P_i_climb = (4*T)*vi_climb; % power required to generate induced velocity
        P_p_climb = cd*solidity*rho*(M*a)^3*A_disk/8; % power required to overcome drag
        P_climb = 1.2*P_i_climb + P_p_climb; % Total power to hover        

        rpm_P_climb(z,n) = P_climb;
        rpm_T_climb(z,n) = T_climb;
        
        % Descent
        
        cd_des = mean(cds_des);
        T_des = sum(Ts_des); % total thrust generated
        Q_des = sum(Qs_des); % total torque
        % Calculating power of 1 rotor
        P_i_des = (4*T)*vi_des; % power required to generate induced velocity
        P_p_des = cd*solidity*rho*(M*a)^3*A_disk/8; % power required to overcome drag
        P_des = 1.2*P_i_des + P_p_des; % Total power to hover        

        rpm_P_des(z,n) = P_des;
        rpm_T_des(z,n) = T_des;
        
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
    
    % Find the power required to climb for each blade
    dif = 10000;
    count = 1;
    for power = rpm_P_climb(z,:)
        if abs(rpm_T_climb(z,count)*16 - weight(1)) < dif
            dif = abs(rpm_T_climb(z,count)*16 - weight(1));
            P_climb(z) = power;
            if dif < 5
                break
            end
        end
        count = count+1;
    end
    
    % Find the power required to descent for each blade
    dif = 10000;
    count = 1;
    for power = rpm_P_des(z,:)
        if abs(rpm_T_des(z,count)*16 - weight(1)) < dif
            dif = abs(rpm_T_des(z,count)*16 - weight(1));
            P_des(z) = power;
            if dif < 5
                break
            end
        end
        count = count+1;
    end
    
    z = z+1;
end

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
