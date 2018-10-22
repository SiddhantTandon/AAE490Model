function [P_hover, P_climb, P_des, rpm_hover, rpm_climb, rpm_des, Q_hover, Q_climb, Q_des] = blade_analysis( chords, alphas, phis, airfoil, r, A, tip_M, a, rho, mu, weight, climb_v, des_v, z )

%% Blade Characteristics
A_disk = pi*r^2;
sections = size(chords,2);
r_min = .1*r;

solidity = A*4/A_disk;
rpm = tip_M*a/(2*pi*r)*60;

%% Matrices
cls_climb = zeros(1,sections); % coefficient of lift
cds_climb = zeros(1,sections); % coefficient of drag
cms_climb = zeros(1,sections); % coefficient of moment
Ls_climb = zeros(1,sections);  % Lift
Ds_climb = zeros(1,sections);  % Drag
Ts_climb = zeros(1,sections);  % Thrust
Qs_climb = zeros(1,sections);  % Torque
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

n = 1;
for M = tip_M
    [ cls, cds, cms, Ls, Ds, Ts, Qs, Ms_pitch, Res, vi ] = InducedVelocity( sections, phis, alphas, chords, airfoil, r, r_min, M, a, rho, mu, A_disk );
    
    % Solve for actual characteristics using induced velocity in hover
    % iterate through sections
    count = 1;
    while count <= sections
        
        %% Hover
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
        Qs(count) = (Ls(count)*sind(phi) + Ds(count)*cosd(phi))*r_sec*r;
        Ms_pitch(count) = cms(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
        
        %% Climb
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
        Qs_climb(count) = (Ls_climb(count)*sind(phi) + Ds_climb(count)*cosd(phi))*r_sec*r;
        Ms_pitch_climb(count) = cms_climb(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
        
        %% Descend
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
        Qs_des(count) = (Ls_des(count)*sind(phi) + Ds_des(count)*cosd(phi))*r_sec*r;
        Ms_pitch_des(count) = cms_des(count)*.5*rho*v^2*chords(count)^2*(r-r_min)/sections;
        
        count = count + 1;
    end
    % Converting individual sections into totals for single blade

    % Hover
    cd = mean(cds); % coefficient of drag of entire blade
    T = sum(Ts); % total thrust generated
    Q_hover(z) = sum(Qs); % total torque
    % Calculating power of 1 rotor
    P_i = (4*T)*vi; % power required to generate induced velocity
    P_p = cd*solidity*rho*(M*a)^3*A_disk/8; % power required to overcome drag
    P = 1.2*P_i + P_p; % Total power to hover

    rpm_P(z,n) = P; % Power at each rpm
    rpm_T(z,n) = T; % Thrust at each rpm

    % Climb
    cd_climb = mean(cds_climb);
    T_climb = sum(Ts_climb); % total thrust generated
    Q_climb(z) = sum(Qs_climb); % total torque
    % Calculating power of 1 rotor
    P_i_climb = (4*T)*vi_climb; % power required to generate induced velocity
    P_p_climb = cd*solidity*rho*(M*a)^3*A_disk/8; % power required to overcome drag
    P_climb = 1.2*P_i_climb + P_p_climb; % Total power to hover        

    rpm_P_climb(z,n) = P_climb; % Power at each rpm
    rpm_T_climb(z,n) = T_climb; % Thrust at each rpm

    % Descent
    cd_des = mean(cds_des);
    T_des = sum(Ts_des); % total thrust generated
    Q_des(z) = sum(Qs_des); % total torque
    % Calculating power of 1 rotor
    P_i_des = (4*T)*vi_des; % power required to generate induced velocity
    P_p_des = cd*solidity*rho*(M*a)^3*A_disk/8; % power required to overcome drag
    P_des = 1.2*P_i_des + P_p_des; % Total power to hover        

    rpm_P_des(z,n) = P_des; % Power at each rpm
    rpm_T_des(z,n) = T_des; % Thrust at each rpm

    n = n+1;
end

% Find the power required to hover for each blade
dif = 10000;
count = 1;
for power = rpm_P(z,:)
    if abs(rpm_T(z,count)*16 - weight(1)) < dif
        dif = abs(rpm_T(z,count)*16 - weight(1));
        P_hover(z) = power;
        rpm_hover(z) = rpm(count);
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
        rpm_climb(z) = rpm(count);
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
        rpm_des(z) = rpm(count);
        if dif < 5
            break
        end
    end
    count = count+1;
end

end

