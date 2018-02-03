function [P_total, M_total, reasonableSln] = func_propeller(T, rho, visc, rho_cf, num_prop, R, cl_v_Re_coefs, maxRe)
    
    % EVENTUALLY UPDATE TO INCLUDE REASONABLE BOUND ON MOTOR RPM
    % Boolean variables
    reasonableSln = 1; %solution falls within tip speed/rpm requirements
    
    % Constants
    prop_eff = 0.65;
    motor_eff = 0.8;
    num_blades = 2; % number of blades per propeller
    T_1_motor = T / num_prop; % thrust provided by single propeller
    c = 0.3 * R; % chord [m]
    a = 240; % speed of sound [m/s]
    t_blade = 0.01 * R; % thickness of propeller blade [m] (proportional to length)
    
    % Coefficients for cl = f(Re)
    polyDeg = length(cl_v_Re_coefs) - 1;

    % Coefficients for T = f(omega)
    power = polyDeg; % inialize index off which coefficient powers are based
    f_omega_coefs = cl_v_Re_coefs; % makes vector of new coefficients correct size (values are replaced)
    for n = 1 : length(cl_v_Re_coefs)
        f_omega_coefs(n) = cl_v_Re_coefs(n) * rho^(power+1) * c^(power+1) * R^(power+3)...
                           * num_blades * num_prop / (2 * (power + 3) * visc^(power));
        power = power - 1; % decrements index for determining powers assigned to variables
    end

    f_omega_root_all = roots([f_omega_coefs, 0, -T]);
    
    % Grabs first positive, real, nonzero root (the valid one)
    m = 1;
    for n = 1 : length(f_omega_root_all)
        if isreal(f_omega_root_all(n)) && (f_omega_root_all(n) > 0)
            f_omega_root_pos_real(m) = f_omega_root_all(n);
            m = m + 1;
        end
    end
    omega = f_omega_root_pos_real(1);
    
    Re = omega * R * rho * c / visc;
    tip_vel = omega * R;
    omega_rpm = omega * 60 / (2*pi);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %MAY NEED THESE CHECKS LATER--CHECK ACCURACY    
    if omega_rpm > (1157.1 * (R^(-0.793)))
        % ensure required rpm is reasonable
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ensure Reynolds numbers across rotors fall within bounds of cl(Re)
    if Re > maxRe
        reasonableSln = 0;
    end
    
    if tip_vel > (1.0 * a)
        reasonableSln = 0;
    end
    
    % Power
    area = pi * R^2; % total area covered by rotor in 1 rotation
    P_1_sys = ((T_1_motor^3 / (2 * area * rho)) ^ 0.5) / (motor_eff * prop_eff); %power for 1 prop
    P_total = P_1_sys * num_prop;

    % Mass
    M_1_motor = P_1_sys / 4910; % mass of 1 motor [kg]
    M_1_prop = (c * R * t_blade) * rho_cf * num_blades; % mass of 1 prop
    M_total = (M_1_prop + M_1_motor) * num_prop;

end