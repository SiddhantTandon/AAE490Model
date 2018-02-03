clear
clc

%% Initial Conditions

a = 240; % speed of sound [m/s]

% Coefficients from cl = f(Re) plot
[cl_v_Re_coefs, maxRe]= BestFit_cl_v_Re();
polyDeg = length(cl_v_Re_coefs) - 1;

% Inputs
num_blades = 4;
num_prop = 4;
rho = 0.013; % density [kg/m^3]
visc = .00001422; % kinematic viscosity [kg/(m*s)]
T = 35; % total required thrust [N]


%% Test Single Rotor Radius
%{
R = 0.5;
c = 0.3 * R; % chord length [m]

%% Zeros of Function
omegaMin = 0;
omegaMax = 1000;
omegas = linspace(omegaMin, omegaMax, 10000);

% Determine coefficients for f(omega) fn. from coefficients of f(Re) fn.
power = polyDeg; % inialize index off which coefficient powers are based
f_omega_coefs = cl_v_Re_coefs; % makes vector of new coefficients correct size (values are replaced)
for n = 1 : length(cl_v_Re_coefs)
    f_omega_coefs(n) = cl_v_Re_coefs(n) * rho^(power+1) * c^(power+1) * R^(power+3)...
                       * num_blades * num_prop / (2 * (power + 3) * visc^(power));
    power = power - 1; % decrements index for determining powers assigned to variables
end


% Generate f(omega) values
f_omega = zeros(1, length(omegas));
power = polyDeg; % inialize index off which omega powers are based
for n = 1 : length(f_omega_coefs)
    f_omega = f_omega + f_omega_coefs(n) * omegas .^ (power+2);
    power = power - 1; % decrement power index
end


% plot f(omega) (Thrust vs. spin rate)
figure(1);
plot(omegas, f_omega);
hold on
plot([omegaMin, omegaMax], [T, T])
hold off
title('Thurst vs. spin rate');
xlabel('\omega [rad/s]')
ylabel('f(\omega) = T [N]')
legend('Thrust Provided', 'Thrust Required')


f_omega_root_all = roots([f_omega_coefs, 0, -T]);


% Save first positive real root (the valid one)
m = 1;
for n = 1 : length(f_omega_root_all)
    if isreal(f_omega_root_all(n)) && (f_omega_root_all(n) > 0)
        f_omega_root_pos_real(m) = f_omega_root_all(n);
        m = m + 1;
    end
end
omega = f_omega_root_pos_real(1);


% Plot Reynolds number across blade
radii = linspace(0, R, 1000);
Re = omega * radii * rho * c / visc;
figure(2);
plot(radii, Re);
hold on
plot([0, R], [0, 0]);
plot([0, R], [maxRe, maxRe]);
hold off
title('Re vs. radius');
xlabel('radius [m]')
ylabel('Re')
legend('Re(r)', 'Lower Re bound', 'Upper Re bound')
%}


%% Test at Different Rotor Radii

r0 = 0.2;
r1 = 5;

radii = linspace(r0, r1, 1000);
omega = zeros(1, length(radii));
Re = zeros(1, length(radii));
tip_vel = zeros(1, length(radii));
k = 1;
for R = radii
    
    c = 0.3 * R; % chord length [m]

    % Determine coefficients for f(omega) fn. from coefficients of f(Re) fn.
    power = polyDeg; % inialize index off which coefficient powers are based
    f_omega_coefs = cl_v_Re_coefs; % makes vector of new coefficients correct size (values are replaced)
    for n = 1 : length(cl_v_Re_coefs)
        f_omega_coefs(n) = cl_v_Re_coefs(n) * rho^(power+1) * c^(power+1) * R^(power+3)...
                           * num_blades * num_prop / (2 * (power + 3) * visc^(power));
        power = power - 1; % decrements index for determining powers assigned to variables
    end

    f_omega_root_all = roots([f_omega_coefs, 0, -T]);

    % Save first positive real root (the valid one)
    m = 1;
    for n = 1 : length(f_omega_root_all)
        if isreal(f_omega_root_all(n)) && (f_omega_root_all(n) > 0)
            f_omega_root_pos_real(m) = f_omega_root_all(n);
            m = m + 1;
        end
    end
    omega(k) = f_omega_root_pos_real(1);

    Re(k) = omega(k) * R * rho * c / visc;
    tip_vel(k) = omega(k) * R;
    k = k + 1;
end

%% Output Results
omega_rpm = omega * 60 / (2*pi);

figure(3);
plot(radii, omega_rpm);
hold on
omega_rpm_limit = 1157.1 * (radii .^ (-0.793));
plot(radii, omega_rpm_limit);
plot(radii, 1.5 * omega_rpm_limit);
hold off
title({'Spin Rate vs. Max Propeller Radius', '(based on Earth helicopter rotor rpms)'});
xlabel('Propeller Radius [m]')
ylabel('Rotation Rate [rpm]')
legend('Rqrd rpm', 'rpm bound', '1.5 * rpm bound');

figure(4);
plot(radii, Re);
hold on
plot([r0, r1], [maxRe, maxRe]);
axis([0, 1.1*r1, 0, 1.1*maxRe]);
hold off
title('Max Reynolds Number vs. Max Propeller Radius');
xlabel('Propeller Radius [m]')
ylabel('Reynolds Number')
legend('peak Re', 'Re bound');


figure(5);
plot(radii, tip_vel);
hold on
plot([r0, r1], [a, a]);
plot([r0, r1], [0.7*a, 0.7*a]);
hold off
title('Tip Speed vs. Max Propeller Radius');
xlabel('Propeller Radius [m]')
ylabel('Tip Speed [m/s]')
legend('Rqrd Tip Speed for Flight', 'Mach 1', 'Mach 0.7');