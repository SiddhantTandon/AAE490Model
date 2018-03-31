function [P_climb, P_climb_accel, omega_climb, Time_climb_hr, t_climb_accel_hr] = Power_Climb(weight, rho, radius, a, tipMach, Cdp, s, Vclimb ,h, A_body, Cd_body, accel_climb, numProp)
% Description:
%   Calculates the power required by each rotor for constant velocity
%   vertical climb and acceleration/deceleration to/from the climb
%   velocity.
%% Calculate Power needed to accelerate to climb speed
Vclimb_0 = 0;                           % Initial vertical velocity of vehicle [m/s]
Vclimb_final = Vclimb;                  % Final vertical velocity of vehicle [m/s]

t_accel = (Vclimb_final - Vclimb_0)/accel_climb;    % time required to accelerate from rest to Vclimb [s]
t_step = 0.1;                                       % time step for loop [s]
i = 1;
for t = 0.1:t_step:t_accel
    Vclimb = t*accel_climb;                                 % Climb speed at this time instant [m/s]
    Drag = 0.5*rho*A_body*Vclimb^2*Cd_body/numProp;         % Drag force each rotor needs to overcome [N]
    vehicle_mass = weight/3.71;                             % Mass each rotor must accelerate [kg]
    Tclimb = vehicle_mass*accel_climb + Drag + weight;      % Thrust of each rotor required to accelerate in climb [N]
    v_i_0 = sqrt(Tclimb/(2*rho*pi*radius^2));               % Induced velocity through each rotor in hover [m/s]
    v_i_climb = -Vclimb/2 + sqrt((Vclimb/2)^2 + v_i_0^2);   % Induced velocity through each rotor in climb [m/s]
    Vtip_climb = tipMach * a;                               % Velocity at the blade tip [m/s]
    omega_climb = Vtip_climb / radius;                      % Rotation rate of rotor [rad/s]
    Pp_climb = (Cdp * s * rho / 8) * (omega_climb*radius)^3 * (pi*radius^2);    % Power required to overcome drag of rotor blades [W]
    P_climb_accel(i) = (Tclimb*(Vclimb + v_i_climb) + Pp_climb);              % Total power required by each rotor to accelerate in climb [W]
    i = i + 1;
end

%% %% Calculate Power needed to decelerate from climb speed to zero velocity
i = 1;
for t = 0.1:t_step:t_accel
    Vclimb = t*accel_climb;
    Drag = 0.5*rho*A_body*Vclimb^2*Cd_body/numProp;
    vehicle_mass = weight/3.71;
    Tclimb = vehicle_mass * -1 * accel_climb + Drag + weight; % Thrust of each rotor required to accelerate in climb [N]
    v_i_0 = sqrt(Tclimb/(2*rho*pi*radius^2));
    v_i_climb = -Vclimb/2 + sqrt((Vclimb/2)^2 + v_i_0^2);
    Vtip_climb = tipMach * a;
    omega_climb = Vtip_climb / radius;
    Pp_climb = (Cdp * s * rho / 8) * (omega_climb*radius)^3 * (pi*radius^2);
    P_climb_decel(i) = (Tclimb*(Vclimb + v_i_climb) + Pp_climb);
      
    i = i + 1;
end

%% Determine max power required during acceleration or deceleration
P_climb_accel = max([max(P_climb_accel), max(P_climb_decel)]);      % max power required during accel/decel [W]
t_climb_accel = (2*(Vclimb_final - Vclimb_0)/accel_climb);          % total time required to accel and decel [s]
t_climb_accel_hr = t_climb_accel/3600;                              % total time required to accel and decel [hr]
dist_accel = 0.5*(Vclimb_final + Vclimb_0)*t_climb_accel;           % total distance covered during accel and decel [m]

%% Caluclate Power required to climb at constant velocity
Vclimb = Vclimb_final;
Time_climb = (h - dist_accel)/Vclimb;                               % time needed to climb, adjusted for distance covered during accel/decel [s]
Time_climb_hr = Time_climb/3600;                                    % time needed to climb, adjusted for distance covered during accel/decel [hr]

Tclimb = weight + 0.5*rho*A_body*Vclimb^2*Cd_body/numProp;                  % Thrust required from each rotor to climb at constant velocity [N]
v_i_0 = sqrt(Tclimb/(2*rho*pi*radius^2));                                   % Induced velocity through each rotor in hover [m/s]
v_i_climb = -Vclimb/2 + sqrt((Vclimb/2)^2 + v_i_0^2);                       % Induced velocity through each rotor in climb [m/s]
Vtip_climb = tipMach * a;
omega_climb = Vtip_climb / radius;
Pp_climb = (Cdp * s * rho / 8) * (omega_climb*radius)^3 * (pi*radius^2);    % Power required to overcome drag of rotor blades [W]
P_climb = (1.2*Tclimb*(Vclimb + v_i_climb) + Pp_climb);                     % Total power required by each rotor to climb [W]
