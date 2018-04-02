function [P_descend, Time_descend_hr, t_descend_accel_hr] = Power_Descend(weight, rho, radius, a, tipMach, Cdp, s, Vdescend, h, A_body, Cd_body, numProp, accel_descend)

% Calculate Power to Descend
    Vdescend_final = Vdescend;
    Vdescend_0 = 0;
    t_descend_accel = (2*(Vdescend_final - Vdescend_0 )/accel_descend);          % total time required to accel and decel [s]
    t_descend_accel_hr = t_descend_accel/3600;                              % total time required to accel and decel [hr]
    dist_accel = 0.5*(Vdescend_final + Vdescend_0 )*t_descend_accel;           % total distance covered during accel and decel [m]

    Time_descend = (h - dist_accel)/Vdescend; 
    Time_descend_hr = Time_descend/3600;

    Tdescend = weight - 0.5*rho*A_body*Vdescend^2*Cd_body/numProp; % subtract drag from weight to get descent thrust req
    v_i_0 = sqrt(Tdescend/(2*rho*pi*radius^2));
    Vdescend = 2.01*v_i_0; %m/s
    
    v_i_descend = Vdescend/2 + sqrt((Vdescend/2)^2 - v_i_0^2);
    Vtip_descend = tipMach * a;
    omega_descend = Vtip_descend / radius;
    Pp_descend = (Cdp * s * rho / 8) * (omega_descend*radius)^3 * (pi*radius^2);
    P_descend = 1.2*Tdescend*(Vdescend - v_i_descend) + Pp_descend;
