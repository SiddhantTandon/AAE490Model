function [Time_descend_hr, P_descend, omega_descend] = Power_Descend(weight, rho, radius, a, tipMach, Cdp, s, Vdescend, h, A_body, Cd_body)

% Calculate Power to Descend
    Vdescend = 2; %m/s
    Time_descend = h/Vdescend;
    Time_descend_hr = Time_descend/3600;
    Tdescend = weight - 0.5*rho*A_body*Vdescend^2*Cd_body; % subtract drag from weight to get descent thrust req
    v_i_descend = Vdescend/2 - sqrt((Vdescend/2)^2 - Tdescend);
    Vtip_descend = tipMach * a;
    omega_descend = Vtip_descend / radius;
    Pp_descend = (Cdp * s * rho / 8) * (omega_descend*radius)^3 * (pi*radius^2);
    P_descend = Tdescend*(Vdescend + v_i_descend) + Pp_descend;