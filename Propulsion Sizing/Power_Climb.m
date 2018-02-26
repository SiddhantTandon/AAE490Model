function [Time_climb_hr, P_climb, omega_climb] = Power_Climb(weight, rho, radius, a, tipMach, Cdp, s, Vclimb ,h, A_body, Cd_body)

% Caluclate Power to climb
    Time_climb = h/Vclimb; %s
    Time_climb_hr = Time_climb/3600; %hr
    Tclimb = weight + 0.5*rho*A_body*Vclimb^2*Cd_body;
    v_i_climb = -Vclimb/2 + sqrt((Vclimb/2)^2 + Tclimb);
    Vtip_climb = tipMach * a;
    omega_climb = Vtip_climb / radius;
    Pp_climb = (Cdp * s * rho / 8) * (omega_climb*radius)^3 * (pi*radius^2);
    P_climb = Tclimb*(Vclimb + v_i_climb) + Pp_climb;