function [fig_merit, P_hover, omega] = Power_Hover(thrust, rho, radius, k, a, tipMach, Cdp, s)
% Calculate induced power Pi per propeller
    Pid = thrust * sqrt( thrust / (2*rho*pi*(radius^2)) );      % Ideal hover power required 
    Pi =  k*Pid; % Corrected this equation by removing FM

    % Calculate the power necessary to overcome drag on the blades Pp
    Vtip = tipMach * a; %Velocity at the tip
    omega = Vtip / radius;
    Pp = (Cdp * s* rho / 8) * (omega*radius)^3 * (pi*radius^2);
    P_hover = Pi+Pp;
    
    % Calculate Figure of Merit
    fig_merit = Pid/P_hover;