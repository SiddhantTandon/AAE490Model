function [ cls, cds, cms, Ls, Ds, Ts, Qs, Ms_pitch, Res, vi ] = InducedVelocity( sections, phis, alphas, chords, airfoil, r, r_min, M, a, rho, mu, A_disk )

cls = zeros(1,sections); % coefficient of lift
cds = zeros(1,sections); % coefficient of drag
cms = zeros(1,sections); % coefficient of moment
Ls = zeros(1,sections);  % Lift
Ds = zeros(1,sections);  % Drag
Ts = zeros(1,sections);  % Thrust
Qs = zeros(1,sections);  % Torque
Ms_pitch = zeros(1,sections); % Pitching Moment
Res = zeros(1,sections); % Reynolds number

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

end
