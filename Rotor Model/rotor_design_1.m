
% Import data file as matrix
airfoil = dlmread('cr001sm Data.txt');
airfoil = airfoil';

% Input Variables
r = .7; %m
c = [.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2,.2]; %m
v_tip = 150; %m/s
sections = 20;

% Constants
rho = .02; %kg/m^3
mu = 1.422e-5; %kg/m/s
material_rho = 500; %kg/m^3    effective density of material taking empty space into account
thickness = .1; %x/c    average thickness of airfoil

% Define Output matrices
alphas = zeros(1,sections); % angle of attack
cls = zeros(1,sections); % coefficient of lift
cds = zeros(1,sections); % coefficient of drag
cms = zeros(1,sections); % coefficient of moment
Ls = zeros(1,sections); % Lift
Ds = zeros(1,sections); % Drag
Ms_pitch = zeros(1,sections); % Pitching Moment
Ms_drag = zeros(1,sections); % Drag Moment
As = zeros(1,sections); % Area
Res = zeros(1,sections); % Reynolds number
ts = c*thickness; % thickness
masses = zeros(1,sections); % mass
inertias = zeros(1,sections); % moment of inertia

count = 1;
while count <= sections % iterate through sections
    v = (count - .5)*v_tip/sections; % average velocity of section
    Re = v*c(count)*rho/mu; % average Reynolds of section
    if rem(Re,1000) < 500 % round Reynolds to nearest 1000
        Res(count) = Re - rem(Re,1000); 
    else
        Res(count) = Re + 1000 - rem(Re,1000);
    end
    maxclcd = 0;
    y = 1;
    for x = airfoil % iterate through data
        if x(2) == Res(count) % find max cl/cd at Reynolds number
            clcd = x(3)/x(4);
            if clcd > maxclcd
                maxclcd = clcd;
                n = y;
            end
        end
        y = y + 1;
    end
    
    % Calculations for individual section of blade
    alphas(count) = airfoil(1,n);
    cls(count) = airfoil(3,n);
    cds(count) = airfoil(4,n);
    cms(count) = airfoil(6,n);
    Ls(count) = cls(count)*.5*rho*v^2*c(count)*r/sections;
    Ds(count) = cds(count)*.5*rho*v^2*c(count)*r/sections;
    Ms_pitch(count) = cms(count)*.5*rho*v^2*c(count)^2*r/sections;
    Ms_drag(count) = Ds(count)*(count-.5)/sections;
    As(count) = c(count)*r/sections;
    masses(count) = As(count)*ts(count)*material_rho;
    inertias(count) = masses(count)*(count-.5)/sections;
    count = count + 1;
end
% Converting individual sections into totals
L = sum(Ls); % total lift
D = sum(Ds); % 2D drag
A = sum(As); % total area
AR = r^2/A; % aspect ratio
Di = (sum(cls)/sections)^2/pi/AR; % minimum induced drag
D_total = D + Di; % total drag
M_pitch = sum(Ms_pitch); % pitching moment
M_drag = sum(Ms_drag); % moment from drag
mass = sum(masses); % mass of blade
inertia = sum(inertias); % moment of inertia of blade
alphas

fprintf('Specifications of Single Blade:\n')
fprintf('Lift: %.3f N\n',L)
fprintf('2D Drag: %.3f N\n',D)
fprintf('Minimum Induced Drag: %.3f N\n',Di)
fprintf('Total Drag: %.3f N\n',D_total)
fprintf('Lift/Drag: %.3f\n\n',L/D_total)
fprintf('Area: %.3f m^2\n',A)
fprintf('Mass: %.3f kg\n',mass)
fprintf('Pitching Moment: %.3f Nm\n',M_pitch)
fprintf('Drag Moment: %.3f Nm\n',M_drag)
fprintf('Moment of Inertia: %.3f Nm\n',inertia)
    
    
    
    
