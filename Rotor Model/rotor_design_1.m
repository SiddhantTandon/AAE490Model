
% Import data file as matrix
airfoil = dlmread('sd2030 Data.txt');
airfoil = airfoil';

% Input Variables
r = .7; %m
c = .2; %m
v_tip = 150; %m/s
sections = 100;

% Constants
rho = .02; %kg/m^3
mu = 1.422e-5; %kg/m/s

% Define Output matrices
alphas = zeros(1,sections);
cls = zeros(1,sections);
cds = zeros(1,sections);
cms = zeros(1,sections);
Ls = zeros(1,sections);
Ds = zeros(1,sections);
Ms = zeros(1,sections);
Res = zeros(1,sections);

count = 1;
while count <= sections % iterate through sections
    v = (count - .5)*v_tip/sections; % average velocity of section
    Re = v*c*rho/mu; % average Reynolds of section
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
    alphas(count) = airfoil(1,n);
    cls(count) = airfoil(3,n);
    cds(count) = airfoil(4,n);
    cms(count) = airfoil(6,n);
    Ls(count) = cls(count)*.5*rho*v^2*c*r/sections;
    Ds(count) = cds(count)*.5*rho*v^2*c*r/sections;
    Ms(count) = cms(count)*.5*rho*v^2*c^2*r/sections;
    count = count + 1;
end
L = sum(Ls);
D = sum(Ds);
AR = r/c;
Di = (sum(cls)/sections)^2/pi/AR;
D_total = D + Di;
M = sum(Ms);
alphas

fprintf('Specifications of Single Rotor:\n')
fprintf('Lift: %.3f N\n',L)
fprintf('2D Drag: %.3f N\n',D)
fprintf('Minimum Induced Drag: %.3f N\n',Di)
fprintf('Total Drag: %.3f N\n',D_total)
fprintf('Lift/Drag: %.3f\n',L/D_total)
fprintf('Moment: %.3f Nm\n',M)
    
    
    
    