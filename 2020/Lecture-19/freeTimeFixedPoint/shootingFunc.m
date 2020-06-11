function XZero = shootingFunc(X) % Function S(z) for which we seek zeros

global l;
global M;

tf = X(1);
px0 = X(2);
py0 = X(3);

options = odeset('AbsTol',1e-9,'RelTol',1e-9);
[t,z] = ode113(@Rdyn,[0.0;tf],[0.0;0.0;px0;py0],options); % ODE: \dot{z}(t) = R(z(t))

xf = z(end,1);
yf = z(end,2);
pxf = z(end,3);
pyf = z(end,4);

XZero = [xf - M; yf - l; % Conditions x(tf) - M = 0 , y(tf) - l = 0
    hamiltonian(xf,yf,pxf,pyf)]; % Condition H(tf) = 0