function [c,ceq] = constraint(var) % Function providing equality and inequality constraints
                                   % ceq(var) = 0 and c(var) \le 0 

global N;
global T;

global v;
global l;
global M;

c = []; % There are no inequality constraints

x = var(1:N+1); y = var(N+2:2*N+2); u = var(2*N+3:3*N+2); % Note: var = [x;y;u]

for i = 1:N
    
    [xDyn,yDyn] = fDyn(x(i),y(i),u(i));
    
    ceq(i) = x(i+1) - x(i) - (1.0*T/(1.0*N))*xDyn; % Imposing dynamical constraints
    ceq(i+N) = y(i+1) - y(i) - (1.0*T/(1.0*N))*yDyn;
    
    ceq(i+2*N) = x(1); % Imposing initial and final conditions
    ceq(i+2*N+1) = y(1);
    ceq(i+2*N+2) = x(N+1) - M;
    ceq(i+2*N+3) = y(N+1) - l;
end