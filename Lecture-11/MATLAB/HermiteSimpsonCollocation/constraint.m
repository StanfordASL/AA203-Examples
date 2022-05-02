function [c,ceq] = constraint(var) % Function providing equality and inequality constraints
                                   % ceq(var) = 0 and c(var) \le 0 

global N;
global T;

global v;
global l;
global M;

c = []; % There are no inequality constraints

x = var(1:N+1); y = var(N+2:2*N+2); u = var(2*N+3:3*N+3); % Note: var = [x;y;u]

for i = 1:N
    [xDyn_i,yDyn_i] = fDyn(x(i),y(i),u(i)); % Evaluating dynamics at times t_i and t_{i+1}
    [xDyn_ii,yDyn_ii] = fDyn(x(i+1),y(i+1),u(i+1));
    
    x_ic = (1./2.)*(x(i) + x(i+1)) + (1.0*T/(1.0*N))/8.*(xDyn_i - xDyn_ii); % Evaluating state and control at collocation points via the Hermite-Simpson formula
    y_ic = (1./2.)*(y(i) + y(i+1)) + (1.0*T/(1.0*N))/8.*(yDyn_i - yDyn_ii);
    u_ic = (u(i) + u(i+1))/2.;
    
    [xDyn_ic,yDyn_ic] = fDyn(x_ic,y_ic,u_ic); % Evaluating dynamics at collocation points
    
    ceq(i) = x(i+1) - x(i) - ((1.0*T/(1.0*N))/6.0)*(xDyn_i + 4*xDyn_ic + xDyn_ii); % Imposing dynamical constraints via the Hermite-Simpson formula
    ceq(i+N) = y(i+1) - y(i) - ((1.0*T/(1.0*N))/6.0)*(yDyn_i + 4*yDyn_ic + yDyn_ii);
end

ceq(1+2*N) = x(1); % Imposing initial and final conditions
ceq(2+2*N) = y(1);
ceq(3+2*N) = x(N+1) - M;
ceq(4+2*N) = y(N+1) - l;