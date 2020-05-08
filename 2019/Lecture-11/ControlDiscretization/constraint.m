function [c,ceq] = constraint(var) % Function providing equality and inequality constraints
                                   % ceq(var) = 0 and c(var) \le 0

global N;
global T;

global l;
global M;

u = var(1:N); % Note that in this case: var = [u]

x_N = 0.0;
y_N = 0.0;

for i = 1:N
    [xDyn,yDyn] = fDyn(x_N,y_N,u(i)); % "Implicitely" evaluating x and y
                                      % at each time t_i, by integrating
                                      % the discretized dynamics until t_i
    x_N = x_N + (1.0*T/(1.0*N))*xDyn;
    y_N = y_N + (1.0*T/(1.0*N))*yDyn;
    
    c(i) = -x_N; % Imposing "implicit" box-type constraints:
                 % 0 \le x \le M , 0 \le y \le l
    c(i+N) = x_N - M;
    c(i+2*N) = -y_N;
    c(i+3*N) = y_N - l;
end

ceq(1) = x_N - M; % Imposing final conditions
ceq(2) = y_N - l;