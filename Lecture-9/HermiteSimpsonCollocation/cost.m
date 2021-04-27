function c = cost(var) % Cost: int^t_f_0 u(s)^2 ds

global N;

c = 0.0;
u = var(2*N+3:3*N+2); % Note: u corresponds to the last part of var

for i = 1:N
    c = c + u(i)*u(i);
end