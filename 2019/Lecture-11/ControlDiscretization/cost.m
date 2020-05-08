function c = cost(var) % Cost: int^t_f_0 u(s)^2 ds

global N;

c = 0.0;
u = var(1:N); % Note var = [u]

for i = 1:N
    c = c + u(i)*u(i);
end