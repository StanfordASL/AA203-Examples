function [fl,flDer] = flowFunc(y) % Parabolic flow in 0 \le y \le l

global l;

coeff = 0.35; % Leading coefficient: maximal magnitude
normFlow = l^2/4.0; % To impose: \int^l_0 flow(y(s)) ds = coeff
fl = (coeff/normFlow)*y*(l - y);
flDer = (coeff/normFlow)*(l - 2*y);