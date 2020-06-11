function XZero = shootingFunc(X) % Function S(z) for which we seek zeros

global v;
global l;

tf = X(1);
px0 = X(2);
py0 = X(3);

options = odeset('AbsTol',1e-9,'RelTol',1e-9);
[t,z] = ode113(@Rdyn,[0.0;tf],[0.0;0.0;px0;py0],options); % ODE: \dot{z}(t) = R(z(t))

xf = z(end,1);
yf = z(end,2);
pxf = z(end,3);
pyf = z(end,4);

% Here we need to return S(z) = [ condition on the final p(tf) ; F(x(tf),y(tf)) ; H(tf) ].
% The last one is the usual "Hamiltonian at tf is zero". The one in the
% middle is "(x,y)(tf) \in Mf = {(x,y) \in R^2 : F(x,y) = y - l = 0 }". The first one
% is p(tf) \perp ker dF(x(tf),y(tf)) (since the final cost h is zero for our problem).
% Let us see how p(tf) \perp ker dF(x(tf),y(tf)) translates in practice.
% First of all we see that: dF(x,y) = (0,1). If dF(x,y)'(w1,w2)=0,
% then: w2 = 0. It follows: ker dF(x(tf),y(tf)) = {(w1,w2) \in R^2: w2=0}.
% Hence, p(tf) \perp ker dF(x(tf),y(tf)) translates in:
% (px(tf),py(tf))'(w1,0) = 0 for every w1 \in R, which gives:
% px(tf) = 0, which is our the condition on the final p(tf).

XZero = [pxf; % Condition is p(tf) \perp ker dF(x(tf),y(tf))
    yf - l; % Condition F(x(tf),y(tf)) = 0
    hamiltonian(xf,yf,pxf,pyf)]; % Condition H(tf) = 0