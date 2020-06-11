% Deriving optimal controls from the minimality condition:
% since we are looking for optimal controls in R, those must satisfy
%
% dH / du = 0.
%
% Since:
%
% H(x,y,px,py,u) = px*(v*cos(u) + fl(y)) + py*v*sin(u) + 1
%
% developing, we have:
%
% 0 = dH / du = d ( px*(v*cos(u) + fl(y)) + py*v*sin(u) + 1 ) / du =
%   = -px*v*sin(u) + py*v*cos(u).
%
% Therefore: px*sin(u) = py*cos(u).



function H = hamiltonian(x,y,px,py) % Hamiltonian related to Zermelo

global v;

[fl,flDer] = flowFunc(y);
if px ~= 0
    u = atan(py/px); % Optimal controls from minimality condition
                     % Note: we need to manage the case px=0 !
else
    if py < 0
        u = pi/2.0; % Optimal controls from minimality condition: px=0
    else
        u = -pi/2.0;
    end
end

H = px*(v*cos(u) + fl) + py*v*sin(u) + 1; % Return H(x,y,px,py)