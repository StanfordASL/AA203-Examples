function [xDyn,yDyn,DxyfDyn,DufDyn] = fDyn(x,y,u) % Original dynamics of the problem and
                                                  % its gradients w.r.t. (x,y) and u
                                                  % evaluated at (x,y,u)

global v;

[fl,flDer] = flowFunc(y);

xDyn = v*cos(u) + fl;
yDyn = v*sin(u);

DxyfDyn = [0 , flDer;0 , 0];
DufDyn = [-v*sin(u);v*cos(u)];