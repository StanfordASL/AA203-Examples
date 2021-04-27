function [xDyn,yDyn] = fDyn(x,y,u) % Original dynamics of the problem

global v;

[fl,flDer] = flowFunc(y);

xDyn = v*cos(u) + fl;
yDyn = v*sin(u);