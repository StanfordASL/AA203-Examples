function [noteTerm_k,DxyfDyn_k,DufDyn_k] = fDynLin(t,x_k,y_k,u_k) % Dynamics of the problem linearized
                                                    % around the nominal tuple (x_k,y_k,u_k)
global N;
global T;
                   
h = (1.0*T/(1.0*N)); % Finding what index i the time t corresponds to
indexTime = 0; indexFound = 0; iterator = 0;
while indexFound == 0
    if iterator*h <= t && t < (iterator + 1)*h % If iterator*h <= t < iteartor*h + h, then we have found the index
        indexTime = iterator + 1;
        indexFound = 1;
    end
    iterator = iterator + 1;
end
if indexFound == 0
    fprintf('Error in the Dynamics!');
end

[xDyn_k,yDyn_k,DxyfDyn_k,DufDyn_k] = fDyn(x_k(indexTime),y_k(indexTime),u_k(indexTime)); % Evaluating the original dynamics
                                                                                         % and its partial derivatives at (x_k,y_k,u_k)

% Evaluating the note term: f(x_k(t),y_k(t))- df/d(x,y)(x_k,y_k,u_k)*[x_k(t);y_k(t)] - df/du(x_k,y_k,u_k)*u_k(t)
noteTerm_k = [xDyn_k;yDyn_k] - DxyfDyn_k*[x_k(indexTime);y_k(indexTime)] - DufDyn_k*u_k(indexTime);