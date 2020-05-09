function var = solveLOCP(var_k) % Procedure to solve (LOCP) at iteration k+1
                                % var_k contains the trajectories and
                                % the controls at the previous SCP
                                % iteration, around which, we linearize
                                % the dynamics
                                % Note: var contains the discretization
                                % in time of (x,y,u) stored as
                                % (x_0,y_0,u_0,x_1,y_1,u_1,...,x_N,y_N)

global N;
global T;

global l;
global M;

global epsTrust;
global uMax;

x_k = zeros(N+1,1); y_k = zeros(N+1,1); u_k = zeros(N,1); % recovering x_k, y_k and u_k form var_k
for i = 1:N+1
    x_k(i) = var_k(3*i-2);
    y_k(i) = var_k(3*i-1);
    if i <= N
        u_k(i) = var_k(3*i);
    end
end

C = zeros(2*N+4,3*N+2); % Cvx accepts equality constraints in the form C*var=d
                        % and bounds of type lb <= var <= ub.
                        % C will contain all the linearized dynamical
                        % constraints
d = zeros(2*N+4,1);
lb = zeros(3*N+2,1);
ub = zeros(3*N+2,1);

C(2*N+1,1) = 1.; C(2*N+2,2) = 1.; % Initial conditions on the state
d(2*N+1) = 0.; d(2*N+2) = 0.;

C(2*N+3,3*N+1) = 1.; C(2*N+4,3*N+2) = 1.; % Final conditions on the state
d(2*N+3) = M; d(2*N+4) = l;

h = (1.0*T/(1.0*N));
for i = 1:N % Dynamics constraints
    [noteTerm_k,DxyfDyn_k,DufDyn_k] = fDynLin((i-1)*h,x_k,y_k,u_k);
    
    d(2*i-1) = h*noteTerm_k(1); d(2*i) = h*noteTerm_k(2);
    
    C(2*i-1,3*i-2) = -1. - h*DxyfDyn_k(1,1); % Filling the matrix constraint C with
                                             % linearized dynamical
                                             % constraints
    C(2*i-1,3*i-1) = -h*DxyfDyn_k(1,2);
    C(2*i-1,3*i) = -h*DufDyn_k(1);
    C(2*i-1,3*i+1) = 1.;
    
    C(2*i,3*i-2) = -h*DxyfDyn_k(2,1);
    C(2*i,3*i-1) = -1. - h*DxyfDyn_k(2,2);
    C(2*i,3*i) = -h*DufDyn_k(2);
    C(2*i,3*i+2) = 1.;
end

for i = 1:N+1
    ub(3*i-2) = x_k(i) + epsTrust; % Lower and upper bounds will contain constant
                                   % "trust region" constraints. These are:
                                   % |x(t)-x_k(t)| <= epsTrust , |y(t)-y_k(t)| <= epsTrust
                                   % Trust region constraints help to
                                   % obtain good linearizations around
                                   % (x_k,y_k,u_k)
    ub(3*i-1) = y_k(i) + epsTrust;
    if i <= N
        ub(3*i) = 10;
    end
    
    lb(3*i-2) = x_k(i) - epsTrust;
    lb(3*i-1) = y_k(i) - epsTrust;
    if i <= N
        lb(3*i) = -10;
    end
end

cvx_begin % Beginning of Cvx environment
    %cvx_begin quiet; % This removes Cvx output

    variable z(3*N+2) % Defining the main variable
    
    cost = 0.;
    for i = 1:N % To increase the chance to find feasible solutions at each SCP iteration, we rather penalize cost/state-constraints
       cost = cost + z(3*i)*z(3*i) + max(z(3*i) - uMax,0.) + max(-(z(3*i) + uMax),0.) + max(-z(3*i-2),0.) + max(-z(3*i-1),0.) + max(z(3*i-2) - M,0.) + max(z(3*i-1) - l,0.);
    end

    minimize( cost ) % Defining and solving the problem
    subject to
        C * z == d
        lb <= z <= ub
cvx_end

var = z;
