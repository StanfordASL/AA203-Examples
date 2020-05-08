% randsys_mpc.m
% compares the performance of finite horizon approximation
% and model predictive control on a randomly generated system
% EE364b, Convex Optimization II, S. Boyd, Stanford University.
% Written by Yang Wang, 04/2008
% Revised by Sumeet Singh, 05/2019

clear; close all; clc; 

%% Set random seed
% rng(0)
randn('state',0); rand('state',0);

%% generate A, B matrices

n = 3; m = 2;
A = randn(n); B = rand(n,m);
[U,S,V] = svd(A); A = U*V;

%% objective, constraints, initial state

Q = eye(n); R = eye(m);
Qhalf = sqrtm(Q); Rhalf = sqrtm(R);
xmax = 1*ones(n,1); xmin = -1*ones(n,1);
umax = 0.5*ones(m,1); umin = -0.5*ones(m,1);

x0 = [-0.9;0.9;-0.9];

%% Infinite vs Finite horizon
% Compare "infinite" horizon (T=200) soln w/ 
% finite horizon solns for T \in {10, 20, 40}
% finite horizon problem - use x(T) = 0 forced convergence constraint

%finite horizon problems
T_list = [10, 15, 25]; 
N_T = length(T_list);

%simulation/plot horizon
N = 50;
tvec = 0:N;

%optimal cost of finite horizon problems
opt_val_finite_T = zeros(N_T,1);
%store solutions
Xall_finite = zeros(n,N+1,N_T);
Uall_finite = zeros(m,N,N_T);

%infinite horizon
T_inf = 200;
cvx_begin
variables Xopt(n,T_inf+1) Uopt(m,T_inf)
    max(Xopt') <= xmax'; max(Uopt') <= umax';
    min(Xopt') >= xmin'; min(Uopt') >= umin';
    Xopt(:,2:T_inf+1) == A*Xopt(:,1:T_inf)+B*Uopt;
    Xopt(:,1) == x0; 
    
    %objective: sqrt of sum_k(x_k'Qx_k + u_k'Ru_k)
    minimize (norm([Qhalf*Xopt(:,1:T_inf); Rhalf*Uopt],'fro')) 
cvx_end
Jopt = cvx_optval^2;

% finite horizon approximation
for t = 1:N_T
    T = T_list(t);
    cvx_begin
        variables X(n,T+1) U(m,T)
        max(X') <= xmax'; max(U') <= umax';
        min(X') >= xmin'; min(U') >= umin';
        X(:,2:T+1) == A*X(:,1:T)+B*U;
        X(:,1) == x0; 
        X(:,T+1) == 0; %forced convergence constraint
        minimize (norm([Qhalf*X(:,1:T); Rhalf*U],'fro'))
    cvx_end
    opt_val_finite_T(t) = cvx_optval;
    Xall_finite(:,1:T+1,t) = X;
    Uall_finite(:,1:T,t) = U;
end
opt_val_finite_T = opt_val_finite_T.^2;

%% Plots for Infinite vs finite horizon

%cost comparison
figure(1)
set(gca,'Fontsize',16);
plot(T_list,opt_val_finite_T,'k'); xlabel('T'); ylabel('optvalfha');
hold on; plot(T_list,Jopt*ones(N_T,1),'k--');
title('cost');

%infinite horizon soln for x1(t) and u(t)
figure(2)
subplot(2,1,1);
set(gca,'Fontsize',16);
stairs(tvec,Xopt(1,1:N+1),'k');
axis([0,50,-1.1,1.1]); 
title('opt');
subplot(2,1,2); 
set(gca,'Fontsize',16);
stairs(tvec(1:end-1),Uopt(1,1:N),'k');
axis([0,50,-0.6,0.6]); xlabel('t');

%finite horizon soln for x1(t) and u(t) for T=10
figure(3)
subplot(2,1,1); 
set(gca,'Fontsize',16);
stairs(tvec,Xall_finite(1,:,1),'k');
axis([0,50,-1.1,1.1]); ylabel('x1');
title('hor10');
subplot(2,1,2); 
set(gca,'Fontsize',16);
stairs(tvec(1:end-1),Uall_finite(1,:,1),'k');
axis([0,50,-0.6,0.6]); xlabel('t'); ylabel('u1');

disp('Press Enter to do MPC');
pause;

%% MPC with different time-horizons 

% model predictive control w/ different horizons (T) from before
optvalmpc = zeros(N_T,1); 

%store solutions
Xallmpc = zeros(n,N+1,N_T); Uallmpc = zeros(m,N,N_T);

for t = 1:N_T
    T = T_list(t); %mpc horizon
    x = x0; %reset initial state
    Xallmpc(:,1,t) = x;
    
    fprintf('T=%d ; t = ',T);
    
    %step through time
    for i = 1:N
        fprintf('%d, ',i-1);
        
        %cvx precision
        cvx_precision(max(min(abs(x))/10,1e-6))
        
        cvx_begin quiet
            variables X(n,T+1) U(m,T)
            max(X') <= xmax'; max(U') <= umax';
            min(X') >= xmin'; min(U') >= umin';
            X(:,2:T+1) == A*X(:,1:T)+B*U;
            X(:,1) == x; %initial state constraint
            X(:,T+1) == 0; %terminal state constraint
            minimize (norm([Qhalf*X(:,1:T); Rhalf*U],'fro'))
        cvx_end
        
        %check feasibility
        if strcmp(cvx_status,'Solved')
            
            %store control
            u= U(:,1);
            Uallmpc(:,i,t) = u;
            
            %accumulate cost
            optvalmpc(t) = optvalmpc(t) + x'*Q*x + u'*R*u;
            
            %forward propagate state
            x = A*x+B*u;
            
            %record state
            Xallmpc(:,i+1,t) = x;
            
        else
           % break from loop
           optvalmpc(t) = Inf;
           break;
        end
    end
    fprintf('\n');
end

%% Plots for MPC

%add mpc cost to cost comparison
figure(1)
plot(T_list,optvalmpc,'kx-'); 
xlabel('T'); ylabel('optvals');
legend('FH','opt','mpc');

%mpc soln for x1(t) and u(t) w/ T=10
figure(4)
subplot(2,1,1);
set(gca,'Fontsize',16);
stairs(tvec,Xallmpc(1,:,1),'k');
axis([0,50,-1.1,1.1]); ylabel('x1');
title('mpc');
subplot(2,1,2); 
set(gca,'Fontsize',16);
stairs(tvec(1:end-1),Uallmpc(1,:,1),'k');
axis([0,50,-0.6,0.6]); xlabel('t'); ylabel('u1');
