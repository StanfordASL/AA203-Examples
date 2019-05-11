clear all; close all; clc; format long;

global N; N = 10; % Scenario
global T; T = 10.;

global v; v = 1.;
global l; l = 5.;
global M; M = 10.;

global epsTrust; epsTrust = 3; % Trust region "radius": try to modify it
                               % to see how this affects the convergence of
                               % SCP
global uMax; uMax = 1.; %uMax = 0.75; % Imposing constraints on the control
                                     % You may change this
global epsSCP; epsSCP = 5e-1;
                                     
x_0 = zeros(N+1,1); y_0 = zeros(N+1,1); u_0 = zeros(N,1);
for i = 1:N+1
    x_0(i) = M*(i-1)/(1.0*N); % (x_0,y_0,u_0) represents the initial tuple
                              % around which we linearize the original
                              % dinamics. This does not need to be
                              % feasible. For the state, we choose a
                              % "straight-line" connecting (0,0) to (M,l)
    y_0(i) = l*(i-1)/(1.0*N);
    if i <= N
        u_0(i) = 0.5*uMax;
    end
end

var_k = zeros(3*N+2,1);
for i = 1:N+1
    var_k(3*i-2) = x_0(i);
    var_k(3*i-1) = y_0(i);
    if i <= N
        var_k(3*i) = u_0(i);
    end
end

var = zeros(3*N+2,1); % Running SCP procedure
counter = 1; done = 0;
while done == 0
    
    fprintf('SCP iteration = %i\n',counter);
    var = solveLOCP(var_k);
    if norm(var - var_k,Inf) < epsSCP % SCP ends if the variation between
                                      % previous iteration is smaller than
                                      % a given tolerance
        done = 1;
    end
    var_k = var;
    counter = counter + 1;
    
end

x = zeros(N+1,1); y = zeros(N+1,1); u = zeros(N,1); % Collecting the solution
for i = 1:N+1
    x(i) = var(3*i-2);
    y(i) = var(3*i-1);
    if i <= N
        u(i) = var(3*i);
    end
end
t = zeros(N,1);
for i = 1:N-1
    t(i+1) = t(i) + (1.0*T/(1.0*N));
end

subplot(121); plot(x,y,'linewidth',3); % Plotting
title('\textbf{a) Optimal Trajectory}','interpreter','latex','FontSize',22,'FontWeight','bold');
xlabel('\boldmath{$x$} \ \textbf{(m)}','interpreter','latex','FontSize',20,'FontWeight','bold');
ylabel('\boldmath{$y$} \ \textbf{(m)}','interpreter','latex','FontSize',20,'FontWeight','bold');
xlim([-1,11]);
ylim([-1,6]);
set(gca,'Xtick',-1:1:11);
set(gca,'Ytick',-1:1:6);
grid on;
subplot(122); plot(t,u,'linewidth',3);
title('\textbf{b) Optimal Control}','interpreter','latex','FontSize',22,'FontWeight','bold');
xlabel('\boldmath{$t$} \ \textbf{(s)}','interpreter','latex','FontSize',20,'FontWeight','bold');
ylabel('\boldmath{$u$}','interpreter','latex','FontSize',20,'FontWeight','bold');
xlim([0,10]);
ylim([-0.25,1]);
set(gca,'Xtick',-1:1:10);
set(gca,'Ytick',-0.25:0.1:1);
grid on;