clear all; clf; clc; format long;

global N; N = 30; % Scenario
global T; T = 10.;

global v; v = 1.;
global l; l = 5.;
global M; M = 10.;

global uMax; uMax = 1.; %uMax = 0.75; % Imposing constraints on the control
                                      % You may change this

uInit = .5*uMax*ones(N,1); % Initialization on the control
varInit = uInit;

lb = -uMax*ones(N,1); ub = uMax*ones(N,1); % Lower and upper bounds for the control: |u| \le uMax

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',10000,'StepTolerance',1e-12);
[var,Fval,convergence] = fmincon(@cost,varInit,[],[],[],[],lb,ub,@constraint,options); % Solving the problem
convergence % = 1, good

x = zeros(N+1,1); y = zeros(N+1,1); u = var(1:N); % Collecting the solution
                                                  % Note that, since x and
                                                  % y are not part of the
                                                  % variables that we optimize,
                                                  % we need to integrate to
                                                  % recover them
t = zeros(N,1);
for i = 1:N-1
    t(i+1) = t(i) + (1.0*T/(1.0*N));
end
for i = 1:N
    [xDyn,yDyn] = fDyn(x(i),y(i),u(i));
    x(i+1) = x(i) + (1.0*T/(1.0*N))*xDyn;
    y(i+1) = y(i) + (1.0*T/(1.0*N))*yDyn;
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