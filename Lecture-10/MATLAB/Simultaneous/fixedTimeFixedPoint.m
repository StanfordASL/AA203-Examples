clear all; clf; clc; format long;

global N; N = 20; % Scenario
global T; T = 10.;

global v; v = 1.;
global l; l = 5.;
global M; M = 10.;

global uMax; uMax = 1.;%uMax = 0.75; %uMax = 1.;  % Imposing constraints on the control
                                      % You may change this

uInit = .5*uMax*ones(N,1); % Initialization on the control
xInit = ones(N+1,1); yInit = ones(N+1,1); % Initialization on the state
varInit = [xInit; yInit; uInit];

lb = -uMax*ones(3*N+2,1); ub = uMax*ones(3*N+2,1); % Lower and upper bounds. For the control: |u| \le uMax
lb(1:N+1) = 0.0; ub(1:N+1) = M; % For the state x : 0 \le x \le M
lb(N+2:2*N+2) = 0.0; ub(N+2:2*N+2) = l; % For the state y : 0\le x \le l

options=optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',10000);
[var,Fval,convergence] = fmincon(@cost,varInit,[],[],[],[],lb,ub,@constraint,options); % Solving the problem
convergence % = 1, good

x = var(1:N+1); y = var(N+2:2*N+2); u = var(2*N+3:3*N+2); % Collecting the solution
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