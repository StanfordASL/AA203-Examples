clear all; clf; clc; format long;

global v; v = 1.; % Scenario
global l; l = 5.;
global M; M = 10.;

TP0Guess = [10.;.1;.1]; % Initial guess for [T;px(0);py(0)].
                        % Note that a good choice is fundamental to make the
                        % indirect method converge!

options=optimset('Display','iter','LargeScale','on');
[TP0,FVAL,EXITFLAG]=fsolve(@shootingFunc,TP0Guess,options); % Solving S(z)=0
EXITFLAG % 1 or 2 if convergence is achieved

T = TP0(1); % Plotting
fprintf('Final time T = %f',T);
P0 = [TP0(2);TP0(3)];
options = odeset('AbsTol',1e-9,'RelTol',1e-9);
[t,z] = ode113(@Rdyn,[0;T],[0.0;0.0;P0],options) ;
subplot(121); plot(z(:,1),z(:,2),'linewidth',3) ;
title('\textbf{a) Optimal Trajectory}','interpreter','latex','FontSize',22,'FontWeight','bold');
xlabel('\boldmath{$x$} \ \textbf{(m)}','interpreter','latex','FontSize',20,'FontWeight','bold');
ylabel('\boldmath{$y$} \ \textbf{(m)}','interpreter','latex','FontSize',20,'FontWeight','bold');
xlim([-1,11]);
ylim([-1,6]);
set(gca,'Xtick',-1:1:11);
set(gca,'Ytick',-1:0.5:6);
grid on;
control = zeros(size(t));
for i = 1:size(t)
    if z(i,3) ~= 0
        control(i) = atan(z(i,4)/z(i,3)); % Optimal controls from minimality condition
                                          % Note: we need to manage the case px=0 !
    else
        if z(i,3) < 0
            control(i) = pi/2.0; % Optimal controls from minimality condition: px=0
        else
            control(i) = -pi/2.0;
        end
    end
end
subplot(122); plot(t,control,'linewidth',3);
title('\textbf{b) Optimal Control}','interpreter','latex','FontSize',22,'FontWeight','bold');
xlabel('\boldmath{$t$} \ \textbf{(s)}','interpreter','latex','FontSize',20,'FontWeight','bold');
ylabel('\boldmath{$u$}','interpreter','latex','FontSize',20,'FontWeight','bold');
xlim([0,10]);
ylim([0.4,0.9]);
set(gca,'Xtick',-1:1:10);
set(gca,'Ytick',0.4:0.05:0.9);
grid on;