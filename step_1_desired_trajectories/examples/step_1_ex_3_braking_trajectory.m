% description
%
% Make an example desired trajectory for the TurtleBot, and show the robot
% trying to track it, then brake along it.
%
% Author: Shreyas Kousik
% Created: 15 May 2019
% Updated: 24 Oct 2019
%
%% user parameters
% desired trajectory
w_des = 1 ; % rad/s
v_des = 1 ; % m/s

% timing
t_plan = 0.5 ; % s

% agent initial state
v_0 = 1.25 ; % rad/s

%% automated from here
% set up agent
A = turtlebot_agent() ;

% set up trajectory
t_stop = v_des/A.max_accel ;
[T_brk,U_brk,Z_brk] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des) ;

%% track the braking trajectory
% reset the agent to its initial speed
z0 = [0;0;0;v_0] ;
A.reset(z0) ;

% execute desired trajectory with braking
A.move(T_brk(end),T_brk,U_brk,Z_brk)

%% plot agent
close all

figure(1) ; hold on ;axis equal

plot_path(A.state(1:2,:),'LineWidth',1.5)
plot_path(Z_brk(1:2,:),'b--','LineWidth',1.5)
plot(A)

% set axes
xlo = min(A.state(1,:)) - 0.25 ;
xhi = max(A.state(1,:)) + 0.25 ;
ylo = min(A.state(2,:)) - 0.25 ;
yhi = max(A.state(2,:)) + 0.25 ;
axis([xlo,xhi,ylo,yhi])

% label plot
xlabel('x [m]')
ylabel('y [m]')
legend('executed traj.','desired traj.')
set(gca,'FontSize',15)

% animate agent
A.animate

%% plot desired trajectories
figure(2) ; clf ;
plot(T_brk,Z_brk','LineWidth',1.5)
xlabel('Time [s]')
ylabel('States')
title('Desired Trajectory')
legend('x','y','\theta','v')
set(gca,'FontSize',15)