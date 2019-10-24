%% description
% This script checks that a non-braking trajectory will travel farther than
% a braking trajectory, given the appropriate duration.
%
% Author: Shreyas Kousik
% Created: 24 Oct 2019
% Updated: -
%
%% automated from here
% set up speed and yaw rate
v_max = 1.5 ; % m/s
w_des = 0.0 ; % rad/s

% set up timing
t_plan = 0.5; % s

% create turtlebot
A = turtlebot_agent() ;

% create non-braking trajectory
t_f = t_plan + 0.4 ;
[T_go,~,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_max) ;

% create braking trajectory
t_stop = v_max / A.max_accel ;
[T_brk,~,Z_brk] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_max) ;

%% plot
figure(1) ; clf ; hold on ; grid on ;

% plot trajectories
plot(T_go,Z_go(1,:),T_brk,Z_brk(1,:),'LineWidth',1.5)
plot([0,1.5],max(Z_brk(1,:)).*ones(1,2),'k--')

% label plot
xlabel('time [s]')
ylabel('distance [m]')
legend('non-braking','braking','Location','SouthEast')
set(gca,'FontSize',15)