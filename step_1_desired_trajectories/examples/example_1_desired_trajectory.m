% example_1_desired_trajectory.m
%
% Create and plot an example desired trajectory.
%
% Author: Shreyas Kousik
% Date: 15 May 2019

t_f = 1 ;
w_des = 0.5 ; % rad/s
v_des = 1.0 ; % m/s
[T,U,Z] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% get the x and y positions of the trajectory
x = Z(1,:) ;
y = Z(2,:) ;

% plot
figure(1) ;
plot(x,y,'b--','LineWidth',1.5)
axis equal
legend('desired traj.')
set(gca,'FontSize',15)