%% description
%
% This script computes the planning time horizon for the TurtleBot by
% computing its braking distance from a user-specified maximum speed. This
% follows from Section 5 and Appendix 12 of "Bridging the Gap Between
% Safety and Real-Time Performance in Receding-Horizon Trajectory Design
% for Mobile Robots."
%
% The paper is available here: https://arxiv.org/abs/1809.06746
%
% Note, since the speed and yaw dynamics are decoupled in the high-fidelity
% model, we can compute the braking time by just considering the robot
% going in a straight line.
%
% Author: Shreyas Kousik
% Date: 12 May 2019
%
%% user parameters
% trajectory information
max_speed = 1.5 ; % m/s
t_plan = 0.5 ;

% this yaw rate and speed are used to check if the TurtleBot can brake
% while staying close to the desired trajectory
validation_desired_yaw_rate = -0.75 ;
validation_desired_speed = 1.1 ;

%% automated from here
% make robot
A = turtlebot_agent ;

% set initial state (x,y,h,v)
z0 = [0;0;0;max_speed] ;
A.reset(z0) ;

% brake to a stop for 5s (this is long enough to make sure the TurtleBot
% actually comes to a stop)
A.stop(5)

% get braking time as the first time the agent's speed falls below 1e-3 m/s
% and round up to the nearest 0.01s
stop_log = A.state(A.speed_index,:) <= 1e-3 ;
stop_idx = find(stop_log,1,'first') ;
t_stop = A.time(stop_idx) ;
t_stop = round(t_stop,2) ;

% get braking distance as in (85)
d_stop = norm(A.state(A.position_indices,end) - [0;0]) ;

% get time required to travel the braking distance at max speed as in (91)
t_v = d_stop / max_speed ;

% output time horizon as in (92), rounding up to nearest 0.01s
t_f = round(t_plan + t_v,2) ;
disp(['Minimum planning time horizon: ',num2str(t_f,'%0.2f'),' seconds'])

%% save info
disp('Saving timing information to turtlebot_timing.mat')
save('turtlebot_timing.mat','t_f','t_plan','t_stop')

%% validate planning time horizon
% create a desired trajectory at the max speed
w_des = validation_desired_yaw_rate ;
v_des = validation_desired_speed ;
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% convert the desired trajectory to a braking trajectory that can be used
% by the TurtleBot with the PD controller that we're using for RTD (NOTE
% we don't have to use PD for RTD, but we've chosen it for this example)
[T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

% reset the agent to its max speed
A.reset(z0) ;

% execute desired trajectory with braking
A.move(T_brk(end),T_brk,U_brk,Z_brk)

%% plotting
close all
figure(1)
hold on
axis equal

plot(A.state(1,:),A.state(2,:),'LineWidth',1.5)
plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)
plot(A)
xlabel('x [m]')
ylabel('y [m]')
legend('executed traj.','desired traj.')
set(gca,'FontSize',15)
