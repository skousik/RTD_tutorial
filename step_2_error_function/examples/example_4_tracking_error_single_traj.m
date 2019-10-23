%% description
% This script demonstrates the tracking error computation for a single
% trajectory of the TurtleBot.
%
% Author: Shreyas Kousik
% Date: 16 May 2019

%% user parameters
% initial condition (we only care about the initial condition in speed,
% because the dynamics are position/rotation invariant)
initial_speed = 0.75 ; % m/s

% command bounds
w_des = 1.0 ; % rad/s ;
v_des = 1.0 ; % m/s

% timing
t_sample = 0.01 ;

%% automated from here
% create turtlebot
A = turtlebot_agent ;

% create initial condition vector
v0 = initial_speed ;

% load timing
try
    disp('Loading turtlebot RTD planner timing info.')
    timing_info = load('turtlebot_timing.mat') ;
    t_plan = timing_info.t_plan ;
    t_stop = timing_info.t_stop ;
    t_f = timing_info.t_f ;
catch
    disp('Could not find timing MAT file. Setting defaults!')
    t_plan = 0.5 ;
    t_stop = 2.61 ;
    t_f = 0.95 ;
end

%% tracking error computation loop
% create the initial condition
z0 = [0;0;0;v0] ; % (x,y,h,v)

% create the desired trajectory
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% get the braking trajectory to see tracking behavior
[T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

% reset the robot
A.reset(z0)

% track the desired trajectory
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;

% get the executed position trajectory
T = A.time ;
Z = A.state(A.position_indices,:) ;

% interpolate the executed trajectory to match the braking traj timing
pos = match_trajectories(T_brk,T,Z) ;

% get the desired trajectory
pos_des = Z_brk(1:2,:) ;

% compute the tracking error
pos_err = abs(pos - pos_des) ;

% save data
x_data = pos_err(1,:) ;
y_data = pos_err(2,:) ;

%% plotting
figure(1) ; clf

% plot x and y error
subplot(1,2,1) ; hold on ;
plot(T_brk,x_data,'--','Color',[0.5 0.2 0.2],'LineWidth',1.5)
plot(T_brk,y_data,'--','Color',[0.2 0.5 0.2],'LineWidth',1.5)
title(['position tracking error, v_0 = ',num2str(v0,'%0.2f')])
xlabel('time [s]')
ylabel('error [m]')
legend('x error','y error','Location','NorthWest')
set(gca,'FontSize',15)

% plot robot
subplot(1,2,2) ; hold on ; axis equal
plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)
plot(A)
title('robot showing tracking error')
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)

% plot trajectories vs. desired trajectories
figure(2) ; clf ; hold on ;
plot(T_brk,Z_brk(1:2,:)')
plot(T,Z(1:2,:)')
