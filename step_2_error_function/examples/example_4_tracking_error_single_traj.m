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

% initialize time vectors for saving tracking error; we use two to separate
% out the braking portion of the trajectory
T_err_1 = 0:t_sample:t_plan ;
T_err_2 = t_plan:t_sample:t_f ;
T_data = [T_err_1(1:end-1), T_err_2] ;

%% tracking error computation loop
% create the initial condition
z0 = [0;0;0;v0] ; % (x,y,h,v)

% create the desired trajectory
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;
pos_des = Z_go(1:2,:) ;

% split the desired trajectory into before/after t_plan parts
pos_des_1 = match_trajectories(T_err_1,T_go,pos_des) ;
pos_des_2 = match_trajectories(T_err_2,T_go,pos_des) ;

% get the braking trajectory to see tracking behavior
[T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

% reset the robot
A.reset(z0)

% track the desired trajectory
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;

% get the executed position trajectory
T = A.time ;
pos = A.state(A.position_indices,:) ;

% compute the error before t_plan
pos_1 = match_trajectories(T_err_1,T,pos) ;
pos_err_1 = abs(pos_1 - pos_des_1) ;

% compute the error after t_plan
pos_2 = match_trajectories(T_err_2,T,pos) ;
pos_err_2 = abs(pos_2 - pos_des_2) ;

% create single error data array
pos_err = [pos_err_1(:,1:end-1), pos_err_2] ;

% save data
x_data = pos_err(1,:) ;
y_data = pos_err(2,:) ;

%% plotting
figure(1) ; clf

% plot x and y error
subplot(1,2,1) ; hold on ;
plot(T_data,x_data,'--','Color',[0.5 0.2 0.2],'LineWidth',1.5)
plot(T_data,y_data,'--','Color',[0.2 0.5 0.2],'LineWidth',1.5)
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