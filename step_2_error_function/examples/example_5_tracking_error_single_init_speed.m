%% description
% This script demonstrates the tracking error computation for a single
% initial condition of the TurtleBot.
%
% Author: Shreyas Kousik
% Date: 16 May 2019

%% user parameters
% initial condition (we only care about the initial condition in speed,
% because the dynamics are position/rotation invariant)
initial_speed = 0.75 ; % m/s

% command bounds
w_min = -1.0 ; % rad/s
w_max =  1.0 ; % rad/s
delta_v = 0.25 ; % m/s

% number of samples in w, and v
N_samples = 4 ;

% timing
t_sample = 0.01 ;

%% automated from here
% create turtlebot
A = turtlebot_agent ;

% create initial condition vector
v0 = initial_speed ;

% create yaw commands
w_vec = linspace(w_min,w_max,N_samples) ;

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

% initialize arrays for saving tracking error; note there will be one row
% for every (w_des,v_des) combination, so there are N_samples^2 rows
N_total = length(w_vec)*N_samples ;
x_data = nan(N_total,length(T_data)) ;
y_data = nan(N_total,length(T_data)) ;

%% tracking error computation loop
err_idx = 1 ;

tic

% create the initial condition
z0 = [0;0;0;v0] ; % (x,y,h,v)

% create the feasible speed commands from the initial condition
v_vec = linspace(v0 - delta_v, v0 + delta_v, N_samples) ;

% for each yaw and speed command...
for w_des = w_vec
    for v_des = v_vec
        % create the desired trajectory
        [T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;
        pos_des = Z_go(1:2,:) ;

        % split the desired trajectory into before/after t_plan parts
        pos_des_1 = match_trajectories(T_err_1,T_go,pos_des) ;
        pos_des_2 = match_trajectories(T_err_2,T_go,pos_des) ;

        % get the braking trajectory to see tracking behavior
        [T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

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
        x_data(err_idx,:) = pos_err(1,:) ;
        y_data(err_idx,:) = pos_err(2,:) ;

        % increment counter
        err_idx = err_idx + 1 ;
    end
end
toc

%% fit g with a polynomial
% get the max of the error
x_err = max(x_data,[],1) ;
y_err = max(y_data,[],1) ;

% fit polynomials to the data
int_g_x_coeffs = polyfit(T_data,x_err,4) ;
int_g_y_coeffs = polyfit(T_data,y_err,4) ;

% take the time derivative of these to get the g functions in x and y; this
% is ok because the constant terms in the position error are negative and
% small, so we can treat the constant of integration for g_x and g_y as 0
g_x_coeffs = polyder(int_g_x_coeffs) ;
g_y_coeffs = polyder(int_g_y_coeffs) ;

%% evaluate polynomial for plotting
int_g_x_coeffs(end) = 0 ;
int_g_y_coeffs(end) = 0 ;
g_x_plot = polyval(int_g_x_coeffs,T_data ) ;
g_y_plot = polyval(int_g_y_coeffs,T_data ) ;

%% plotting
figure(1) ;

% plot x error
subplot(2,1,1) ; hold on ;
plot(T_data,x_data','b--')
g_x_handle = plot(T_data,g_x_plot,'r-','LineWidth',1.5) ;
title(['error in x position, v_0 = ',num2str(v0,'%0.2f')])
xlabel('time [s]')
ylabel('x error [m]')
legend(g_x_handle,'\int g_x(t) dt','Location','NorthWest')
set(gca,'FontSize',15)

% plot y error
subplot(2,1,2) ; hold on ;
plot(T_data,y_data','b--')
g_y_handle = plot(T_data,g_y_plot,'r-','LineWidth',1.5) ;
title(['error in y position, v_0 = ',num2str(v0,'%0.2f')])
xlabel('time [s]')
ylabel('y error [m]')
legend(g_y_handle,'\int g_y(t) dt','Location','NorthWest')
set(gca,'FontSize',15)