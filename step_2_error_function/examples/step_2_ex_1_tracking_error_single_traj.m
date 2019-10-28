%% description
% This script demonstrates the tracking error computation for a single
% trajectory of the TurtleBot.
%
% Author: Shreyas Kousik
% Created: 16 May 2019
% Updated: 28 Oct 2019
%
%% user parameters
% initial condition (we only care about the initial condition in speed,
% because the dynamics are position/rotation invariant)
v_0 = 0.75 ; % m/s

% command bounds
w_des = 1.0 ; % rad/s ;
v_des = 1.0 ; % m/s

%% automated from here
% create turtlebot
A = turtlebot_agent ;

% create the initial condition
z0 = [0;0;0;v_0] ; % (x,y,h,v)
A.reset(z0)

%% trajectory setup and tracking
% make the desired trajectory
t_f = get_t_f_from_v_0(v_0) ;
[T_des,U_des,Z_des] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% track the desired trajectory
A.move(T_des(end),T_des,U_des,Z_des) ;

%% tracking error computation
% get the realized position trajectory
T = A.time ;
Z = A.state(A.position_indices,:) ;

% interpolate the realized trajectory to match the braking traj timing
pos = match_trajectories(T_des,T,Z) ;

% get the desired trajectory
pos_des = Z_des(1:2,:) ;

% compute the tracking error
pos_err = abs(pos - pos_des) ;
x_err = pos_err(1,:) ;
y_err = pos_err(2,:) ;

%% plot robot
figure(1) ; clf ; hold on ; axis equal ; grid on ;
plot_path(Z_des,'b--','LineWidth',1.5)
plot_path(Z,'b','LineWidth',1.5)
plot(A)
legend('desired traj','realized traj')
title('robot showing tracking error')
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)

%% plot tracking error
figure(2) ; clf ; hold on ;
plot(T_des,x_err,'--','Color',[0.5 0.2 0.2],'LineWidth',1.5)
plot(T_des,y_err,'--','Color',[0.2 0.5 0.2],'LineWidth',1.5)
title('tracking error in x and y')
xlabel('time [s]')
ylabel('error [m]')
legend('x error','y error')
set(gca,'FontSize',15)