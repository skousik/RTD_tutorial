%% description
% This script plots the output of a saved Turtlebot FRS in the FRS frame
% and the global frame
%
% Author: Shreyas Kousik
% Created: 28 Oct 2019
% Updated: 29 Oct 2019
%
clear ; clc ; close all
%% user parameters
% initial speed
v_0 = 1.25 ; % m/s

% yaw rate and speed
w_des = 0.4 ; % rad/s
v_des = 1.25 ; % m/s

% degree FRS to visualize (pick 4, 6, 8, 10, or 12)
degree = 6 ;

%% automated from here
% load timing
load('turtlebot_timing.mat')

% load the right FRS
FRS = get_FRS_from_v_0(v_0,degree) ;

% fix the initial speed
if abs(v_des - v_0) > delta_v
    error('Please pick v_0 and v_des so that |v_0 - v_des| <= delta_v')
end

%% set up and move the turtlebot
% get agent
A = turtlebot_agent ;

% create the initial condition
z0 = [0;0;0;v_0] ; % (x,y,h,v)

% create the desired trajectory
t_f = FRS.time_scale ;
[T_des,U_des,Z_des] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% put trajectory into FRS frame
Z_FRS = world_to_FRS(Z_des(1:2,:),zeros(3,1),FRS.initial_x,FRS.initial_y,FRS.distance_scale) ;

% create the braking trajectory for the robot to track in the world frame
t_stop = get_t_stop_from_v(v_des) ;
[T_brk,U_brk,Z_brk] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des) ;

% move the robot
A.reset(z0)
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;

%% prep to plot FRS
% create k_eval from w_des and v_des
k_eval = get_k_from_w_and_v(FRS,w_des,v_des) ;

% get FRS initial condition set and variables
z = FRS.z ;
k = FRS.k ;
h_Z0 = FRS.h_Z0 ;

% get FRS polynomial
I = FRS.FRS_polynomial ;
I_z = msubs(I,FRS.k,k_eval) ;

%% plot in FRS frame
figure(1) ;
subplot(1,2,1) ; hold on ; axis equal ;

% plot initial condition
plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b')

% plot FRS subset for given k
plot_2D_msspoly_contour(I_z,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3])

% plot desired trajectory
plot_path(Z_FRS,'b--','LineWidth',1.5)

set(gca,'FontSize',15)
axis([-1,1,-1,1])
xlabel('x / (dist. scale)')
ylabel('y / (dist. scale)')
title('FRS Frame')

%% plot in global frame
subplot(1,2,2) ; hold on ; axis equal;

% plot the initial condition
offset = -FRS.distance_scale*[FRS.initial_x ; FRS.initial_y] ;
plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b',...
    'Offset',offset,'Scale',FRS.distance_scale)

% plot the FRS
plot_2D_msspoly_contour(I_z,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3],...
    'Offset',offset,'Scale',FRS.distance_scale)

% plot the desired trajectory
plot_path(Z_des,'b--','LineWidth',1.5)

% plot the agent
plot(A)

% labeling
set(gca,'FontSize',15)
xlabel('x [m]')
ylabel('y [m]')
title('World Frame')