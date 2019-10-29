%% description
% This script visualizes the TurtleBot FRS computed for degrees 6, 8, and
% 10 on one plot.
%
% Author: Shreyas Kousik
% Created: 21 May 2019
% Updated: 26 Oct 2019
%
%% user parameters
% trajectory to evaluate
w_des = 1 ;
v_des = 1.5 ;

% robot initial condition
v_0 = 1.25 ;

%% automated from here
% load timing
load('turtlebot_timing.mat')

% load the FRSes
FRS_06 = get_FRS_from_v_0(v_0,6) ;
FRS_08 = get_FRS_from_v_0(v_0,8) ;
FRS_10 = get_FRS_from_v_0(v_0,10) ;

% get one h_Z0 to plot
h_Z0 = FRS_06.h_Z0 ;
z = FRS_06.z ;
k = FRS_06.k ;

%% create agent for visualization
% get agent
A = turtlebot_agent ;

% create k_eval from w_des and v_des
k_eval = get_k_from_w_and_v(FRS_06,w_des,v_des) ;

% fix the initial speed
if abs(v_des - v_0) > delta_v
    error('Please pick v_0 and v_des so that |v_0 - v_des| <= delta_v')
end

% create the initial condition
z0 = [0;0;0;v_0] ; % (x,y,h,v)

% create the desired trajectory
t_f = FRS_06.time_scale ;
[T_des,U_des,Z_des] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% put trajectory into FRS frame
Z_FRS = world_to_FRS(Z_des(1:2,:),zeros(3,1),FRS_06.initial_x,FRS_06.initial_y,FRS_06.distance_scale) ;

% move the robot
A.reset(z0)
A.move(T_des(end),T_des,U_des,Z_des) ;


%% create offsets and distance scales for the FRSes
D4 = FRS_06.distance_scale ;
O4 = -D4 * [FRS_06.initial_x ; FRS_06.initial_y] ;

D6 = FRS_08.distance_scale ;
O6 = -D6 * [FRS_08.initial_x ; FRS_08.initial_y] ;

D8 = FRS_10.distance_scale ;
O8= -D8 * [FRS_10.initial_x ; FRS_10.initial_y] ;

%% plot
figure(1) ; clf ; hold on ; axis equal

% plot the initial condition
plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b',...
    'Offset',O4,'Scale',D4)

% plot degree 4 FRS
I_z_4 = msubs(FRS_06.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour(I_z_4,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3],...
    'Offset',O4,'Scale',D4)

% plot degree 6 FRS
I_z_6 = msubs(FRS_08.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour(I_z_6,z,1,'LineWidth',1.5,'Color',0.6*[0.1 1 0.3],...
    'Offset',O6,'Scale',D6)

% plot degree 8 FRS
I_z_8 = msubs(FRS_10.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour(I_z_8,z,1,'LineWidth',1.5,'Color',0.4*[0.1 1 0.3],...
    'Offset',O8,'Scale',D8)

% plot the desired (braking) trajectory
plot_path(Z_des,'b--','LineWidth',1.5)

% plot the agent
plot(A)

% labeling
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)