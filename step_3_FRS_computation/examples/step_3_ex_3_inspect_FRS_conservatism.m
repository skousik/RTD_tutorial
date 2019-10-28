%% description
% This script visualizes the TurtleBot FRS computed for degrees 4, 6, and
% 8, all on the same plot.
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

% load the right FRS
FRS_4 = get_FRS_from_v_0(v_0,4) ;
FRS_6 = get_FRS_from_v_0(v_0,6) ;
FRS_8 = get_FRS_from_v_0(v_0,8) ;

% get one h_Z0 to plot
h_Z0 = FRS_4.h_Z0 ;
z = FRS_4.z ;
k = FRS_4.k ;

%% create agent for visualization
% get agent
A = turtlebot_agent ;

% create k_eval from w_des and v_des
k_eval = get_k_from_w_and_v(FRS_4,w_des,v_des) ;

% fix the initial speed
if abs(v_des - v_0) > delta_v
    error('Please pick v_0 and v_des so that |v_0 - v_des| <= delta_v')
end

% create the initial condition
z0 = [0;0;0;v_0] ; % (x,y,h,v)

% create the desired trajectory
t_stop = get_t_stop_from_v(v_des) ;
[T_brk,U_brk,Z_brk] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des) ;

% put trajectory into FRS frame
Z_FRS = world_to_FRS(Z_des(1:2,:),zeros(3,1),FRS.initial_x,FRS.initial_y,FRS.distance_scale) ;

% move the robot
A.reset(z0)
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;


%% create offsets and distance scales for the FRSes
D4 = FRS_4.distance_scale ;
O4 = -D4 * [FRS_4.initial_x ; FRS_4.initial_y] ;

D6 = FRS_6.distance_scale ;
O6 = -D6 * [FRS_6.initial_x ; FRS_6.initial_y] ;

D8 = FRS_8.distance_scale ;
O8= -D8 * [FRS_8.initial_x ; FRS_8.initial_y] ;

%% plot
figure(1) ; clf ; hold on ; axis equal

% plot the initial condition
plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b',...
    'Offset',O4,'Scale',D4)

% plot degree 4 FRS
I_z_4 = msubs(FRS_4.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour(I_z_4,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3],...
    'Offset',O4,'Scale',D4)

% plot degree 6 FRS
I_z_6 = msubs(FRS_6.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour(I_z_6,z,1,'LineWidth',1.5,'Color',0.6*[0.1 1 0.3],...
    'Offset',O6,'Scale',D6)

% plot degree 8 FRS
I_z_8 = msubs(FRS_8.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour(I_z_8,z,1,'LineWidth',1.5,'Color',0.4*[0.1 1 0.3],...
    'Offset',O8,'Scale',D8)

% plot the desired (braking) trajectory
plot_path(Z_brk,'b--','LineWidth',1.5)

% plot the agent
plot(A)

% labeling
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)

axis([-0.75, 1.75, -1, 0.75])