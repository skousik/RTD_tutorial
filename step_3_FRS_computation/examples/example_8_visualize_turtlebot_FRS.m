%% description
% This script visualizes the TurtleBot FRS computed for degrees 4, 6, and
% 10, all on the same plot.
%
%% user parameters
% trajectory parameter to evaluate
k_eval = [-1;1] ;

% robot initial condition
initial_speed = 0.75 ;

% speed range (uncomment out one of the following)
v0_range = [0.0, 0.5] ;
% v0_range = [0.5, 1.0] ;
% v0_range = [1.0, 1.5] ;

%% automated from here
% load timing
load('turtlebot_timing.mat')

% load FRSes
switch v0_range(1)
    case 0.0
        FRS_4 = load('turtlebot_FRS_deg_4_v0_0.0_to_0.5.mat') ;
        FRS_6 = load('turtlebot_FRS_deg_6_v0_0.0_to_0.5.mat') ;
        FRS_10 = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;
    case 0.5
        FRS_4 = load('turtlebot_FRS_deg_4_v0_0.5_to_1.0.mat') ;
        FRS_6 = load('turtlebot_FRS_deg_6_v0_0.5_to_1.0.mat') ;
        FRS_10 = load('turtlebot_FRS_deg_10_v0_0.5_to_1.0.mat') ;
    case 1.0
        FRS_4 = load('turtlebot_FRS_deg_4_v0_1.0_to_1.5.mat') ;
        FRS_6 = load('turtlebot_FRS_deg_6_v0_1.0_to_1.5.mat') ;
        FRS_10 = load('turtlebot_FRS_deg_10_v0_1.0_to_1.5.mat') ;
    otherwise
        error('Hey! You picked an invalid speed range for the tutorial!')
end

% get one h_Z0 to plot
h_Z0 = FRS_4.h_Z0 ;
z = FRS_4.z ;
k = FRS_4.k ;

%% create agent for visualization
% get agent
A = turtlebot_agent ;

% get w_des and v_des in terms of k
w_des = FRS_4.w_des ;
v_des = FRS_4.v_des ;

% create w_des and v_des from k_eval
w_in = full(msubs(w_des,k,k_eval)) ;
v_in = full(msubs(v_des,k,k_eval)) ;

% create the initial condition
z0 = [0;0;0;initial_speed] ; % (x,y,h,v)

% create the desired trajectory
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_in,v_in) ;

% create the braking trajectory
[T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

% move the robot
A.reset(z0)
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;

%% create offsets and distance scales for the FRSes
D4 = FRS_4.distance_scale ;
O4 = -D4 * [FRS_4.initial_x ; FRS_4.initial_y] ;

D6 = FRS_6.distance_scale ;
O6 = -D6 * [FRS_6.initial_x ; FRS_6.initial_y] ;

D10 = FRS_10.distance_scale ;
O10= -D10 * [FRS_10.initial_x ; FRS_10.initial_y] ;

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

% plot degree 10 FRS
I_z_10 = msubs(FRS_10.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour(I_z_10,z,1,'LineWidth',1.5,'Color',0.4*[0.1 1 0.3],...
    'Offset',O10,'Scale',D10)

% plot the desired trajectory
plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)

% plot the agent
plot(A)

% labeling
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)

axis([-0.75, 1.75, -1, 0.75])