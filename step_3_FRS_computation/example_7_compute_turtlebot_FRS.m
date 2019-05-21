%% description
% This script computes a Forward-Reachable Set (FRS) for the TurtleBot. The
% user specifies the SOS polynomial degree.
%
% Author: Shreyas Kousik
% Date: 21 May 2019
%
%% user parameters
% degree of SOS polynomial solution
degree = 4 ; % this should be 4 or 6 unless you have like 100+ GB of RAM

% trajectory parameter to visualize after computing the FRS:
initial_speed = 0.5 ; % m/s
k_eval = [0.5 ; 1] ;

%% automated from here
% load timing
load('turtlebot_timing.mat')
load('turtlebot_error_functions_v0_0.0_to_0.5.mat')
load('turtlebot_FRS_scaling_v0_0.0_to_0.5.mat')

% uncomment this line to make the degree 4 FRS fit inside the [-1,1]^2 box
distance_scale = 1.5 * distance_scale ;

% create agent to use for footprint
A = turtlebot_agent ;
footprint = A.footprint ;

%% set up the FRS computation variables and dynamics
% set up the indeterminates
t = msspoly('t', 1) ; % time t \in T
z = msspoly('z', 2) ; % state z = (x,y) \in Z
k = msspoly('k', 2) ; % parameters k \in K

x = z(1) ; y = z(2) ;

% create polynomials that are positive on Z, and K, thereby
% defining them as semi-algebraic sets
Z_range = [-1, 1 ; -1, 1] ; % z \in [-1,1]^2

Z0_radius = footprint/distance_scale ; % z(0) \in Z_0

K_range = [-1, 1 ; -1, 1] ; % k \in [-1,1]^2

h_Z = (z - Z_range(:,1)).*(Z_range(:,2) - z) ;

h_Z0 = 1 - ((x - initial_x)/(footprint/distance_scale)).^2 + ...
         - ((y - initial_y)/(footprint/distance_scale)).^2 ;

h_K = (k - K_range(:,1)).*(K_range(:,2) - k) ;

%% specify dynamics and error function
% set up w_des in terms of k_1
w_des = w_max*k(1) ;

% set up v_des in terms of k_2
v_range = [v0_min - delta_v, v0_max + delta_v] ;
v_range = bound_values(v_range,[0, max_speed]) ;
v_des = (diff(v_range)/2)*k(2) + mean(v_range) ;

% create dynamics
scale = (time_scale/distance_scale) ;
f = scale*[v_des - w_des*(y - initial_y) ;
                 + w_des*(x - initial_x)] ;

% create tracking error dynamics; first, make the monomials of time in
% decreasing power order
g_x_t_vec = t.^(length(g_x_coeffs)-1:-1:0) ;
g_y_t_vec = t.^(length(g_y_coeffs)-1:-1:0) ;
g = scale*[g_x_t_vec*g_x_coeffs', 0 ;
           0, g_y_t_vec*g_y_coeffs'] ;

%% create cost function
% this time around, we care about the indicator function being on Z x K
int_ZK = boxMoments([z;k], [Z_range(:,1);K_range(:,1)], [Z_range(:,2);K_range(:,2)]);

%% setup the problem structure
solver_input_problem.t = t ;
solver_input_problem.z = z ;
solver_input_problem.k = k ;
solver_input_problem.f = f ;
solver_input_problem.g = g ;
solver_input_problem.hZ = h_Z ;
solver_input_problem.hZ0 = h_Z0 ;
solver_input_problem.hK = h_K ;
solver_input_problem.cost = int_ZK ;
solver_input_problem.degree = degree ;

%% compute FRS without tracking error
solve_time = tic ;
solver_output = compute_FRS(solver_input_problem) ;
solve_time = toc(solve_time) ;

%% extract FRS polynomial result
FRS_polynomial = solver_output.indicator_function ;
FRS_lyapunov_function = solver_output.lyapunov_function ;

%% prep to visualize the output
% prep
I = FRS_polynomial ;
I_z = msubs(I,k,k_eval) ;

% create w_des and v_des from k_eval
w_in = full(msubs(w_des,k,k_eval)) ;
v_in = full(msubs(v_des,k,k_eval)) ;

% create the initial condition
z0 = [0;0;0;initial_speed] ; % (x,y,h,v)

% create the desired trajectory
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_in,v_in) ;

% create the braking trajectory
[T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

% move the robot
A.reset(z0)
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;

%% visualize FRS indicator function polynomial
figure(1) ; clf ; axis equal ; hold on ;

% plot the initial condition
plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b',...
    'Offset',-[initial_x;initial_y],'Scale',distance_scale)

% plot the FRS
plot_2D_msspoly_contour(I_z,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3],...
    'Offset',-[initial_x;initial_y],'Scale',distance_scale)

% plot the desired trajectory
plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)

% plot the agent
plot(A)

% labeling
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)