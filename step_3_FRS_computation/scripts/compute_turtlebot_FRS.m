%% description
% This script computes a Forward-Reachable Set (FRS) for the TurtleBot. The
% user specifies
%
% Author: Shreyas Kousik
% Date: 20 May 2019
%
%% user parameters
% degree of SOS polynomial solution
degree = 6 ; % this should be < 8 if you don't have like 100+ GB of RAM

% include tracking error or not
include_tracking_error = false ;

% speed range (uncomment out one of the following)
v0_range = [0.0, 0.5] ;
% v0_range = [0.5, 1.0] ;
% v0_range = [1.0, 1.5] ;

% whether or not to save output
save_result = true ;

%% automated from here
% load timing
load('turtlebot_timing.mat')

% load the error functions and distance scales
switch v0_range(1)
    case 0.0
        load('turtlebot_error_functions_v0_0.0_to_0.5.mat')
        load('turtlebot_FRS_scaling_v0_0.0_to_0.5.mat')
    case 0.5
        load('turtlebot_error_functions_v0_0.5_to_1.0.mat')
        load('turtlebot_FRS_scaling_v0_0.5_to_1.0.mat')
    case 1.0
        load('turtlebot_error_functions_v0_1.0_to_1.5.mat')
        load('turtlebot_FRS_scaling_v0_1.0_to_1.5.mat')
    otherwise
        error('Hey! You picked an invalid speed range for the tutorial!')
end

% create agent to use for footprint
A = turtlebot_agent ;
footprint = A.footprint ;

%% set up the FRS computation variables and dynamics
% set up the indeterminates
t = msspoly('t', 1) ; % time t \in T
z = msspoly('z', 2) ; % state z = (x,y) \in Z
k = msspoly('k', 2) ; % parameters k \in K

x = z(1) ; y = z(2) ;

% create polynomials that are positive on T, Z, and K, thereby
% defining them as semi-algebraic sets
Z_range = [-1, 1 ; -1, 1] ; % z \in [-1,1]^2

Z0_radius = footprint/distance_scale ; % z(0) \in Z_0

K_range = [-1, 1 ; -1, 1] ; % k \in [-1,1]^2

h_Z = (z - Z_range(:,1)).*(Z_range(:,2) - z) ;

h_Z0 = 1 - ((x - initial_x)/(footprint/distance_scale)).^2 + ...
         - ((y - initial_y)/(footprint/distance_scale)).^2 ;

h_K = (k - K_range(:,1)).*(K_range(:,2) - k) ;

%% specify dynamics and error function
% set up w_des and v_des in terms of k_1
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
g = [g_x_t_vec*g_x_coeffs' ;
     g_y_t_vec*g_y_coeffs'] ;

%% create cost function
% this time around, we care about the indicator function being on Z x K
int_ZK = boxMoments([z;k], [Z_range(:,1);K_range(:,1)], [Z_range(:,2);K_range(:,2)]);

%% setup the problem structure
solver_input_problem.t = t ;
solver_input_problem.z = z ;
solver_input_problem.k = k ;
solver_input_problem.f = f ;
solver_input_problem.hZ = h_Z ;
solver_input_problem.hZ0 = h_Z0 ;
solver_input_problem.hK = h_K ;
solver_input_problem.cost = int_ZK ;
solver_input_problem.degree = degree ;

if include_tracking_error
    solver_input_problem.g = g ;
end

%% compute FRS without tracking error
solve_time = tic ;
solver_output = compute_FRS(solver_input_problem) ;
solve_time = toc(solve_time) ;

%% extract FRS polynomial result
FRS_polynomial = solver_output.indicator_function ;
FRS_lyapunov_function = solver_output.lyapunov_function ;

%% save result
if save_result
    % create the filename for saving
    filename = ['turtlebot_FRS_deg_',num2str(degree),'_v0_',...
                num2str(v0_min,'%0.1f'),'_to_',...
                num2str(v0_max,'%0.1f'),'.mat'] ;

    % save!
    save(filename,'FRS_polynomial','FRS_lyapunov_function','t','z','k',...
        'time_scale','distance_scale','v0_min','v0_max','v_des','w_des',...
        'max_speed','footprint','f','g','initial_x','initial_y','t_f',...
        't_plan','t_stop','v_range','delta_v','degree','h_Z','h_Z0','h_K')
end

%% visualize the output
% prep
I = FRS_polynomial ;
I_z = msubs(I,k,[0;1]) ;

% plot
figure(1) ; clf ; hold on ;

plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b')
plot_2D_msspoly_contour(I_z,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3])

axis equal
set(gca,'FontSize',15)
xlabel('x [m]')
ylabel('y [m]')