%% description
% This script computes an FRS for the TurtleBot given a particular, fixed
% command input. So, the problem is only 3-D (x,y,t), which can be solved
% on most laptops. In addition, we don't include the tracking error
% function, which further reduces memory requirements.
%
% Author: Shreyas Kousik
% Date: 16 May 2019
%
%% user parameters
% chosen command
w_des = 0.5 ; % rad/s
v_des = 1.0 ; % m/s

% initial speed (varying this will change tracking for visualization)
initial_speed = 0.75 ; % m/s

% timing information
load('turtlebot_timing.mat')

% sums-of-squares polynomial degree
degree = 6 ; % this should be an even number

%% automated from here
% first, let's figure out how far the robot is going to travel, so that we
% know how to scale and shift the whole problem to fit in the [-1,1]^n box;
% this is because we are optimizing over SOS polynomials of high degree, so
% they can get really large when evaluated on values greater than 1

% get the robot's footprint
A = turtlebot_agent ;
footprint = A.footprint ;

% make the trajectory we are computing an FRS for; notice that the dynamics
% are time-scaled by t_f, then ode45 is called over the time horizon [0,1],
% since we are going to normalize time in the FRS computation
T = [0, 1] ;
U = [w_des, w_des ; v_des, v_des] ;
z0 = zeros(3,1) ;
[~,Z] = ode45(@(t,z) t_f.*turtlebot_trajectory_producing_model(t,z,T,U),T,z0) ;
Z = Z' ; % transpose to get column trajectory format

% figure out how far the robot traveled in x and y, and add the footprint
dx = max(Z(1,:)) - min(Z(1,:)) + footprint ;
dy = max(Z(2,:)) - min(Z(2,:)) + footprint ;

% pick a scaling factor that makes the larger of dx and dy equal to 1, then
% make the center-of-mass dynamics start at (-0.5,0) when scaled
time_scale = t_f ;
distance_scale = max(dx,dy) ;
initial_x = -0.5 ;
initial_y =  0.0 ;

%% set up the FRS computation variables and dynamics
% set up FRS variables
t = msspoly('t', 1) ; % time t
z = msspoly('z', 2) ; % states z = (x,y)
x = z(1) ;
y = z(2) ;

% define the domains of the FRS variables with semi-algebraic sets (i.e.,
% the sets are defined by where the polynomials hT, hZ, and hZ0 are
% positive on their respective domains)
h_T = t * (1 - t) ;
h_Z = (z - [-1;-1]) .* ([1;1] - z) ;

% create a circular footprint for the initial condition
h_Z0 = 1 - ((x - initial_x)/(footprint/distance_scale)).^2 + ...
         - ((y - initial_y)/(footprint/distance_scale)).^2 ;

% create trajectory-producing model
scale = (time_scale/distance_scale) ;
f = scale*[v_des - w_des*(y - initial_y) ;
    + w_des*(x - initial_x)] ;

%% create the spotless program to solve
% note that this code is also in the compute_FRS.m file in the RTD repo,
% but we're writing a simplified form here for demonstration purposes; the
% program we are writing down is labeled (D) in the paper at this link:
% https://arxiv.org/abs/1809.06746

% initialize program and indeterminate variables
prog = spotsosprog;
prog = prog.withIndeterminate(t) ;
prog = prog.withIndeterminate(z) ;

% create monomials for decision variable polynomials; v is like a Lyapunov
% function and w is like an indicator function on the FRS
V_mon = monomials([t;z], 0:degree) ;
I_mon = monomials(z, 0:degree) ;

% create the decision variable polynomials
[prog, V, ~] = prog.newFreePoly(V_mon) ;
[prog, I, I_coeff] = prog.newFreePoly(I_mon) ;

% create variables for the constraints of the program (D)
t0 = 0 ;
V0 = subs(V,t,t0) ;
dVdt = diff(V,t) ;
dVdz = diff(V,z) ;
LfV = dVdt + dVdz*f ;

% define each constraint from program (D), ignoring the ones containing q
% (which represents the tracking error)

% -LfV > 0 on T x Z
prog = sosOnK(prog, -LfV, [t;z], [h_T; h_Z], degree) ;

% V(t,.) + I > 1 on T x Z
prog = sosOnK(prog, V + I - 1, [t;z], [h_T; h_Z], degree) ;

% I > 0 on Z
prog = sosOnK(prog, I, z, h_Z, degree) ;

% -V(0,.) > 0 on Z0
prog = sosOnK(prog, -V0, z, h_Z0, degree) ;

% define the cost function (the integral of I over the domain Z)
int_Z = boxMoments(z, [-1;-1], [1;1]) ; % this integrates I over Z
obj = int_Z(I_mon)' * I_coeff ; 

%% solve for FRS
options = spot_sdp_default_options() ;
options.verbose = 1 ;
options.domain_size = 1;
options.solveroptions = [];

start_tic = tic ;
sol = prog.minimize(obj, @spot_mosek, options) ;
end_time = toc(start_tic) ;

%% extract results
I_sol = sol.eval(I) ;

%% get actual trajectory of agent
% create the initial condition
z0 = [0;0;0;initial_speed] ; % (x,y,h,v)

% create the desired trajectory
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% create the braking trajectory
[T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

% move the robot
A.reset(z0)
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;

%% visualize FRS indicator function polynomial
figure(1) ; clf ; axis equal ; hold on ;

% plot the initial condition
offset = -distance_scale*[initial_x;initial_y] ;
plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b',...
    'Offset',offset,'Scale',distance_scale)

% plot the FRS
plot_2D_msspoly_contour(I_sol,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3],...
    'Offset',offset,'Scale',distance_scale)

% plot the desired trajectory
plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)

% plot the agent
plot(A)

% labeling
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)