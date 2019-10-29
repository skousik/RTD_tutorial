%% description
% In this script, we create a random polygonal obstacle, then use the FRS
% computed in Step 3 to map the obstacle to the trajectory parameter space.
%
% Author: Shreyas Kousik
% Created: 29 May 2019
% Updated: 29 Oct 2019
%
%% user parameters
obstacle_location = [1 ; 0] ; % (x,y)
obstacle_scale = 1.0 ;
N_vertices = 5 ;
obstacle_buffer = 0.05 ; % m

%% automated from here
% load FRS
FRS = load('turtlebot_FRS_deg_10_v_0_0.0_to_0.5.mat') ;

% create turtlebot
A = turtlebot_agent ;
A.reset()

% create obstacle
O = make_random_polygon(N_vertices,obstacle_location,obstacle_scale) ;

%% compute discretization values
% first, make sure the buffer is valid
b = obstacle_buffer ;
R = A.footprint ; % footprint radius
bbar = R ;

if b > bbar
    disp('Resizing obstacle buffer to be valid!')
    b = bbar - 0.01 ;
end

% now compute the point spacing r and arc point spacing a, as per Example
% 67 on page 35 of the Big Paper (https://arxiv.org/abs/1809.06746)
theta_1 = acos((R-b)/R) ;
theta_2 = acos(b/(2*R)) ;
r = 2*R*sin(theta_1) ;
a = 2*b*sin(theta_2) ;

%% discretize obstacle
% for now, just don't use the arc point spacing by setting the miter limit
% to 2 in buffer_polygon_obstacles
O_buf = buffer_polygon_obstacles(O,b,2) ;
O_pts = interpolate_polyline_with_spacing(O_buf,r) ;

% put the obstacles into the FRS frame
x0 = FRS.initial_x ;
y0 = FRS.initial_y ;
D = FRS.distance_scale ;
O_FRS = world_to_FRS(O_pts,A.state(:,end),x0,y0,D) ;

% filter out points that are too far away to be reached
O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;

%% put points in param space
% get FRS polynomial and variables
I = FRS.FRS_polynomial ;
k = FRS.k ;
z = FRS.z ;

% swap the speed and steer parameters for visualization purposes
I = subs(I,k,[k(2);k(1)]) ;

% evaluate FRS polynomial on obstacle points
I_k = msubs(I,z,O_FRS) ;

%% find a collision-free parameter
tic

% generate grid of k values
k_vec = linspace(-1,1) ;
[K1,K2] = meshgrid(k_vec,k_vec) ;
K_vals = [K1(:), K2(:)]' ;

% check each k value in the grid for collision with each of the obstacle
% points
K_log = true(1,length(K_vals)) ;
for idx = 1:length(I_k)
    I_log = msubs(I_k(idx),k,K_vals) ;
    K_log = K_log & (I_log < 1) ;
end

% eliminate the unsafe k values
K_vals = K_vals(:,K_log) ;

% pick a random feasible k value from the remaining set
if ~isempty(K_vals)
    disp('Feasible trajectory found!')
    k_test_idx = round(rand_range(1,size(K_vals,2))) ;
    k_test = K_vals(:,k_test_idx) ;
else
    disp('No feasible trajectories available!')
    k_test = [] ;
end

toc

%% get contour of collision-free parameter in K and Z
if ~isempty(k_test)
    I_z_test = msubs(I,k,k_test) ;
    C_FRS = get_2D_contour_points(I_z_test,z,1) ;
    C_world = FRS_to_world(C_FRS,A.state(:,end),x0,y0,D) ;
end

%% plot world frame
figure(1) ; clf ;

subplot(1,3,3) ; hold on ; axis equal ; set(gca,'FontSize',15)

% plot robot
plot(A)

% plot buffered obstacle
patch(O_buf(1,:),O_buf(2,:),[1 0.5 0.6])

% plot actual obstacle
patch(O(1,:),O(2,:),[1 0.7 0.8])

% plot discretized obstacle
plot(O_pts(1,:),O_pts(2,:),'.','Color',[0.5 0.1 0.1],'MarkerSize',15)

% plot test value of k
if ~isempty(k_test)
    I_z_test = msubs(I,k,k_test) ;
    plot(C_world(1,:),C_world(2,:),'Color',[0.3 0.8 0.5],'LineWidth',1.5)
end

axis([-0.5 1.5 -1 1])

% labeling
title('World Frame')
xlabel('x [m]')
ylabel('y [m]')

%% plot FRS frame
h_Z0 = FRS.h_Z0 ;

subplot(1,3,2) ; hold on ; axis equal ; grid on ; set(gca,'FontSize',15)

% plot initial condition set
plot_2D_msspoly_contour(h_Z0,z,0,'Color',[0 0 1],'LineWidth',1.5)

% plot FRS obstacles
plot(O_FRS(1,:),O_FRS(2,:),'.','Color',[0.5 0.1 0.1],'MarkerSize',15)

% plot test value of k
if ~isempty(k_test)
    plot(C_FRS(1,:),C_FRS(2,:),'Color',[0.3 0.8 0.5],'LineWidth',1.5)
end

% labeling
title('FRS Frame')
xlabel('x (scaled)')
ylabel('y (scaled)')

%% plot traj param space
subplot(1,3,1) ; hold on ; axis equal ; set(gca,'FontSize',15)

% plot obstacle point contours
for idx = 1:length(I_k)
    I_idx = I_k(idx) ;
    plot_2D_msspoly_contour(I_idx,k,1,'FillColor',[1 0.5 0.6])
end

% plot k_test
if ~isempty(k_test)
    plot(k_test(1),k_test(2),'.','Color',[0.3 0.8 0.5],'MarkerSize',15)
    plot(k_test(1),k_test(2),'ko','MarkerSize',6)
end

% label
title('Traj Params')
xlabel('speed param')
ylabel('yaw rate param')