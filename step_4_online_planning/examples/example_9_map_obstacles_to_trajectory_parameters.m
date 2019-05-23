%% description
% In this script
%
%% user parameters
obstacle_location = [1.25 ; 0] ; % (x,y)
obstacle_scale = 1.0 ;
obstacle_buffer = 0.05 ; % m

%% automated from here
% load FRS
FRS = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;

% create turtlebot
A = turtlebot_agent ;
A.reset()

% create obstacle
O = make_random_polygon(5,obstacle_location,obstacle_scale) ;

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
% for now, just don't use the arc point spacing
O_buf = buffer_polygon_obstacles(O,b,2) ;
O_r = interpolate_polyline_with_spacing(O_buf,r) ;

% put the obstacles into the FRS frame
set_dx = FRS.initial_x ;
set_dy = FRS.initial_y ;
O_FRS = world_to_local(A.state(1:3),O_r,FRS.initial_x,FRS.initial_y,FRS.distance_scale) ;

% filter out points that are too far away
O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;

%% put points in param space
% get FRS polynomial and variables
I = FRS.FRS_polynomial ;
k = FRS.k ;
z = FRS.z ;

% evaluate FRS polynomial on obstacle points
I_k = msubs(I,z,O_FRS) ;

%% plotting
figure(1) ; clf ;

% Z space
subplot(1,2,2) ; hold on ; axis equal ; set(gca,'FontSize',15)

% plot robot
plot(A)

% plot buffered obstacle
patch(O_buf(1,:),O_buf(2,:),[1 0.5 0.6])

% plot actual obstacle
patch(O(1,:),O(2,:),[1 0.7 0.8])

% plot discretized obstacle
plot(O_r(1,:),O_r(2,:),'.','Color',[0.5 0.1 0.1],'MarkerSize',15)

% K space
subplot(1,2,1) ; hold on ; axis equal ; set(gca,'FontSize',15)
for idx = 1
    I_idx = I_k(idx) ;
    plot_2D_msspoly_contour(I_idx,k,1,'FillColor',[1 0.7 0.8])
end

