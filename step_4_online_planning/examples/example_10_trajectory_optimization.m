%% description
% In this script, we first create constraints on the trajectory parameters
% given a random polygonal obstacle. Then, we optimize over the trajectory
% parameters to find a feasible, safe trajectory.
%
% Author: Shreyas Kousik
% Created: 30 May 2019
% Updated: 31 May 2019
%
%% user parameters
% robot initial condition (note that x, y, and h are 0 for this example)
initial_speed = 0.2 ; % m/s

% robot desired location
x_des = 0.75 ;
y_des = 0.5 ;

% obstacle
obstacle_location = [1.5 ; 0] ; % (x,y)
obstacle_scale = 1.0 ;
N_vertices = 5 ;
obstacle_buffer = 0.05 ; % m

%% automated from here
% load FRS
disp('Loading fastest feasible FRS')
if initial_speed >= 1.0 && initial_speed <= 1.5
    FRS = load('turtlebot_FRS_deg_10_v0_1.0_to_1.5.mat') ;
elseif initial_speed >= 0.5
    FRS = load('turtlebot_FRS_deg_10_v0_0.5_to_1.0.mat') ;
elseif initial_speed >= 0.0
    FRS = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;
else
    error('Please pick an initial speed between 0.0 and 1.5 m/s')
end
    
% create turtlebot
A = turtlebot_agent ;
A.reset([0;0;0;initial_speed])

% create obstacle
O = make_random_polygon(N_vertices,obstacle_location,obstacle_scale) ;

%% create cost function
% create waypoint from desired location
wp = [x_des; y_des] ;

% transform waypoint to FRS coordinates
wp_local = FRS_to_world(wp,A.state(:,end),FRS.initial_x,FRS.initial_y,FRS.distance_scale) ;

% use waypoint to make cost function
cost = @(k) turtlebot_cost_for_fmincon(k,FRS,wp_local) ;

%% create constraint function
% discretize obstacle
point_spacing = compute_turtlebot_point_spacings(A.footprint,obstacle_buffer) ;
[O_FRS, O_buf] = compute_turtlebot_discretized_obs(O,...
                    A.state(:,end),obstacle_buffer,point_spacing,FRS) ;

% get FRS polynomial and variables
FRS_msspoly = FRS.FRS_polynomial ;
k = FRS.k ;
z = FRS.z ;

% decompose polynomial into simplified structure (this speeds up the
% evaluation of the polynomial on obstacle points)
FRS_poly = get_FRS_polynomial_structure(FRS_msspoly,z,k) ;

% swap the speed and steer parameters for visualization purposes
FRS_poly_viz = subs(FRS_msspoly,k,[k(2);k(1)]) ;

% evaluate the FRS polynomial structure input on the obstacle points to get
% the list of constraint polynomials
cons_poly = evaluate_FRS_polynomial_on_obstacle_points(FRS_poly,O_FRS) ;

% get the gradient of the constraint polynomials
cons_poly_grad = get_constraint_polynomial_gradient(cons_poly) ;

% create nonlinear constraint function for fmincon
nonlcon = @(k) turtlebot_nonlcon_for_fmincon(k,cons_poly,cons_poly_grad) ;

% create bounds for yaw rate
k_1_bounds = [-1,1] ;

% create bounds for speed
v_0 = initial_speed ;
v_max = FRS.v_range(2) ;
v_des_lo = max(v_0 - FRS.delta_v, FRS.v_range(1)) ;
v_des_hi = min(v_0 + FRS.delta_v, FRS.v_range(2)) ;
k_2_lo = (v_des_lo - v_max/2)*(2/v_max) ;
k_2_hi = (v_des_hi - v_max/2)*(2/v_max) ;
k_2_bounds = [k_2_lo, k_2_hi] ;

% combine bounds
k_bounds = [k_1_bounds ; k_2_bounds] ;

%% run trajectory optimization
% create initial guess
initial_guess = zeros(2,1) ;

% create optimization options
options =  optimoptions('fmincon',...
                'MaxFunctionEvaluations',1e5,...
                'MaxIterations',1e5,...
                'OptimalityTolerance',1e-3',...
                'CheckGradients',false,...
                'FiniteDifferenceType','central',...
                'Diagnostics','off',...
                'SpecifyConstraintGradient',true,...
                'SpecifyObjectiveGradient',true);

% call fmincon
[k_opt,~,exitflag] = fmincon(cost,...
                            initial_guess,...
                            [],[],... % linear inequality constraints
                            [],[],... % linear equality constraints
                            k_bounds(:,1),... % lower bounds
                            k_bounds(:,2),... % upper bounds
                            nonlcon,...
                            options) ;

%% get contour of trajopt output
% if ~isempty(k_opt)
%     I_z_opt = msubs(FRS_msspoly,k,k_opt) ;
%     C_FRS = get_2D_contour_points(I_z_opt,z,1) ;
%     C_world = FRS_to_world(C_FRS,A.state(:,end),x0,y0,D) ;
% end

%% plot actual xy space
% figure(1) ; clf ;
% 
% subplot(1,3,3) ; hold on ; axis equal ; set(gca,'FontSize',15)
% 
% % plot robot
% plot(A)
% 
% % plot buffered obstacle
% patch(O_buf(1,:),O_buf(2,:),[1 0.5 0.6])
% 
% % plot actual obstacle
% patch(O(1,:),O(2,:),[1 0.7 0.8])
% 
% % plot discretized obstacle
% plot(O_pts(1,:),O_pts(2,:),'.','Color',[0.5 0.1 0.1],'MarkerSize',15)
% 
% % plot test value of k
% if ~isempty(k_test)
%     I_z_test = msubs(FRS_msspoly,k,k_test) ;
%     plot(C_world(1,:),C_world(2,:),'Color',[0.3 0.8 0.5],'LineWidth',1.5)
% end
% 
% % labeling
% title('Global Frame')
% xlabel('x [m]')
% ylabel('y [m]')

%% plot FRS frame
% h_Z0 = FRS.h_Z0 ;
% 
% subplot(1,3,2) ; hold on ; axis equal ; grid on ; set(gca,'FontSize',15)
% 
% % plot initial condition set
% plot_2D_msspoly_contour(h_Z0,z,0,'Color',[0 0 1],'LineWidth',1.5)
% 
% % plot FRS obstacles
% plot(O_FRS(1,:),O_FRS(2,:),'.','Color',[0.5 0.1 0.1],'MarkerSize',15)
% 
% % plot test value of k
% if ~isempty(k_test)
%     plot(C_FRS(1,:),C_FRS(2,:),'Color',[0.3 0.8 0.5],'LineWidth',1.5)
% end
% 
% % labeling
% title('FRS Frame')
% xlabel('x (scaled)')
% ylabel('y (scaled)')

%% plot traj param space
% subplot(1,3,1) ; hold on ; axis equal ; set(gca,'FontSize',15)
% 
% % plot obstacle point contours
% for idx = 1:length(I_k)
%     I_idx = I_k(idx) ;
%     plot_2D_msspoly_contour(I_idx,k,1,'FillColor',[1 0.5 0.6])
% end
% 
% % plot k_test
% if ~isempty(k_test)
%     plot(k_test(1),k_test(2),'.','Color',[0.3 0.8 0.5],'MarkerSize',15)
%     plot(k_test(1),k_test(2),'ko','MarkerSize',6)
% end
% 
% % label
% title('Traj Params')
% xlabel('speed param')
% ylabel('yaw rate param')