%% description
% In this script, we create a cost function and its analytic derivative for
% the TurtleBot. The cost function is simply to minimize distance to a
% waypoint while tracking a trajectory. We'll save the cost and gradient as
% MATLAB functions.
%
% Author: Shreyas Kousik
% Created: 30 May 2019
%
%% user parameters
% numerical integration parameters
dt_int = 0.05 ; % s
discount_factor = [0.9 ; 1.0] ; % compensates for integration error in x/y

% desired speed and yaw rate for validation
w_val = 0.8; % rad/s
v_val = 1.5 ; % m/s

% max speed and yaw rate for validation
w_max_val = 1.0 ;
v_max_val = 1.5 ;

% save cost and gradient flag
save_flag = false ;

%% automated from here
% load timing
load('turtlebot_timing.mat')

% create symbolic variables to use in the cost
syms w_max v_max k_1 k_2 real

% get the desired speed and yaw rate
w_des = w_max * k_1 ;
v_des = (v_max/2)*k_2 + (v_max/2) ;

%% numerically compute endpoint of desired traj
disp('Numerically estimating desired traj. endpoint')

% we'll express the endpoint of the desired trajectory at t_plan using
% numerical integration - first, set up the initial condition
z = [0;0] ; % (x,y)

tmax = 300 ; % timer just in case
tcur = tic ;
for tidx = 0:dt_int:t_plan
    dzdt = turtlebot_symbolic_traj_prod_model(z,v_des,w_des) ;
    z = z + discount_factor.*dt_int.*dzdt ;

    if toc(tcur) > tmax
        break
    end
end
toc(tcur)

% make a function out of the endpoint to use for validation
z_fn = matlabFunction(z,'Vars',[k_1,k_2,w_max,v_max]) ;

%% make the cost and analytic gradient
% create waypoints
syms x_des y_des real

% make the cost
cost = sum(([x_des;y_des] - z).^2) ;

% get the analytic gradient
cost_grad = [diff(cost,k_1), diff(cost,k_2)] ;

%% make a function out of the cost and gradient
if save_flag
    disp('Saving cost and cost gradient')
    matlabFunction(cost,'Vars',[k_1,k_2,w_max,v_max,x_des,y_des],'File','turtlebot_cost')
    matlabFunction(cost_grad,'Vars',[k_1,k_2,w_max,v_max,x_des,y_des],'File','turtlebot_cost_grad')
else
    disp('Not saving cost and cost gradient')
end

%% time the cost and gradient
try
    test_input = num2cell(2*rand(1,6)) ;
    disp('Timing cost function')
    cost_time = timeit(@() turtlebot_cost(test_input{:})) ;
    disp(['    Average exec time: ', num2str(cost_time),' s'])
    
    disp('Timing gradient function')
    cost_grad_time = timeit(@() turtlebot_cost_grad(test_input{:})) ;
    disp(['    Average exec time: ', num2str(cost_grad_time),' s'])
catch
    disp('Cost and gradient not found on path!')
end

%% validation
% get the actual endpoint at t_plan
[~,~,Z] = make_turtlebot_desired_trajectory(t_plan,w_val,v_val) ;

z_true = Z(1:2,end) ;

% sub in k_1, and k_2 to z
k_1_val = w_val / w_max_val ;
k_2_val = (v_val - v_max_val/2)*(2/v_max_val) ;
z_val = double(subs(z,[k_1,k_2,w_max,v_max], [k_1_val,k_2_val,w_max_val,v_max_val])) ;

%% plotting
figure(1) ; clf ; hold on ; axis equal ; grid on

% plot desired trajectory
plot(Z(1,:),Z(2,:),'b')

% plot approximate endpoint
plot(z_val(1),z_val(2),'bo') ;

% cleanup
legend('desired traj.','numerical endpoint')

set(gca,'FontSize',15)
xlabel('x')
ylabel('y')