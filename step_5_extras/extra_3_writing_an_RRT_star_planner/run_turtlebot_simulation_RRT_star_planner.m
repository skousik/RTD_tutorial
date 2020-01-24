%% description
% This script runs a simulation with the TurtleBot in the simulator
% framework, using RRT to plan online.
%
% Author: Shreyas Kousik
% Created: 19 Oct 2019
% Updated: 24 Jan 2019
%
%% user parameters
% agent
desired_speed = 0.75 ; % m/s
sensor_radius = 2.5 ; % make this larger if initialize_tree_mode is 'once'

% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 7 ;
bounds = [-4,4,-2,2] ;
goal_radius = 0.5 ;
world_buffer = 0.25 ; % this is used to make start and goal locations that are not too close to the robot or boundary

% planner
buffer = 0.3 ; % [m] distance used to buffer obstacles for planning, must be larger than the agent's footprint
t_plan = 1.0 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;
initialize_tree_mode = 'iter' ; % 'iter' or 'once'
HLP_grow_tree_mode = 'new' ; % 'new' or 'seed' or 'keep' (only matters if using 'iter' above)
grow_tree_once_timeout = 10 ;
HLP_type = 'rrt' ; % 'rrt' or 'rrt*' or 'connect' or 'connect*'
new_node_growth_distance = 0.5 ;

% plotting
plot_HLP_flag = true ;
plot_tree_growth_flag = true ;

% simulation
verbose_level = 5 ;
max_sim_iterations = 100 ;
max_sim_time = 100 ;

%% automated from here
A = turtlebot_agent('sensor_radius',sensor_radius) ;

% this is needed for the agent to track the RRT* output
A.LLC.gains.yaw = 10 ;
A.LLC.lookahead_time = 0.1 ;

P = turtlebot_RRT_planner('verbose',verbose_level,'buffer',buffer,...
    't_plan',t_plan,'t_move',t_move,'desired_speed',desired_speed,...
    'HLP_type',HLP_type,...
    'initialize_tree_mode',initialize_tree_mode,...
    'grow_tree_once_timeout',grow_tree_once_timeout,...
    'HLP_grow_tree_mode',HLP_grow_tree_mode) ;

P.HLP.new_node_growth_distance = new_node_growth_distance ;
P.plot_HLP_flag = plot_HLP_flag ;
P.HLP.plot_while_growing_tree_flag = plot_tree_growth_flag ;

W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',world_buffer,...
                     'verbose',verbose_level,'goal_radius',goal_radius,...
                     'obstacle_size_bounds',obstacle_size_bounds) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iterations) ;


%% set up plot
if strcmpi(initialize_tree_mode,'once')
    figure(1) ; clf ; axis equal ; hold on ;
    plot(W)
end
          
%% run simulation
S.run() ;