%% description
% This script demonstrates the RRT high-level planner variants in a static
% box world instance.
%
% Author: Shreyas Kousik
% Created: 30 Oct 2019
% Updated: 6 Nov 2019

clear ;

%% user parameters
% agent
sensor_radius = 10 ;

% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 12 ;
bounds = [-4,4,-2,2] ;
goal_radius = 0.5 ;

% high-level planner
HLP_type = 'rrt*' ; % choose 'RRT' or 'RRT*' or 'connect' or 'connect*'
timeout = 20 ;

% simulation
verbose_level = 5 ;
max_sim_time = 1000 ;
max_sim_iterations = 1000 ;

% plotting
plot_tree_flag = true ;
plot_best_path_flag = true ;
plot_while_growing_tree_flag = true ;

%% automated from here
% create dummy agent (the HLP needs agent_info)
A = agent() ;

switch lower(HLP_type)
    case 'rrt'
        HLP = RRT_HLP('timeout',timeout,'plot_best_path_flag',plot_best_path_flag,...
            'plot_tree_flag',plot_tree_flag,'plot_while_growing_tree_flag',plot_while_growing_tree_flag) ;
    case 'connect'
        HLP = RRT_connect_HLP('timeout',timeout,'plot_best_path_flag',plot_best_path_flag,...
            'plot_tree_flag',plot_tree_flag,'plot_while_growing_tree_flag',plot_while_growing_tree_flag) ;
    case 'rrt*'
        HLP = RRT_star_HLP('timeout',timeout,'plot_best_path_flag',plot_best_path_flag,...
            'plot_tree_flag',plot_tree_flag,'plot_while_growing_tree_flag',plot_while_growing_tree_flag) ;
    case 'connect*'
        HLP = RRT_star_connect_HLP('timeout',timeout,'plot_best_path_flag',plot_best_path_flag,...
            'plot_tree_flag',plot_tree_flag,'plot_while_growing_tree_flag',plot_while_growing_tree_flag) ;
    otherwise
        error('Invalid HLP type!')
end


W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',0.25,...
    'verbose',verbose_level,'goal_radius',goal_radius,...
    'obstacle_size_bounds',obstacle_size_bounds) ;

A.reset(W.start)
A.sensor_radius = sensor_radius ;

% setup to run RRT*
HLP.grow_tree_mode = 'keep' ;
HLP.bounds = W.bounds ;
HLP.goal = W.goal ;
AI = A.get_agent_info() ;
WI = W.get_world_info(AI) ;
HLP.setup(AI,WI) ;

%% run RRT*
if plot_while_growing_tree_flag
    figure(1) ; clf ; hold on ; axis equal
    plot(W)
end

HLP.grow_tree(AI,WI) ;

%% plotting
figure(1) ; clf ; hold on ; axis equal
plot(W)
plot(HLP)