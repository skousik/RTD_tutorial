%% description
% This script runs a simulation with the TurtleBot in the simulator
% framework, using RTD to plan online.
%
% Author: Shreyas Kousik
% Created: 6 June 2019
% Updated: 30 Oct 2019
%
%% user parameters
% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 12 ;
bounds = [-4,4,-2,2] ;
goal_radius = 0.5 ;

% planner
buffer = 0.01 ; % m
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;
FRS_degree = 12 ;
plot_FRS_flag = true ;
plot_HLP_flag = true ;

% simulation
verbose_level = 5 ;
max_sim_time = 1000 ;
max_sim_iterations = 1000 ;

%% automated from here
A = turtlebot_agent ;

P = turtlebot_RTD_planner_static('verbose',verbose_level,'buffer',buffer,...
                                 't_plan',t_plan,'t_move',t_move,...
                                 'FRS_degree',FRS_degree,...
                                 'plot_FRS_flag',plot_FRS_flag,...
                                 'plot_HLP_flag',plot_HLP_flag) ;

P.HLP = RRT_star_HLP() ;

W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',0.25,...
                     'verbose',verbose_level,'goal_radius',goal_radius,...
                     'obstacle_size_bounds',obstacle_size_bounds) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',max_sim_time,...
              'max_sim_iterations',max_sim_iterations) ;

%% run simulation
S.run() ;