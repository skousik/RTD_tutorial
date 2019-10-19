%% description
% This script runs a simulation with the TurtleBot in the simulator
% framework, using RRT* to plan online.
%
% Author: Shreyas Kousik
% Created: 19 Oct 2019
%
%% user parameters
% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 7 ;
bounds = [-4,4,-2,2] ;
goal_radius = 0.5 ;

% planner
buffer = 0.1 ; % m
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;

% simulation
verbose_level = 5 ;

%% automated from here
A = turtlebot_agent ;

% this is needed to the agent to track the RRT* output
A.LLC.yaw_gain = 10 ;

P = turtlebot_RRT_star_planner('verbose',verbose_level,'buffer',0.1,...
    't_plan',t_plan,'t_move',t_move) ;

W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',0.25,...
                     'verbose',verbose_level,'goal_radius',goal_radius,...
                     'obstacle_size_bounds',obstacle_size_bounds) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',30,'max_sim_iterations',60) ;

%% run simulation
S.run() ;