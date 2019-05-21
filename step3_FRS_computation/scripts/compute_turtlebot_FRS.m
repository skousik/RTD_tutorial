%% description
% This script computes a Forward-Reachable Set (FRS) for the TurtleBot. The
% user specifies 
%
% Author: Shreyas Kousik
% Date: 16 May 2019
%
%% user parameters
% include uncertainty or not
include_uncertainty = true ;

% uncomment one of the following lines to load the relevant error function:
load('turtlebot_error_functions_v0_0.0_to_0.5.mat')
% load('turtlebot_error_functions_v0_0.5_to_1.0.mat')
% load('turtlebot_error_functions_v0_1.0_to_1.5.mat')

%% automated from here
% In this first cell, we set up the information we need to construct the
% FRS problem. In particular, we load the timing information, and find the
% distance scaling required given the "worst case" behavior of the
% dynamics. We also get the geometry of the robot to set up the initial
% condition set for the FRS solver.

% load timing information
load('turtlebot_timing.mat')

% create the range of command inputs and initial conditions
w_range = [w_min, w_max] ;
v_range = [-delta_v, delta_v] ;

% figure out the "farthest" that the trajectory-tracking dynamics f + g can
% travel given the initial condition 