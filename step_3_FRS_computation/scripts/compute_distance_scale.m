%% description
% This script computes the distance scaling needed to compute an FRS for
% the entire TurtleBot FRS.
%
% Author: Shreyas Kousik
% Date: 20 May 2019
%
%% user parameters
% uncomment one of the following lines to load the relevant error function
% data; we'll compute the FRS for that error function

% filename = 'turtlebot_error_functions_v0_0.0_to_0.5.mat' ;
% filename = 'turtlebot_error_functions_v0_0.5_to_1.0.mat' ;
filename = 'turtlebot_error_functions_v0_1.0_to_1.5.mat' ;

%% automated from here
% load timing info
load('turtlebot_timing.mat')

% load error info
load(filename)

% set max speed
max_speed = 1.5 ; % we'll just hard code this for now

% create the agent
A = turtlebot_agent ;

% create the range of command inputs and initial conditions
w_range = [0, w_max] ;
v_range = [-delta_v, delta_v] ;
v0_range = [v0_min, v0_max] ;

% initialize time and distance scales
time_scale = t_f ;
distance_scale_x = 0 ;
distance_scale_y = 0 ;

% iterate through yaw rates, speeds, and initial conditions
for v0 = v0_range
    
    for w_des = w_range
        v_des_range = v0 + v_range ;
        
        for v_des_cur = v_des_range
            % make sure we're not above the max speed
            v_des = bound_values(v_des_cur,[0, max_speed]) ;
            
            % reset the agent
            z0 = [0;0;0;v0] ;
            A.reset(z0) ;
            
            % create the trajectory to execute
            [T_brk,U_brk,Z_brk] = make_turtlebot_braking_trajectory(t_plan,t_f,t_stop,w_des,v_des) ;
            
            % run the agent
            A.move(T_brk(end),T_brk,U_brk,Z_brk) ;
            
            % find the distance traveled
            distance_scale_x = max(distance_scale_x,max(A.state(A.position_indices(1),:))) ;
            distance_scale_y = max(distance_scale_y,2*max(A.state(A.position_indices(2),:))) ; 
        end
    end
end

% find the distance scale
distance_scale = max(distance_scale_x,distance_scale_y) + 2*A.footprint ;
disp(['Distance scale found: ',num2str(distance_scale,'%0.2f')])

% set the (x0,y0) offset so that x0 lies at -0.5, since the dynamics are
% scaled to travel a distance of around 1
initial_x = -0.5 ;
initial_y = 0 ;

%% save output
filename = ['turtlebot_FRS_scaling_v0_',...
            num2str(v0_min,'%0.1f'),'_to_',...
            num2str(v0_max,'%0.1f'),'.mat'] ;
save(filename,'time_scale','distance_scale',...
    'distance_scale_x','distance_scale_y',...
    'max_speed',...
    'initial_x','initial_y') ;
