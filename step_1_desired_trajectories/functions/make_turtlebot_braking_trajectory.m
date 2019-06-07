function [T,U,Z] = make_turtlebot_braking_trajectory(t_plan,t_f,t_stop,w_des,v_des)
% [T,U,Z] = make_turtlebot_braking_trajectory(t_plan,t_f,t_stop,w_des,v_des)
%
% Create a Dubins path with braking as a full-state trajectory for the
% TurtleBot.
%
% The inputs are:
%   t_plan   planning timeout
%   t_f      planning time horizon
%   t_stop   duration required for robot to come to a stop
%   w_des    desired yaw rate
%   v_des    desired speed
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (yaw rate and acceleration) as 2-by-N array
%   Z        desired trajectory (x,y,h,v) as a 4-by-N array
%
% Author: Shreyas Kousik
% Created: 12 May 2019
% Updated: 7 June 2019

    [T,U,Z] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;
    [T,U,Z] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T,U,Z) ;
end