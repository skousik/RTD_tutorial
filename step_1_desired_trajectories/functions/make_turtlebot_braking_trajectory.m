function [T,U,Z] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des)
% [T,U,Z] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des)
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
% Updated: 24 Oct 2019

    % set up timing
    t_sample = 0.01 ;
    t_total = t_plan + t_stop ;
    T = unique([0:t_sample:t_total,t_total]);

    % create braking traj. for w and v
    t_log = T >= t_plan ;
    braking_scale_power = 1 ;
    scale = ones(size(T)) ;
    scale(t_log) = get_turtlebot_braking_scale_at_time(T(t_log),t_plan,t_stop,braking_scale_power) ;

    % get inputs for desired trajectories
    w_traj = w_des.*scale ;
    v_traj = v_des.*scale ;
    U_in = [w_traj ; v_traj] ;

    % compute desired trajectory
    z0 = zeros(3,1) ;
    [~,Z] = ode45(@(t,z) turtlebot_trajectory_producing_model(t,z,T,U_in),T,z0) ;

    % append velocity to (x,y,h) trajectory to make it a full-state
    % trajectory for the turtlebot
    Z = [Z' ; v_traj] ;

    % compute inputs for robot
    a_traj = diff(v_traj)./t_sample ;
    U = [w_traj ; [a_traj, a_traj(end)]] ;
end