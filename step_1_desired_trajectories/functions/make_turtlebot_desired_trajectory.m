function [T,U,Z] = make_turtlebot_desired_trajectory(t_f,w_des,v_des)
% [T,U,Z] = make_turtlebot_desired_trajectory(t_f,w_des,v_des)
%
% Create a Dubins path as a full-state trajectory for the TurtleBot.
%
% The inputs are:
%   t_f      planning time horizon
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
% Updated: 30 May 2019

    % set up timing
    t_sample = 0.01 ;
    T = unique([0:t_sample:t_f,t_f]);
    N_t = length(T) ;
    
    % get inputs for desired trajectories
    w_traj = w_des*ones(1,N_t) ;
    v_traj = v_des*ones(1,N_t) ;
    U_in = [w_traj ; v_traj] ;

    % compute desired trajectory
    z0 = zeros(3,1) ;
    [~,Z] = ode45(@(t,z) turtlebot_trajectory_producing_model(t,z,T,U_in),T,z0) ;

    % append velocity to (x,y,h) trajectory to make it a full-state
    % trajectory for the turtlebot
    Z = [Z' ; v_traj] ;
    
    % compute inputs for robot
    a_traj = [diff(v_traj)./t_sample, 0] ;
    U = [w_traj ; a_traj] ;
end