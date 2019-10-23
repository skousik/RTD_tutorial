function [T,U,Z] = make_turtlebot_desired_trajectory_with_braking(t_plan,t_stop,w_des,v_des)
% [T,U,Z] = make_turtlebot_desired_trajectory_with_braking(t_plan,t_f,t_stop,w_des,v_des)
%
% Create a Dubins path as a full-state trajectory for the TurtleBot, with
% braking beginning at t_plan until t_stop.
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
% Created: 23 Oct 2019
% Updated: -

    % set up timing
    t_sample = 0.01 ;
    t_f = t_plan + t_stop ;
    T = unique([0:t_sample:t_f,t_f]);
    N_t = length(T) ;
    
    % get inputs for desired trajectories
    w_traj = w_des*ones(1,N_t) ;
    v_traj = v_des*ones(1,N_t) ;
    U_in = [w_traj ; v_traj] ;

    % compute desired trajectory
    z0 = zeros(3,1) ;
    [~,Z] = ode45(@(t,z) turtlebot_trajectory_producing_model_with_braking(t,z,T,U_in,t_plan,t_stop),T,z0) ;

    % append velocity to (x,y,h) trajectory to make it a full-state
    % trajectory for the turtlebot
    Z = [Z' ; v_traj] ;
    
    % compute inputs for robot
    a_traj = [diff(v_traj)./t_sample, 0] ;
    U = [w_traj ; a_traj] ;
end