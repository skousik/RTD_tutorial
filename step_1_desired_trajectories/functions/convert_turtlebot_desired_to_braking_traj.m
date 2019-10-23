function [T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T,U,Z)
% [T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T,U,Z)
%
% Given a desired trajectory without braking, transform it into one with
% braking by commanding a hard stop and linearly decreasing the yaw rate.
% This causes the TurtleBot to brake "along" the original desired
% trajectory.
%
% Author: Shreyas Kousik
% Created: 12 May 2019
% Updated: 22 Oct 2019

    % initialize the output as a copy of the initial trajectory
    T_brk = T ;
    Z_brk = Z ;

    % get the part of the trajectory after t_plan as the braking part of
    % the trajectory
    t_log = T >= t_plan ;
    
    % reinterpolate the timing to be long enough for stopping
    N_t = sum(t_log) ;
    t_new = linspace(t_plan,t_plan+t_stop,N_t) ;
    T_brk(t_log) = t_new ;
    
    % create a new angular speed input that quadratically decreases to zero
    % while the robot brakes to a stop
    w_brk = U(1,:) ;
    quadratic_decrease = (linspace(1,0,N_t).^2) ;
    w_brk(t_log) = quadratic_decrease.*w_brk(t_log) ;
    
    % set the desired velocity to zero (NOTE this means the trajectory is
    % no longer dynamically feasible!)
    Z_brk(4,t_log) = 0 ;
    
    % ALTERNATIVE BRAKING: set the desired velocity to decrease quadratically
    % Z_brk(4,t_log) = quadratic_decrease.*Z_brk(4,t_log) ;
    
    % set the new acceleration input based on the average acceleration over
    % the duration t_stop
    a_avg = -max(Z_brk(4,:))./t_stop ;
    a_brk = U(2,:) ;
    a_brk(t_log) = a_avg ;
    
    % create braking feedforward input
    U_brk = [w_brk ; a_brk] ;
end