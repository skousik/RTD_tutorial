function [T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T,U,Z)
% [T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T,U,Z)
%
% Given a desired trajectory without braking, transform it into one with
% braking by commanding a hard stop and linearly decreasing the yaw rate.
% This causes the TurtleBot to brake "along" the original desired
% trajectory.

    % initialize the output as a copy of the initial trajectory
    T_brk = T ;
    Z_brk = Z ;

    % get the part of the trajectory after t_plan as the braking part of
    % the trajectory
    t_log = T >= t_plan ;
    
    % set the desired velocity to zero (NOTE this means the trajectory is
    % no longer dynamically feasible!)
    Z_brk(4,t_log) = 0 ;
    
    % reinterpolate the timing to be long enough for stopping
    N_t = sum(t_log) ;
    t_new = linspace(t_plan,t_plan+t_stop,N_t) ;
    T_brk(t_log) = t_new ;
    
    % create a new angular speed input that linearly decreases towards zero
    % while the robot brakes to a stop
    w_brk = U(1,:) ;
    w_brk(t_log) = (linspace(1,0,N_t).^2).*w_brk(t_log) ;
    
    % back out the new acceleration input from the new position trajectory
    d = vecnorm(Z(1:2,:)) ;
    v_brk = [diff(d)./diff(T_brk), 0] ;
    a_brk = [diff(v_brk)./diff(T_brk), 0] ;
    
    % create braking nominal input
    U_brk = [w_brk ; a_brk] ;
end