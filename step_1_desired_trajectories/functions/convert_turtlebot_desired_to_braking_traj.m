function [T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T,U,Z,braking_type)
% [T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T,U,Z)
% [T_brk,U_brk,Z_brk] = convert_turtlebot_desired_to_braking_traj(t_plan,t_stop,T,U,Z,braking_type)
%
% Given a desired trajectory without braking, transform it into one with
% braking by commanding a hard stop and linearly decreasing the yaw rate.
% This causes the TurtleBot to brake "along" the original desired
% trajectory.
%
% Author: Shreyas Kousik
% Created: 12 May 2019
% Updated: 23 Oct 2019

    if nargin < 6
        braking_type = -1 ;
    end

    % initialize the output as a copy of the initial trajectory
    T_brk = T ;
    Z_brk = Z ;

    % get the part of the trajectory after t_plan as the braking part of
    % the trajectory
    t_log = T >= t_plan ;
    
    % reinterpolate the timing to be long enough for stopping
    N_t = sum(t_log) ;
    dt = t_stop ./ N_t ;
    t_new = linspace(t_plan + dt,t_plan+t_stop,N_t) ;
    T_brk(t_log) = t_new ;
    
    % set up a quadratic decrease to be used with the braking flags
    quadratic_decrease = (linspace(1,0,N_t).^2) ;
    
    % set up state values to be adjusted for braking
    w_brk = U(1,:) ;
    v_brk = Z_brk(4,:) ;
    
    switch braking_type
        case -1    
            % create a new angular speed input that quadratically decreases to zero
            % while the robot brakes to a stop
            w_brk(t_log) = quadratic_decrease.*w_brk(t_log) ;

            % set the desired velocity to zero (NOTE this means the trajectory is
            % no longer dynamically feasible!)
            v_brk(t_log) = 0 ;
        case 0
            w_brk(t_log) = 0 ;
            v_brk(t_log) = 0 ;
        otherwise
            % set the angular and linear velocities to decrease oct-tically
            w_brk(t_log) = (quadratic_decrease.^braking_type).*w_brk(t_log) ;
            v_brk(t_log) = (quadratic_decrease.^braking_type).*v_brk(t_log) ;
    end
    
    % set the braking velocity desired state
    Z_brk(4,:) = v_brk ;
        
    % set the new acceleration input based on the average acceleration over
    % the duration t_stop
    a_avg = -max(Z_brk(4,:))./t_stop ;
    a_brk = U(2,:) ;
    a_brk(t_log) = a_avg ;
    
    % create braking feedforward input
    U_brk = [w_brk ; a_brk] ;
end