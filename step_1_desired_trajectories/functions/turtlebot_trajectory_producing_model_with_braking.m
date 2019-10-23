function zd = turtlebot_trajectory_producing_model_with_braking(t,z,T_in,U_in,t_plan,t_stop)
% zd = turtlebot_trajectory_producing_model(t,z,T_in,U_in,t_plan,t_stop)
%
% Output the dynamics of the simplified Dubins' car that we use as a
% trajectory-producing model for the TurtleBot. This can get plugged in to
% ode45, for example.
%
% This version includes braking after a given time t_plan for a duration
% t_stop.
%
% Author: Shreyas Kousik
% Created: 23 Oct 2019
% Udpated: -

    if nargin < 5
        t_plan = 0.5 ;
        if nargin < 6
            t_stop = 2.61 ;
        end
    end

    % extract states
    x = z(1) ;
    y = z(2) ;

    % get inputs
    u = match_trajectories(t,T_in,U_in) ;
    w_des = u(1) ;
    v_des = u(2) ;

    % compute dynamics
    if t < t_plan
        zd = [v_des - w_des*y ;
            w_des*x ;
            w_des ] ;
    else
        t_dec = (t_stop - t)/(t_stop - t_plan) ;
        zd = t_dec.*[v_des - w_des*y ;
            w_des*x ;
            w_des ] ;
    end
end