function zd = turtlebot_trajectory_producing_model(t,z,T_in,U_in)
% zd = turtlebot_trajectory_producing_model(t,z,T_in,U_in)
%
% Output the dynamics of the simplified Dubins' car that we use as a
% trajectory-producing model for the TurtleBot. This can get plugged in to
% ode45, for example. See make_turtlebot_desired_trajectory for how it is
% used.
%
% Author: Shreyas Kousik
% Created: 12 May 2019
% Updated: 28 Oct 2019

    % get heading state
    h = z(3) ;

    % get inputs
    u = match_trajectories(t,T_in,U_in) ;
    w_des = u(1) ;
    v_des = u(2) ;

    % compute dynamics
    zd = [v_des*cos(h) ;
          v_des*sin(h) ;
          w_des ] ;
    end