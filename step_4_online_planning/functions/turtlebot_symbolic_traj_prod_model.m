function zd = turtlebot_symbolic_traj_prod_model(z,v_des,w_des)
% zd = turtlebot_symbolic_traj_prod_model(z,v_des,w_des)
%
% Output the dynamics of the TurtleBot's trajectory-producing model
% that can be used with symbolic variables.
%
% Author: Shreyas Kousik
% Created: 30 May 2019

    % extract states
    x = z(1) ;
    y = z(2) ;
    
    % compute dynamics
    zd = [v_des - w_des*y ;
          w_des*x ] ;
end