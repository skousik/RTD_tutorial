function [c, gc] = turtlebot_cost_for_fmincon(k,FRS,wp_local,start_tic,timeout)
% [c, gc] = turtlebot_cost_for_fmincon(k,FRS,wp_local)
% [c, gc] = turtlebot_cost_for_fmincon(k,FRS,wp_local,start_tic,timeout)
%
% Evaluate the cost and cost gradient for use with fmincon when performing
% trajectory optimization for the TurtleBot.
%
% Author: Shreyas Kousik
% Created: 31 May 2019
%
% See also: turtlebot_cost, turtlebot_cost_grad,
% turtlebot_nonlcon_for_fmincon

    % evaluate cost and gradient
    c = turtlebot_cost(k(1),k(2),FRS.w_max,FRS.v_range(2),wp_local(1),wp_local(2)) ;
    gc = turtlebot_cost_grad(k(1),k(2),FRS.w_max,FRS.v_range(2),wp_local(1),wp_local(2)) ;
    
    % perform timeout check
    if nargin > 3 && toc(start_tic) > timeout
        error('Timed out while evaluating cost function!')
    end
end