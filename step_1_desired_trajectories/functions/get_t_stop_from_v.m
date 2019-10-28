function t_stop = get_t_stop_from_v(v,accel)
% t_stop = get_t_stop_from_v(v,accel)
%
% Given a current speed v, compute the amount of time needed for the
% Turtlebot to brake to a stop, given a maximum allowable acceleration
% (accel).
%
% Author: Shreyas Kousik
% Created: 24 Oct 2019
% Updated: 27 Oct 2019
    
    if nargin < 2
        accel = 2 ;
    end
    
    t_stop = v ./ accel ;
end