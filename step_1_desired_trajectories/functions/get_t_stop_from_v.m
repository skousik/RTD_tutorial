function t_stop = get_t_stop_from_v(v,delta_v,v_max,accel)
% t_stop = get_t_stop_from_v(v,delta_v,v_max,accel)
%
% Given an initial speed v, compute the amount of time needed for the
% Turtlebot to brake to a stop, given a maximum allowable commanded change
% in speed (delta_v), a maximum speed (v_max), and a maximum acceleration
% (accel).
%
% Author: Shreyas Kousik
% Created: 24 Oct 2019
% Updated: 25 Oct 2019
    if nargin < 2
        delta_v = 0.25 ;
    end
    
    if nargin < 3
        v_max = 1.5 ;
    end
    
    if nargin < 4
        accel = 2 ;
    end
    
    t_stop = min(v + delta_v, v_max) ./ accel ;
end