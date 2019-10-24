function s = get_turtlebot_braking_scale_at_time(t_cur,t_plan,t_stop,pwr)
% s = get_turtlebot_braking_scale_at_time(t_cur,t_plan,t_stop,pwr)
%
% Return the scalar value s \in [0,1] by which to scale a trajectory's
% desired angular and linear speeds to cause the turtlebot to brake to a
% stop over the time interval [t_plan, t_plan + t_stop].
%
% Author: Shreyas Kousik
% Created: 23 Oct 2019
% Updated: -

    if nargin < 2
        t_plan = 0.5 ;
        t_stop = 2.61 ;
        pwr = 4 ;
    end

    s = ((t_stop - t_cur + t_plan)./(t_stop)).^pwr ;
end