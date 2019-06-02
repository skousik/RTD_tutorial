function [r,a] = compute_turtlebot_point_spacings(R,b)
% [r,a] = compute_turtlebot_point_spacings(R,b)
%
% Compute the point spacing r and arc point spacing a, as per Example 67
% on page 35 of the Big Paper (https://arxiv.org/abs/1809.06746). These are
% used to discretize obstacles for online planning.
%
% Author: Shreyas Kousik
% Created: 30 May 2019

    % first, make sure the buffer is valid
    bbar = R ;

    if b > bbar
        disp('Resizing obstacle buffer to be valid!')
        b = bbar - 0.01 ;
    end

    % now compute the point spacing r and arc point spacing a
    theta_1 = acos((R-b)/R) ;
    theta_2 = acos(b/(2*R)) ;
    r = 2*R*sin(theta_1) ;
    
    if nargout > 1
        a = 2*b*sin(theta_2) ;
    end
end