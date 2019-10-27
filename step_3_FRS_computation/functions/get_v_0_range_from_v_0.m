function v_0_range = get_v_0_range_from_v_0(v_0,v_0_min,v_0_max)
% v_0_range = get_v_0_range_from_v_0(v_0)
% v_0_range = get_v_0_range_from_v_0(v_0,v_0_min,v_0_max)
%
% Given a value of v_0, return the v_0 range that contains v_0 (and
% corresponds to a particular FRS for the turtlebot)
%
% Author: Shreyas Kousik
% Created: 26 Oct 2019
% Updated: -
    
    if nargin < 2
        v_0_min = [0.0 0.5 1.0] ;
    end

    if nargin < 3
        v_0_max = [0.5 1.0 1.5] ;
    end
    
    % figure out which interval the given v_0 is in
    idxs = (v_0 >= v_0_min) & (v_0 <= v_0_max) ;
    idx = find(idxs,1,'last') ;
    
    % return the corresponding v_0 range
    v_0_range = nan(1,2) ;
    v_0_range(1) = v_0_min(idx) ;
    v_0_range(2) = v_0_max(idx) ;
end