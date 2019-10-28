function FRS = get_FRS_from_v_0(v_0,degree)
% FRS = get_FRS_from_v_0(v_0)
%
% Load the fastest feasible FRS for the given initial speed v_0
%
% Author: Shreyas Kousik
% Created: 28 Oct 2019
% Updated: -

    if nargin < 2
        degree = 10 ;
    end

    v_0_range = get_v_0_range_from_v_0(v_0) ;

    switch v_0_range(1)
        case 0.0
            FRS = load(['turtlebot_FRS_deg_',num2str(degree),'_v_0_0.0_to_0.5.mat']) ;
        case 0.5
            FRS = load(['turtlebot_FRS_deg_',num2str(degree),'_v_0_0.5_to_1.0.mat']) ;
        case 1.0
            FRS = load(['turtlebot_FRS_deg_',num2str(degree),'_v_0_1.0_to_1.5.mat']) ;
        otherwise
            error('Hey! You picked an invalid speed range for the tutorial!')
    end
end