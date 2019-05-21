function visualize_turtlebot_FRS(FRS_info,k_eval,use_FRS_scaling)
% visualize_turtlebot_FRS(FRS_info,k_eval,use_FRS_scaling)
%
% Visualize the FRS contained in FRS_info by plotting the contour for the
% input k_eval. If the input use_FRS_scaling is omitted or true, then the
% FRS is plotted in its scaled and shifted coordinates. Otherwise, the
% scaling and shifting is undone, and the FRS is plotted with the initial
% condition at the origin.
%
% Author: Shreyas Kousik
% Date: 20 May 2019

    % parse inputs
    if nargin < 3
        use_FRS_scaling = true ;
    end

    % extract variables
    z = FRS_info.z ;
    k = FRS_info.k ;

    % extract FRS polynomial
    I = FRS_info.FRS_polynomial ;

    % extract initial condition set
    h_Z0 = FRS_info.h_Z0 ;

    % extract scaling info
    distance_scale = FRS_info.distance_scale ;
    initial_x = FRS_info.initial_x ;
    initial_y = FRS_info.initial_y ;

    % evaluate I(.,k)
    I_z = msubs(I,k,k_eval) ;

    % set up for plot
    figure(1) ; clf ; hold on ;

    if use_FRS_scaling
        % plot the FRS
        plot_2D_msspoly_contour(I_z,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3])

        % plot the initial condition
        plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b')
    else
        % plot the FRS
        plot_2D_msspoly_contour(I_sol,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3],...
            'Offset',-[initial_x;initial_y],'Scale',distance_scale)

        % plot the initial condition
        plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b',...
            'Offset',-[initial_x;initial_y],'Scale',distance_scale)
    end
end