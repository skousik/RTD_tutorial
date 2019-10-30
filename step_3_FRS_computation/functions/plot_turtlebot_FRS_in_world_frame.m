function h = plot_turtlebot_FRS_in_world_frame(FRS,k,pose,varargin)
% plot_turtlebot_FRS_in_world_frame(FRS,k,pose,'property1',value1,'property2',value2,...)
%
% Given an FRS structure output by compute_turtlebot_FRS.m, a choice of k,
% and a turtlebot pose (x,y,h), plot the FRS corresponding to k at that
% pose.
%
% Note that k should be a 2-by-1, with values between -1 and 1.
%
% This function plots a patch, so it also takes in all the usual patch
% property/value arguments, and will return a patch plot handle if an
% output is specified.
%
% Author: Shreyas Kousik and Sean Vaskov
% Created: 21 Oct 2019
% Updated: 29 Oct 2019
%
% See also: get_2D_contour_points, FRS_to_world

    % evaluate FRS at k
    I = FRS.FRS_polynomial ;
    I = msubs(I,FRS.k,k) ;
    
    % generate patch info
    [~,FRS_patch_info,N_vertices] = get_2D_contour_points(I,FRS.z,1,'Bounds',0.9) ;
    
    if ~isempty(FRS_patch_info)
        % get the FRS properties needed to plot in world frame
        x0 = FRS.initial_x ; y0 = FRS.initial_y ;
        if isfield(FRS,'distance_scale_x') && isfield(FRS,'distance_scale_y')
            Dx = FRS.distance_scale_x ;
            Dy = FRS.distance_scale_y ;
        else
            Dx = FRS.distance_scale ;
            Dy = FRS.distance_scale ;
        end
        
        % find the patch info with the largest number of vertices, because
        % that is probably the one that matters
        [~,idx] = max(N_vertices) ;
        
        % get the data for the patch into the world frame
        V_idx = FRS_patch_info(idx).Vertices' ;
        FRS_patch_info(idx).Vertices = FRS_to_world(V_idx,pose,x0,y0,Dx,Dy)' ;
        
        % plot the patch
        h = patch(FRS_patch_info(idx),varargin{:}) ;
    end
    
    if nargout < 1
        clear h
    end
end