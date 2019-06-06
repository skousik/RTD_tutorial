function [O_FRS, O_buf_out, O_pts_out] = compute_turtlebot_discretized_obs(O_world,turtlebot_pose,b,r,FRS)
% O_FRS = compute_turtlebot_discretized_obs(O_world,turtlebot_pose,b,r,FRS)
%
% Take obstacle points defined in a global coordinate frame and transform
% them into the scaled, shifted FRS frame for the TurtleBot.
%
% Author: Shreyas Kousik
% Created: 30 May 2019

    % for now, just don't use the arc point spacing by setting the miter limit
    % to 2 in buffer_polygon_obstacles
    O_buf = buffer_polygon_obstacles(O_world,b,2) ;
    O_pts = interpolate_polyline_with_spacing(O_buf,r) ;

    % put the obstacles into the FRS frame
    x0 = FRS.initial_x ;
    y0 = FRS.initial_y ;
    D = FRS.distance_scale ;
    O_FRS = world_to_FRS(O_pts,turtlebot_pose,x0,y0,D) ;

    % filter out points that are too far away to be reached
    O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;
    
    % return the buffered obstacle as well
    if nargout > 1
        O_buf_out = O_buf ;
        if nargout > 2
            O_pts_out = O_pts ;
        end
    end
end