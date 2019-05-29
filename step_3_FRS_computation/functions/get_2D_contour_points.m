function P = get_2D_contour_points(p,x,l,varargin)
% P = get_2D_contour_points(p,x,l,'keyword1',value1,'keyword2',value2,...)
%
% Given an msspoly p in the 2D variable x, return a 2-by-N polyline of the
% set where p(x) = l, i.e. the l-level set of p.
%
% In addition to p, x, and l, additional inputs can be passed in as keyword
% arguments. The keywords allowed are:
%   Offset          a 2D position offset of the origin in the frame used
%                   for computing p (e.g., if p represents a semi-algebraic
%                   set)
%
%   Pose            a 3-by-1 in SE(2) where the contour should be centered
%                   at and rotated by
%
%   Scale           a scalar representing how much 2D space is scaled in
%                   the frame used for computing p
%
%   Bounds          [xlo,xhi,ylo,yhi] lower and upper bounds of the space
%                   containing the contour (for now, these bounds are
%                   applied to both x and y axes) in the frame that p is
%                   computed in
%
%   GridDensity     the number of points in x and y to use for computing
%                   the contour (default is 100)
%
%
% Authors: Shreyas Kousik and Sean Vaskov
% Date: 29 May 2019
%
    %% parse input arguments
    if nargin < 3
        l = 0 ;
    end

    % create default inputs
    Offset = [0;0] ;
    Pose = [0;0;0] ;
    Scale = 1 ;
    Bounds = [-1, 1, -1, 1] ;
    GridDensity = 100 ;

    for idx = 1:2:length(varargin)
        switch varargin{idx}
            case 'Offset'
                Offset = varargin{idx+1} ;
            case 'Pose'
                Pose = varargin{idx+1} ;
            case 'Scale'
                Scale = varargin{idx+1} ;
            case 'Bounds'
                Bounds = varargin{idx+1} ;
            case 'GridDensity'
                GridDensity = varargin{idx+1} ;
        end
    end

    %% create contour points
    % make 2D grid for plotting
    x_vec = linspace(Bounds(1),Bounds(2),GridDensity) ;
    y_vec = linspace(Bounds(3),Bounds(4),GridDensity) ;
    [X1,X2] = meshgrid(x_vec, y_vec) ;
    X = [X1(:) X2(:)]' ;
    F = reshape(full(msubs(p,x(1:2), X)),GridDensity,GridDensity) ;

    % create contour matrix
    P_raw = contourc(x_vec,y_vec,F,[l l]);

    % find the columns in the contour matrix that separate the individual
    % contours - the first entry in these columns will be the input l, and
    % the second entry will be an integer
    idxs = find((P_raw(1,:) == l) & (mod(P_raw(2,:),1) == 0));

    % for each contour, extract the vertices
    P = [] ;
    for idx = idxs
        P = [P, nan(2,1),...
            P_raw(:, (idx+1):(idx + P_raw(2,idx)))] ;
    end
    
    % remove column of nans at end of P
    if isnan(P(1,end))
        P = P(:,1:end-1) ;
    end

    % scale, shift, and rotate the contour points
    Position = Pose(1:2) ;
    Rotation = Pose(3) ;

    Px = P(1,:) ;
    Py = P(2,:) ;

    x0 = Offset(1) ;
    y0 = Offset(2) ;

    x_shift = (Scale*Px - x0)*cos(Rotation) - sin(Rotation)*(Scale*Py - y0) ;
    y_shift = (Scale*Py - y0)*cos(Rotation) + sin(Rotation)*(Scale*Px - x0) ;

    % create the final output
    P = Position + [x_shift ; y_shift];
end