classdef turtlebot_RRT_star_planner < planner
% Class: turtlebot_RRT_star_planner
%
% This class is a "pass through" for the RRT* high-level planner. It runs
% an RRT* algorithm until a timeout is reached, then converts the resulting
% best path into a trajectory to pass to the agent to execute.
    
    properties
        current_path = [] ;
        lookahead_distance = 1 ;
        desired_speed = 1 ;
    end
    
    %% methods
    methods
        %% constructor
        function P = turtlebot_RRT_star_planner(varargin)
            % set up default properties and construct planer
            P = parse_args(P,'buffer',0,varargin{:}) ;
            
            % set high-level planner
            P.HLP = RRT_star_HLP() ;
            
            % set up plot data
            P.plot_data.best_path = [] ;
            P.plot_data.obstacles = [] ;
        end
        
        %% setup
        function setup(P,agent_info,world_info)
            P.vdisp('Setting up high-level planner',4)
            
            P.HLP.goal = world_info.goal ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
            % set up bounds
            P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;
            
            try
                P.HLP.bounds = P.bounds ;
            catch
                P.vdisp('High level planner has no bounds field.',9) ;
            end
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            % process obstacles
            O = world_info.obstacles ;
            O_buf = buffer_polygon_obstacles(O,P.buffer,2) ;
            
            % run RRT star
            P.HLP.plan_path(agent_info,O_buf,P.lookahead_distance) ;
            
            % get the current best path out
            X = P.HLP.plan ;
            
            % if the path is empty or too short, make sure it's long enough
            switch size(X,2)
                case 0
                    X = repmat(agent_info.state(1:2,end),1,2) ;
                case 1
                    X = [X X] ;
            end
            
            % the RRT* HLP outputs a path from end to beginning, so flip it
            X = X(:,end:-1:1) ;
                
            % convert X to a trajectory by assuming that we traverse it at
            % the given max speed
            s = P.desired_speed ;
            d = dist_polyline_cumulative(X) ;
            T = d./s ;
            
            % generate headings along trajectory
            dX = diff(X,[],2) ;
            h = atan2(dX(2,:),dX(1,:)) ;
            h = [h, h(end)] ;
            
            % generate trajectory and input
            N = size(X,2) ;
            Z = [X ; h ; s.*ones(1,N)] ;
            
            % generate inputs
            U = zeros(2,N) ;
            
            % save current best path and obstacles
            P.current_path = X ;
            P.current_obstacles = O_buf ;
        end
        
        %% plot
        function plot(P,~)
            % plot current best path
            X = P.current_path ;
            
            if ~isempty(X)
                if check_if_plot_is_available(P,'best_path')
                    P.plot_data.best_path.XData = X(1,:) ;
                    P.plot_data.best_path.YData = X(2,:) ;
                else
                    data = plot_path(X,'b--') ;
                    P.plot_data.best_path = data ;
                end
            end
            
            % plot obstacles
            O = P.current_obstacles ;
            
            if ~isempty(O)
                if check_if_plot_is_available(P,'obstacles')
                    P.plot_data.obstacles.XData = O(1,:) ;
                    P.plot_data.obstacles.YData = O(2,:) ;
                else
                    data = plot(O(1,:),O(2,:),'r:') ;
                    P.plot_data.obstacles = data ;
                end
            end
        end
        
        function plot_at_time(P,t)
            plot(P) ;
        end
    end
end