classdef turtlebot_RRT_planner < planner
    % Class: turtlebot_RRT_planner
    %
    % This class is a "pass through" for the RRT high-level planner. It runs
    % an RRT* algorithm until a timeout is reached, then converts the resulting
    % best path into a trajectory to pass to the agent to execute.
    %
    % Author: Shreyas Kousik
    % Created: 31 Oct 2019
    % Updated: 31 Oct 2019
    
    properties
        lookahead_distance = 1 ;
        desired_speed = 1 ;
        plot_HLP_flag = false ;
        
        % method for growing the tree; if 'iter' is chosen, then the RRT
        % runs for P.t_plan every planning iteration, but if 'once' is
        % chosen, then the entire tree is grown in the first iteration
        grow_tree_mode = 'iter' ;
        
        % if P.grow_tree_mode is 'once' then we pick an amount of time
        % allowed to grow the tree
        grow_tree_once_timeout = 5 ; % s
    end
    
    %% methods
    methods
        %% constructor
        function P = turtlebot_RRT_planner(varargin)
            % set up default properties and construct planner
            P = parse_args(P,'buffer',0,...
                'name','TurtleBot RRT Planner',...
                varargin{:}) ;
            
            % set high-level planner
            P.HLP = RRT_HLP() ;
            
            % set up plot data
            P.plot_data.best_path = [] ;
            P.plot_data.obstacles = [] ;
        end
        
        %% setup
        function setup(P,agent_info,world_info)
            P.vdisp('Setting up high-level planner',4)
            
            P.HLP.nodes = agent_info.position(:,end) ;
            P.HLP.nodes_parent = 0 ;
            P.HLP.goal = world_info.goal ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            P.HLP.timeout = P.t_plan ;
            P.HLP.grow_tree_mode = 'new' ;
            P.HLP.plot_tree_flag = false ;
            P.HLP.plot_best_path_flag = false ;
            P.HLP.plot_waypoint_flag = false ;
            
            % set up bounds
            P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;
            
            try
                P.HLP.bounds = P.bounds ;
            catch
                P.vdisp('High level planner has no bounds field.',9) ;
            end
            
            % grow the tree if necessary
            if strcmpi(P.grow_tree_mode,'once')
                % set the RRT timeout
                P.HLP.timeout = P.grow_tree_once_timeout ;
                
                % get obstacles
                O = P.process_obstacles(world_info) ;
                
                % grow the tree
                exit_flag = P.HLP.grow_tree(agent_info,O) ;
                
                if exit_flag > 0
                    P.vdisp('Tree grown successfully!',3)
                else
                    error('Tree growth failed!')
                end
            end
            
            % initialize current plan
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
            
            % set up info object
            P.info = struct('plan',[],'obstacles',[],'agent_time',[]) ;
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            % get obstacles
            O = P.process_obstacles(world_info) ;
            
            % run RRT
            if strcmpi(P.grow_tree_mode,'iter')
                P.vdisp('Growing tree',5)
                exit_flag = P.HLP.grow_tree(agent_info,O) ;
            else
                P.vdisp('Using existing tree',5) ;
                exit_flag = 1 ;
            end
            
            if exit_flag > 0
                P.vdisp('Plan found successfully',5)
                [T,U,Z] = P.process_tree(agent_info) ;
            else
                P.vdisp('No new plan found')
                [T,U,Z] = P.increment_plan(agent_info) ;
            end
            
            % save current best path and obstacles
            P.vdisp('Updating current plan',6)
            P.current_plan.T = T ;
            P.current_plan.U = U ;
            P.current_plan.Z = Z ;
            P.current_obstacles = O ;
            
            % save info
            P.vdisp('Saving info from this iteration',7)
            P.info.plan = [P.info.plan, {Z}] ;
            P.info.obstacles = [P.info.obstacles, {O}] ;
            P.info.agent_time = [P.info.agent_time, agent_info.time(end)] ;
        end
        
        function O_out = process_obstacles(P,world_info)
            O = world_info.obstacles ;
            
            if ~isempty(O)
                O_out = buffer_polygon_obstacles(O,P.buffer) ;
            else
                O_out = O ;
            end
        end
        
        function [T,U,Z] = process_tree(P,agent_info)
            % if the RRT is successful, return the best path
            X = P.HLP.best_path ;
            
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
            
            % generate inputs (these are dummy inputs, since we'll just
            % track the trajectory with feedback)
            U = zeros(2,N) ;
        end
        
        function [T,U,Z] = increment_plan(P,agent_info)
            % get the previously-found plan
            T = P.current_plan.T ;
            U = P.current_plan.U ;
            Z = P.current_plan.Z ;
            
            % try to increment the previous plan
            if any(T >= 2*P.t_move)
                P.vdisp('Incrementing previous plan',5)
                T_log = T >= P.t_move ;
                T_interp = unique([P.t_move, T(T_log)],'stable') ;
                [U,Z] = match_trajectories(T_interp,T,U,T,Z) ;
                T = T_interp - P.t_move ;
            else
                % otherwise, just give back the agent's current state
                P.vdisp('Staying stopped',5)
                T = [0, 2*P.t_move] ;
                U = zeros(2) ;
                Z = [repmat(agent_info.state(1:3,end),1,2) ; zeros(1,2)] ;
            end
        end
        
        %% plot
        function plot(P,~)
            % plot current plan
            P.plot_object(P.current_plan.Z,'best_path','--','Color',[0.7 0.5 0.2]) ;
            
            % plot obstacles
            P.plot_object(P.current_obstacles,'obstacles','r:') ;
            
            % plot RRT nodes
            if P.plot_HLP_flag
                plot(P.HLP)
            end
        end
        
        function plot_at_time(P,t)
            if nargin < 2
                t = 0 ;
            end
            
            % get the info index corresponding to the input t
            T = P.info.agent_time ;
            idx = find(t >= T,1,'last') ;
            
            if ~isempty(idx)
                % plot best path
                Z = P.info.plan{idx} ;
                P.plot_object(Z,'best_path','--','Color',[0.7 0.5 0.2]) ;
                
                % plot obstacles
                O = P.info.obstacles{idx} ;
                P.plot_object(O,'obstacles','r:') ;
            end
        end
        
        function plot_object(P,obj,plot_data_fieldname,varargin)
            if ~isempty(obj)
                if check_if_plot_is_available(P,plot_data_fieldname)
                    P.plot_data.(plot_data_fieldname).XData = obj(1,:) ;
                    P.plot_data.(plot_data_fieldname).YData = obj(2,:) ;
                else
                    obj = plot(obj(1,:),obj(2,:),varargin{:}) ;
                    P.plot_data.(plot_data_fieldname) = obj ;
                end
            end
        end
    end
end