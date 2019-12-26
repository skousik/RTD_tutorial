classdef turtlebot_RRT_planner < planner
    % Class: turtlebot_RRT_planner
    %
    % This class is a "pass through" for the RRT high-level planner. It runs
    % an RRT* algorithm until a timeout is reached, then converts the resulting
    % best path into a trajectory to pass to the agent to execute.
    %
    % Author: Shreyas Kousik
    % Created: 31 Oct 2019
    % Updated: 3 Dec 2019
    
    properties
        lookahead_distance = 1 ;
        desired_speed = 1 ;
        plot_HLP_flag = false ;
        
        % HLP type
        HLP_type = 'RRT' ; % 'rrt' or 'rrt*' or 'connect' or 'connect*'
        
        % method for initializing the tree; if 'iter' is chosen, then the
        % RRT runs for P.t_plan every planning iteration, but if 'once' is
        % chosen, then the entire tree is grown when "setup" is called
        % (before the receding-horizon planning loop begins)
        initialize_tree_mode = 'iter' ;
        
        % method for growing the tree within the RRT_HLP; choose 'new' or
        % 'seed' or 'keep'; see the grow_tree_mode property in RRT_HLP
        HLP_grow_tree_mode = 'seed' ;
        
        % if P.initialize_tree_mode is 'once' then we pick an amount of time
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
            switch lower(P.HLP_type)
                case 'rrt'
                    P.HLP = RRT_HLP() ;
                case 'rrt*'
                    P.HLP = RRT_star_HLP() ;
                case 'connect'
                    P.HLP = RRT_connect_HLP() ;
                case 'connect*'
                    P.HLP = RRT_star_connect_HLP() ;
                otherwise
                    error('Invalid high level planner type!')
            end
            
            % set up plot data
            P.plot_data.best_path = [] ;
            P.plot_data.obstacles = [] ;
        end
        
        %% setup
        function setup(P,agent_info,world_info)
            P.vdisp('Setting up high-level planner',4)
            
            P.HLP.setup(agent_info,world_info) ;
            
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            P.HLP.timeout = P.t_plan ;
            P.HLP.grow_tree_mode = P.HLP_grow_tree_mode ;
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
            
            % initialize current plan
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
            
            % grow the tree if necessary
            if strcmpi(P.initialize_tree_mode,'once')
                P.vdisp('Growing RRT!',2)
                
                % set the RRT timeout
                P.HLP.timeout = P.grow_tree_once_timeout ;
                
                % FOR DEBUGGING
                % P.HLP.plot_while_growing_tree_flag = true ;
                
                % get obstacles
                O = P.process_obstacles(world_info) ;
                
                % reassign the obstacles to be buffered for the RRT
                world_info.obstacles = O ;
                
                % grow the tree
                exit_flag = P.HLP.grow_tree(agent_info,world_info) ;
                
                if exit_flag > 0
                    P.vdisp('Tree grown successfully!',3)
                    [T,U,Z] = P.process_tree(agent_info) ;
                    
                    % add some time to the beginning of the plan to deal
                    % with incrementing
                    T = [-P.t_plan, T] ;
                    U = [U(:,1), U] ;
                    Z = [Z(:,1), Z] ;
                    
                    % set the current plan
                    P.current_plan.T = T ;
                    P.current_plan.U = U ;
                    P.current_plan.Z = Z ;
                else
                    error('Tree growth failed!')
                end
            end
            
            % set up info object
            P.info = struct('plan',[],'obstacles',[],'agent_time',[]) ;
        end
        
        %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            % get obstacles
            O = P.process_obstacles(world_info) ;
            
            % reassign the obstacles to be buffered for the RRT
            world_info.obstacles = O ;
            
            % run RRT
            if strcmpi(P.initialize_tree_mode,'iter')
                P.vdisp('Growing tree',5)
                exit_flag = P.HLP.grow_tree(agent_info,world_info) ;
                
                if exit_flag > 0
                    P.vdisp('Plan found successfully',5)
                    [T,U,Z] = P.process_tree(agent_info) ;
                else
                    P.vdisp('No new plan found')
                    [T,U,Z] = P.increment_plan(agent_info) ;
                end
            else
                P.vdisp('Using existing tree',5) ;
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
            
            % make sure the nodes are unique
            X = unique(X','rows','stable')' ;
            
            % convert X to a trajectory by assuming that we traverse it at
            % the given max speed
            s = P.desired_speed ;
            d = dist_polyline_cumulative(X) ;
            T = d./s ;
            
            % if the tree growth mode is using a seed, then shift the time
            % and trajectory by the agent's current time
            if strcmpi(P.HLP.grow_tree_mode,'seed') || strcmpi(P.HLP.grow_tree_mode,'keep')
                t_cur = agent_info.time(end) ;
                T_log = T >= t_cur ;
                T_shift = T(T_log) ;
                
                if T_shift(1) > t_cur
                    T_shift = [t_cur, T_shift] ;
                end
                X = match_trajectories(T_shift,T,X) ;
                
                T = T_shift - t_cur ;
            end
            
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
            hc = hold_switch() ;
            
            % plot RRT nodes
            if P.plot_HLP_flag
                plot(P.HLP)
            end
            
            % plot current plan
            if ~isempty(P.current_plan.Z)
                plot_object(P,P.current_plan.Z,'best_path','--','Color',[0.7 0.5 0.2]) ;
            end
            
            % plot obstacles
            if ~isempty(P.current_obstacles)
                plot_object(P,P.current_obstacles,'obstacles','r:') ;
            end
            
            hold_switch(hc) ;
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
                if ~isempty(Z)
                    plot_object(P,Z,'best_path','--','Color',[0.7 0.5 0.2]) ;
                end
                
                % plot obstacles
                O = P.info.obstacles{idx} ;
                if ~isempty(O)
                    plot_object(P,O,'obstacles','r:') ;
                end
            end
        end
    end
end