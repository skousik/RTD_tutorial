classdef turtlebot_RTD_planner_static < planner
% Class: turtlebot_RTD_planner_static < planner
%
% This class implements RTD for a Turtlebot in static environments. It does
% not inherit the generic RTD planner superclass, so that you can see how
% all the parts of a planner should be written in a simulator framework.
%
% Author: Shreyas Kousik
% Created: 6 Jun 2019
% Updated: 29 Oct 2019

    %% properties
    properties
    % inherited properties with default values (see planner.m)
        % name
        % bounds % world bounds plus planner-specific buffer
        % buffer % minimum amount to buffer obstacles, given by the world
        % HLP % high level planner
        % current_plan ;
        % current_obstacles ;
        % verbose = 0 ;
        % timeout = 1 ; % time allowed for "replan" function to execute
        % t_plan = 1 ; % same as timeout; just for notational purposes
        % t_move = 1 ;% amount of time the planner expects the agent to move
        % info % information structure to keep a log when planning
        % plot_data % data for current plot
        % plot_waypoints_flag = false ;
        
    % RTD-specific properties
        % FRS handling
        FRS % this is a cell array of loaded info from FRS .mat files
        FRS_degree = 10 ; % set to 4, 6, 8, 10, or 12
        FRS_polynomial_structure
        
        % obstacle handling
        point_spacing
        current_obstacles_raw
        current_obstacles_in_FRS_coords
        bounds_as_obstacle
        
        % plan handling
        current_waypoint
        lookahead_distance = 1.5 ;
        
        % plotting
        plot_obstacles_flag = true ;
        plot_FRS_flag = false ;
        plot_HLP_flag = false ;
    end
    
    %% methods
    methods
    %% constructor
        function P = turtlebot_RTD_planner_static(varargin)
            % P = turtlebot_RTD_planner_static(varargin)
            %
            % This constructs the RTD planner.
            
            % set default values of some properties for the superclass that
            % are relevant to this particular planner; note that other
            % properties are defined in the planner "setup" method
            name = 'TurtleBot RTD Planner' ;
            buffer = 1 ;
            HLP = straight_line_HLP() ; % default high level planner
            
            % parse the input arguments; these should be given in the
            % format 'property1', value1, 'property2', value2,...
            P = parse_args(P,'name',name,'buffer',buffer,'HLP',HLP,...
                           varargin{:}) ;
            
            % load FRS files
            FRS_data = cell(1,3) ;
            FRS_data{1} = get_FRS_from_v_0(0.0,P.FRS_degree) ;
            FRS_data{2} = get_FRS_from_v_0(0.5,P.FRS_degree) ;
            FRS_data{3} = get_FRS_from_v_0(1.0,P.FRS_degree) ;
            P.FRS = FRS_data ;
        end
        
    %% setup
        function setup(P,agent_info,world_info)
            % P.setup(agent_info, world_info)
            %
            % This is used to set stuff up before planning. For RTD, we use
            % it for the following:
            %   1. compute the obstacle discretization point spacing
            %   2. set up world boundaries as an obstacle
            %   3. give the high level planner the global goal info
            %   4. decompose the FRS polynomial into a usable form
            
            P.vdisp('Running setup',3)
            
        %% 1. compute point spacing
            P.vdisp('Computing point spacing',4)
        
            bbar = agent_info.footprint ;
            b = P.buffer ;
            
            if b >= bbar
                P.buffer = bbar - 0.001 ;
                P.vdisp('Reducing buffer to be feasible!',2)
            end
            
            P.point_spacing = compute_turtlebot_point_spacings(bbar,P.buffer) ;
            
        %% 2. set up world boundaries as an obstacle
            P.vdisp('Setting up world bounds as an obstacle',4)
        
            P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;
            
            % create world bounds as an obstacle; note this passes the
            % bounds in as a clockwise polyline, so everything outside of
            % the world bounds counts as inside the polyline if using
            % functions like inpolygon
            xlo = P.bounds(1) ; xhi = P.bounds(2) ;
            ylo = P.bounds(3) ; yhi = P.bounds(4) ;
            
            B = [xlo, xhi, xhi, xlo, xlo ;
                ylo, ylo, yhi, yhi, ylo] ;
            B = [B, nan(2,1), 1.01.*B(:,end:-1:1)] ;
            
            P.bounds_as_obstacle = B ;
            
        %% 3. set up high level planner
            P.vdisp('Setting up high-level planner',4)
            
            P.HLP.goal = world_info.goal ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
            % try giving the HLP bounds
            try
                P.HLP.bounds = P.bounds ;
            catch
                P.vdisp('High level planner has no bounds field.',9) ;
            end
            
            
        %% 4. process the FRS polynomial
            P.vdisp('Processing FRS polynomial',4)
            
            P.FRS_polynomial_structure = cell(1,3) ;
            
            for idx = 1:3
                I = P.FRS{idx}.FRS_polynomial - 1 ;
                z = P.FRS{idx}.z ;
                k = P.FRS{idx}.k ;
                P.FRS_polynomial_structure{idx} = get_FRS_polynomial_structure(I,z,k) ;
            end
            
        %% 5. initialize the current plan as empty
            P.vdisp('Initializing current plan',4)
            
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
            
        %% 6. clear plot data
            P.plot_data.obstacles = [] ;
            P.plot_data.waypoint = [] ;
            P.plot_data.FRS = [] ;
            
        %% 7. set up info structure to save replan dat
            I = struct('agent_time',[],'agent_state',[],...
                'k_opt_found',[],...
                'FRS_index',[],...
                'waypoint',[],...
                'obstacles',[],...
                'obstacles_in_world_frame',[],...
                'obstacles_in_FRS_frame',[]) ;
            P.info = I ;
        end
        
    %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            % [T,U,Z] = P.replan(agent_info,world_info)
            %
            % This is the core of the RTD planner. In this method, we
            % generate a new trajectory plan, or continue the old plan, by
            % using the FRS to identify unsafe plans, and then using
            % an optimization program to find a safe plan.
            
            P.vdisp('Planning!',3)
            
            % start a timer to enforce the planning timeout P.t_plan
            start_tic = tic ;
            
        %% 1. determine the current FRS based on the agent
            P.vdisp('Determining current FRS',4)

            agent_state = agent_info.state(:,end) ; % (x,y,h,v)
            v_cur = agent_state(4) ;
            
            % pick fastest FRS for current speed
            if v_cur >= 1.0
                current_FRS_index = 3 ;
            elseif v_cur >= 0.5
                current_FRS_index = 2 ;
            else
                current_FRS_index = 1 ;
            end
            
            FRS_cur = P.FRS{current_FRS_index} ;
            
        %% 2. process obstacles
            P.vdisp('Processing obstacles',4)
        
            O = world_info.obstacles ;
            
            % add world bounds as obstacle
            O = [O, nan(2,1), P.bounds_as_obstacle] ;
            
            % buffer and discretize obstacles
            [O_FRS, ~, O_pts] = compute_turtlebot_discretized_obs(O,...
                    agent_state,P.buffer,P.point_spacing,FRS_cur) ;
            
            % save obstacles
            P.current_obstacles_raw = O ; % input from the world
            P.current_obstacles = O_pts ; % buffered and discretized
            P.current_obstacles_in_FRS_coords = O_FRS ;
        
        %% 3. create the cost function for fmincon
            P.vdisp('Creating cost function',4)
            
            % make a waypoint
            z_goal = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            P.current_waypoint = z_goal ;
            
            % put waypoint into FRS frame to use for planning
            z_goal_local = world_to_local(agent_state(1:3),z_goal(1:2)) ;
            
            % create cost function
            cost = @(k) turtlebot_cost_for_fmincon(k,...
                            FRS_cur,z_goal_local,...
                            start_tic,P.t_plan) ;
        
        %% 4. create the constraints for fmincon
            P.vdisp('Creating constraints',4)
        
            % create nonlinear constraints from the obstacles
            if ~isempty(O_FRS)
                % remove NaNs
                O_log = isnan(O_FRS(1,:)) ;
                O_FRS = O_FRS(:,~O_log) ;
                
                % get FRS polynomial
                FRS_poly = P.FRS_polynomial_structure{current_FRS_index} ;

                % plug in to FRS polynomial
                cons_poly = evaluate_FRS_polynomial_on_obstacle_points(FRS_poly,O_FRS) ;

                % get the gradient of the constraint polynomials
                cons_poly_grad = get_constraint_polynomial_gradient(cons_poly) ;
                
                % create constraint function
                nonlcon = @(k) turtlebot_nonlcon_for_fmincon(k,...
                                  cons_poly,cons_poly_grad,...
                                  start_tic,P.t_plan) ;
            else
                % if there are no obstacles then we don't need to consider
                % any constraints
                nonlcon = [] ;
            end
            
            % create bounds for yaw rate
            k_1_bounds = [-1,1] ;

            % create bounds for speed
            v_max = FRS_cur.v_range(2) ;
            v_des_lo = max(v_cur - FRS_cur.delta_v, FRS_cur.v_range(1)) ;
            v_des_hi = min(v_cur + FRS_cur.delta_v, FRS_cur.v_range(2)) ;
            k_2_lo = (v_des_lo - v_max/2)*(2/v_max) ;
            k_2_hi = (v_des_hi - v_max/2)*(2/v_max) ;
            k_2_bounds = [k_2_lo, k_2_hi] ;

            % combine bounds
            k_bounds = [k_1_bounds ; k_2_bounds] ;
            
        %% 5. call trajectory optimization
            P.vdisp('Running trajectory optimization',4)
            
            % create initial guess
            initial_guess = zeros(2,1) ;

            % create optimization options
            options =  optimoptions('fmincon',...
                            'MaxFunctionEvaluations',1e5,...
                            'MaxIterations',1e5,...
                            'OptimalityTolerance',1e-3',...
                            'CheckGradients',false,...
                            'FiniteDifferenceType','central',...
                            'Diagnostics','off',...
                            'SpecifyConstraintGradient',true,...
                            'SpecifyObjectiveGradient',true);
                        
            % call fmincon, with a try/catch since the cost and constraint
            % functions bail out by erroring if the timeout is reached
            try
                [k_opt,~,exitflag] = fmincon(cost,...
                                            initial_guess,...
                                            [],[],... % linear inequality constraints
                                            [],[],... % linear equality constraints
                                            k_bounds(:,1),... % lower bounds
                                            k_bounds(:,2),... % upper bounds
                                            nonlcon,...
                                            options) ;
            catch
                exitflag = -1 ;
            end
        
        %% 6. make the new plan or continue the old plan
            P.vdisp('Creating new plan',4)
            
            % if fmincon was successful, create a new plan
            if exitflag > 0
                P.vdisp('New plan successfully found!',5)
                w_des = full(msubs(FRS_cur.w_des,FRS_cur.k,k_opt)) ;
                v_des = full(msubs(FRS_cur.v_des,FRS_cur.k,k_opt)) ;

                % create the desired trajectory
                t_stop = v_des / 2 ;
                [T,U,Z] = make_turtlebot_braking_trajectory(FRS_cur.t_plan,...
                            t_stop,w_des,v_des) ;
                        
                % move plan to world coordinates
                Z(1:3,:) = local_to_world(agent_state,Z(1:3,:)) ;
            else
            % if fmincon was unsuccessful, try to continue executing the
            % previous plan
                P.vdisp('Continuing previous plan!',5)
                k_opt = nan(2,1) ; % dummy k_opt to fill in info struct
                
                % first, check if there is enough of the past plan left to
                % keep executing
                T_old = P.current_plan.T ;
                U_old = P.current_plan.U ;
                Z_old = P.current_plan.Z ;
                
                if ~isempty(T_old)
                    % try shifting the current plan by P.t_plan
                    T_log = T_old >= P.t_move ;
                else
                    T_log = false ;
                end
                   
                % if there is enough of the previous plan remaining, pass
                % that along; otherwise, just send a control input that
                % commands a stop
                if any(T_log)
                    % increment the time and input
                    T = T_old(T_log) - P.t_move ;
                    U = U_old(:,T_log) ;
                    Z = Z_old(:,T_log) ;
                else
                    % create stopped control input
                    T = [0, 2*P.t_move] ;
                    U = zeros(2,2) ;
                    Z = [zeros(4,2)] ;
                end
                
                % make sure the new plan is long enough
                if T(end) < P.t_move
                    T = [T, P.t_move] ;
                    U = [U, zeros(2,1)] ;
                    Z = [Z, [Z(1:3,end);0] ] ;
                end
            end
            
            % save the new plan
            P.current_plan.T = T ;
            P.current_plan.U = U ;
            P.current_plan.Z = Z ;
            
        %% 7. update the info structure
            I = P.info ;
            
            I.agent_time = [I.agent_time, agent_info.time(end)] ;
            I.agent_state = [I.agent_state, agent_state] ;
            I.k_opt_found = [I.k_opt_found, k_opt] ;
            I.FRS_index = [I.FRS_index, current_FRS_index] ;
            I.waypoint = [I.waypoint, z_goal] ;
            I.obstacles = [I.obstacles, {O}] ;
            I.obstacles_in_world_frame = [I.obstacles_in_world_frame, {O_pts}] ;
            I.obstacles_in_FRS_frame = [I.obstacles_in_FRS_frame, {O_FRS}] ;
            P.info = I ;
            
        end
        
        %% plotting
        function plot(P,~)
            P.plot_at_time() ;
        end
        
        function plot_at_time(P,t)
            if nargin < 2
                if ~isempty(P.info.agent_time)
                    t = P.info.agent_time(end) ;
                else
                    t = 0 ;
                end
            end
            
            P.vdisp('Plotting!',8)
            
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on ;
            end
            
            % figure out the info index closest to the current time
            I = P.info ;
            info_idx = find(t >= I.agent_time,1,'last') ;
            info_idx_check = ~isempty(info_idx) ;
            
            % plot current obstacles
            if P.plot_obstacles_flag && info_idx_check
                O = I.obstacles_in_world_frame{info_idx} ;

                if isempty(O)
                    O = nan(2,1) ;       
                end

                if check_if_plot_is_available(P,'obstacles')
                    P.plot_data.obstacles.XData = O(1,:) ;
                    P.plot_data.obstacles.YData = O(2,:) ;
                else
                    obs_data = plot(O(1,:),O(2,:),'r.') ;
                    P.plot_data.obstacles = obs_data ;
                end
            end
            
            % plot current waypoint
            if P.plot_waypoints_flag && info_idx_check
                wp = I.waypoint(:,info_idx) ;
                if isempty(wp)
                    wp = nan(2,1) ;
                end

                if check_if_plot_is_available(P,'waypoint')
                    P.plot_data.waypoint.XData = wp(1) ;
                    P.plot_data.waypoint.YData = wp(2) ;
                else
                    wp_data = plot(wp(1),wp(2),'b*') ;
                    P.plot_data.waypoint = wp_data ;
                end
            end
            
            % plot FRS
            if P.plot_FRS_flag && info_idx_check
                % iterate back through the info indices until the last
                % info index where k_opt was found
                FRS_info_idx = info_idx ;
                k_opt_idx = nan(2,1);
                while FRS_info_idx > 0
                    k_opt_idx = I.k_opt_found(:,FRS_info_idx) ;
                    if ~isnan(k_opt_idx(1))
                        break
                    else
                        FRS_info_idx = FRS_info_idx - 1 ;
                    end
                end
                
                % get the FRS and agent state for the current info index
                if ~isempty(FRS_info_idx) && FRS_info_idx > 0 && ~isnan(k_opt_idx(1))
                    FRS_idx = P.FRS{I.FRS_index(FRS_info_idx)} ;
                    agent_state = I.agent_state(:,FRS_info_idx) ;
                    
                    if check_if_plot_is_available(P,'FRS')
                        % get polynomial sliced by k_opt
                        FRS_poly = msubs(FRS_idx.FRS_polynomial,FRS_idx.k,k_opt_idx) ;
                        
                        % get the 2D contour points to plot
                        [~,FRS_patch_info,N] = get_2D_contour_points(FRS_poly,FRS_idx.z,1,'Bounds',0.9) ;
                        
                        % get the contour with the most vertices
                        [~,plot_idx] = max(N) ;
                        FRS_patch_info = FRS_patch_info(plot_idx) ;
                        
                        % put the vertices in the world frame
                        V = FRS_patch_info.Vertices ;
                        V = FRS_to_world(V',agent_state,...
                            FRS_idx.initial_x,FRS_idx.initial_y,FRS_idx.distance_scale)' ;
                        
                        P.plot_data.FRS.Faces = FRS_patch_info.Faces ;
                        P.plot_data.FRS.Vertices = V ;
                    else
                        if ~isnan(k_opt_idx(1))
                            FRS_data = plot_turtlebot_FRS_in_world_frame(FRS_idx,...
                                k_opt_idx,agent_state,...
                                'FaceColor',[0.5 1 0.3],'FaceAlpha',0.2,...
                                'EdgeColor',[0 0.6 0],'EdgeAlpha',0.5') ;
                            P.plot_data.FRS = FRS_data ;
                        end
                    end
                end
            end
            
            % plot high-level planner
            if P.plot_HLP_flag
                plot(P.HLP)
            end
            
            if hold_check
                hold off
            end
        end
    end
end