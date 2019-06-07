classdef turtlebot_RTD_planner_static < planner
% Class: turtlebot_RTD_planner_static < planner
%
% This class implements RTD for a TurtleBot in static environments. It does
% not inherit the generic RTD planner superclass, so that you can see how
% all the parts of a planner should be written in a simulator framework.
%
% Author: Shreyas Kousik
% Created: 6 June 2019

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
        FRS_polynomial_structure
        
        % obstacle handling
        point_spacing
        current_obstacles_raw
        current_obstacles_in_FRS_coords
        bounds_as_obstacle
        
        % plan handling
        current_waypoint
        lookahead_distance = 1.5 ;
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
            HLP = straight_line_HLP() ; % high level planner
            
            % parse the input arguments; these should be given in the
            % format 'property1', value1, 'property2', value2,...
            P = parse_args(P,'name',name,'buffer',buffer,'HLP',HLP,...
                           varargin{:}) ;
            
            % load FRS files
            FRS_data = cell(1,3) ;
            FRS_data{1} = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;
            FRS_data{2} = load('turtlebot_FRS_deg_10_v0_0.5_to_1.0.mat') ;
            FRS_data{3} = load('turtlebot_FRS_deg_10_v0_1.0_to_1.5.mat') ;
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
            
        %% 1. compute point spacing
            bbar = agent_info.footprint ;
            b = P.buffer ;
            
            if b >= bbar
                P.buffer = bbar - 0.001 ;
                P.vdisp('Reducing buffer to be feasible!',2)
            end
            
            P.point_spacing = compute_turtlebot_point_spacings(bbar,P.buffer) ;
            
        %% 2. set up world boundaries as an obstacle
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
            P.HLP.goal = world_info.goal ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
        %% 4. process the FRS polynomial
            P.FRS_polynomial_structure = cell(1,3) ;
            
            for idx = 1:3
                I = P.FRS{idx}.FRS_polynomial - 1 ;
                z = P.FRS{idx}.z ;
                k = P.FRS{idx}.k ;
                P.FRS_polynomial_structure{idx} = get_FRS_polynomial_structure(I,z,k) ;
            end
            
        %% 5. initialize the current plan as empty
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
        end
        
    %% replan
        function [T,U,Z] = replan(P,A,W)
            % [T,U,Z] = P.replan(agent_info,world_info)
            %
            % This is the heart of the RTD planner. In this method, we
            
            % start a timer to enforce the planning timeout P.t_plan
            start_tic = tic ;
            
        %% 1. determine the current FRS based on the agent
            agent_state = A.state(:,end) ; % (x,y,h,v)
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
            O = W.obstacles ;
            
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
            % make a waypoint
            z_goal = P.HLP.get_waypoint(A,W,P.lookahead_distance) ;
            P.current_waypoint = z_goal ;
            
            % put waypoint into FRS frame to use for planning
            z_goal_local = world_to_local(agent_state(1:3),z_goal(1:2),0,0,1) ;
            
            % create cost function
            cost = @(k) turtlebot_cost_for_fmincon(k,...
                            FRS_cur,z_goal_local,...
                            start_tic,P.t_plan) ;
        
        %% 4. create the constraints for fmincon
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
                P.trajopt_problem.nonlcon_function = [] ;
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
            % if fmincon was successful, create a new plan
            if exitflag > 0
                w_des = full(msubs(FRS_cur.w_des,FRS_cur.k,k_opt)) ;
                v_des = full(msubs(FRS_cur.v_des,FRS_cur.k,k_opt)) ;

                % create the desired trajectory
                [T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(FRS_cur.t_f,w_des,v_des) ;

                % create the trajectory with braking
                [T,U,Z] = make_turtlebot_RTD_braking_traj(FRS_cur.t_plan,FRS_cur.t_stop,T_go,U_go,Z_go) ;
            else
            % if fmincon was unsuccessful, try to continue executing the
            % previous plan
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
                    
                    % make sure the new plan is long enough
                    if T(end) < P.t_move
                        T = [T, P.t_move] ;
                        U = [U, zeros(2,1)] ;
                        Z = [Z, [Z(1:3,end);0] ] ;
                    end
                else
                    % create stopped control input
                    T = [0, 2*P.t_move] ;
                    U = zeros(2,2) ;
                    Z = [repmat(agent_state(1:3),1,2) ; zeros(1,2)] ;
                end
            end
            
            % save the new plan
            P.current_plan.T = T ;
            P.current_plan.U = U ;
            P.current_plan.Z = Z ;
        end
        
        %% plotting
        function plot(P,~)
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on ;
            end
            
            % plot current obstacles
            O = P.current_obstacles ;
            
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
            
            % plot current waypoint
            wp = P.current_waypoint ;
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
            
            if hold_check
                hold off
            end
        end
    end
end