classdef my_turtlebot_RTD_planner < planner
    properties
        FRS
        FRS_polynomial_structure
        point_spacing
        bounds_as_obstacle
        lookahead_distance = 1.5 ; % default value
        current_obstacles_raw
        current_obstacles_in_FRS_coords
    end
    methods
        function P = my_turtlebot_RTD_planner(varargin)
            name = 'My TurtleBot RTD planner' ;
            buffer = 1 ; % we'll overwrite this if we need to
            HLP = straight_line_HLP() ; % this is part of the simulator repo
            
            P = parse_args(P,'name',name,'buffer',buffer,'HLP',HLP,varargin{:}) ;
            
            FRS_data = cell(1,3) ;
            FRS_data{1} = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;
            FRS_data{2} = load('turtlebot_FRS_deg_10_v0_0.5_to_1.0.mat') ;
            FRS_data{3} = load('turtlebot_FRS_deg_10_v0_1.0_to_1.5.mat') ;
            P.FRS = FRS_data ;
        end
        
        function setup(P,agent_info,world_info)
            P.point_spacing = compute_turtlebot_point_spacings(agent_info.footprint,P.buffer) ;
            
            P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;
            
            xlo = P.bounds(1) ; xhi = P.bounds(2) ;
            ylo = P.bounds(3) ; yhi = P.bounds(4) ;
            
            B = [xlo, xhi, xhi, xlo, xlo ; ylo, ylo, yhi, yhi, ylo] ;
            B = [B, nan(2,1), 1.01.*B(:,end:-1:1)] ;
            
            P.bounds_as_obstacle = B ;
            
            P.HLP.goal = world_info.goal ;
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
            P.FRS_polynomial_structure = cell(1,3) ;
            
            for idx = 1:3
                I = P.FRS{idx}.FRS_polynomial - 1 ;
                z = P.FRS{idx}.z ;
                k = P.FRS{idx}.k ;
                P.FRS_polynomial_structure{idx} = get_FRS_polynomial_structure(I,z,k) ;
            end
            
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
        end
        
        function [T,U,Z] = replan(P,agent_info,world_info)
            start_tic = tic ; % to enforce the planning timeout P.t_plan
            
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
            
            % make a waypoint
            z_goal = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            
            % put waypoint into FRS frame to use for planning
            z_goal_local = world_to_local(agent_state(1:3),z_goal(1:2),0,0,1) ;
            
            % create cost function
            cost = @(k) turtlebot_cost_for_fmincon(k,FRS_cur,z_goal_local,start_tic,P.t_plan) ;
            
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
            
            if exitflag > 0
                w_des = full(msubs(FRS_cur.w_des,FRS_cur.k,k_opt)) ;
                v_des = full(msubs(FRS_cur.v_des,FRS_cur.k,k_opt)) ;
                
                % create the desired trajectory
                [T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(FRS_cur.t_f,w_des,v_des) ;
                
                % create the trajectory with braking
                [T,U,Z] = make_turtlebot_RTD_braking_traj(FRS_cur.t_plan,FRS_cur.t_stop,T_go,U_go,Z_go) ;
            else
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
                
                if any(T_log)
                    % increment the time and input
                    T = T_old(T_log) - P.t_move ;
                    U = U_old(:,T_log) ;
                    Z = Z_old(:,T_log) ;
                else
                    % create emergency stop control input
                    T = [0, 2*P.t_move] ;
                    U = zeros(2,2) ;
                    Z = [repmat(agent_state(1:3),1,2) ; zeros(1,2)] ;
                end
                
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
        end
    end
end