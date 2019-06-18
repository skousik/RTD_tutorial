classdef turtlebot_RTD_planner_static_subclass < generic_RTD_planner
% Class: turtlebot_RTD_planner_static_subclass < generic_RTD_planner
%
% This class implements RTD for a TurtleBot in static environments.
%
% Author: Shreyas Kousik
% Created: 6 June 2019

    properties
        point_spacing
        FRS_polynomial_structure
        current_obstacles_raw
        current_obstacles_in_FRS_coords
        bounds_as_obstacle
    end
    
    methods
        %% constructor and setup
        function P = turtlebot_RTD_planner_static_subclass(varargin)
            % set default parameters
            t_plan = 0.5 ;
            t_move = 0.5 ;
            lookahead_distance = 1.75 ;
            
            % parse arguments
            P@generic_RTD_planner('lookahead_distance',lookahead_distance,...
                't_plan',t_plan,'t_move',t_move,varargin{:})
        end
        
        function load_FRS_files(P,~,~)
            FRS_data = cell(1,3) ;
            FRS_data{1} = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;
            FRS_data{2} = load('turtlebot_FRS_deg_10_v0_0.5_to_1.0.mat') ;
            FRS_data{3} = load('turtlebot_FRS_deg_10_v0_1.0_to_1.5.mat') ;
            P.FRS = FRS_data ;
        end
        
        function setup(P,agent_info,world_info)
            % call superclass setup
            setup@generic_RTD_planner(P,agent_info,world_info)
            
            % make sure buffer is small enough, i.e. b < \bar{b}
            bbar = agent_info.footprint ;
            b = P.buffer ;
            
            if b >= bbar
                P.buffer = bbar - 0.001 ;
                P.vdisp('Reducing buffer to be feasible!',2)
            end
            
            % set point spacing
            P.point_spacing = compute_turtlebot_point_spacings(bbar,P.buffer) ;
            
            % set the world bounds with the buffer
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
            
            % P.bounds_as_obstacle = interpolate_polyline_with_spacing(B(:,end:-1:1),P.point_spacing);
            P.bounds_as_obstacle = B ;
            
            % process the FRS polynomial
            for idx = 1:3
                I = P.FRS{idx}.FRS_polynomial - 1 ;
                z = P.FRS{idx}.z ;
                k = P.FRS{idx}.k ;
                P.FRS_polynomial_structure{idx} = get_FRS_polynomial_structure(I,z,k) ;
            end
        end
        
        %% online planning: process obstacles
        function determine_current_FRS(P,agent_info)
            if nargin < 2
                v_0 = 0 ;
            else
                v_0 = agent_info.state(4,end) ; % initial speed
            end
            
            % pick fastest FRS for current speed
            if v_0 >= 1.0
                P.current_FRS_index = 3 ;
            elseif v_0 >= 0.5
                P.current_FRS_index = 2 ;
            else
                P.current_FRS_index = 1 ;
            end
        end
        
        function process_world_info(P,world_info,~)
            % extract obstacles
            O = world_info.obstacles ;
            
            % get world bounds as obstacle
            O_bounds = P.bounds_as_obstacle ;
            
            % get current FRS
            FRS_cur = P.FRS{P.current_FRS_index} ;
            
            % buffer and discretize obstacles
            [O_FRS, ~, O_pts] = compute_turtlebot_discretized_obs(O,...
                    P.agent_state,P.buffer,P.point_spacing,FRS_cur,O_bounds) ;
            
            % save obstacles
            P.current_obstacles_raw = O ;
            P.current_obstacles = O_pts ;
            P.current_obstacles_in_FRS_coords = O_FRS ;
        end
        
        %% online planning: cost function
        function create_cost_function(P,agent_info,world_info,start_tic)
            % create waypoint
            P.create_waypoint(agent_info,world_info,start_tic) ;
            
            % get agent state
            z = P.agent_state ;
            
            % rotate waypoint to body-fixed frame
            z_goal = P.current_waypoint ;
            z_goal_local = world_to_local(z(1:3),z_goal(1:2),0,0,1) ;
            
            % create cost function
            FRS_cur = P.FRS{P.current_FRS_index} ;
            P.trajopt_problem.cost_function = @(k) turtlebot_cost_for_fmincon(k,...
                                                    FRS_cur,z_goal_local,...
                                                    start_tic,P.t_plan) ;
        end
        
        %% online planning: constraint function
        function create_constraints(P,start_tic)
            % get the processed obstacles
            O_FRS = P.current_obstacles_in_FRS_coords ;
            
            % get current FRS
            FRS_cur = P.FRS{P.current_FRS_index} ;
            
            if ~isempty(O_FRS)
                % remove NaNs
                O_log = isnan(O_FRS(1,:)) ;
                O_FRS = O_FRS(:,~O_log) ;
                
                % get FRS polynomial
                FRS_poly = P.FRS_polynomial_structure{P.current_FRS_index} ;

                % plug in to FRS polynomial
                cons_poly = evaluate_FRS_polynomial_on_obstacle_points(FRS_poly,O_FRS) ;

                % get the gradient of the constraint polynomials
                cons_poly_grad = get_constraint_polynomial_gradient(cons_poly) ;
                
                % create constraint function
                P.trajopt_problem.nonlcon_function = @(k) turtlebot_nonlcon_for_fmincon(k,...
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
            v_0 = P.agent_state(end) ;
            v_max = FRS_cur.v_range(2) ;
            v_des_lo = max(v_0 - FRS_cur.delta_v, FRS_cur.v_range(1)) ;
            v_des_hi = min(v_0 + FRS_cur.delta_v, FRS_cur.v_range(2)) ;
            k_2_lo = (v_des_lo - v_max/2)*(2/v_max) ;
            k_2_hi = (v_des_hi - v_max/2)*(2/v_max) ;
            k_2_bounds = [k_2_lo, k_2_hi] ;

            % combine bounds
            k_bounds = [k_1_bounds ; k_2_bounds] ;
            
            % create the inequality constraints and problem bounds
            P.trajopt_problem.Aineq = [] ;
            P.trajopt_problem.bineq = [] ;
            P.trajopt_problem.k_bounds = k_bounds ;
        end
        
        %% online planning: create output given successful replan
        function [T_out,U_out,Z_out] = create_desired_trajectory(P,~,k_opt)
            % get current FRS and k
            FRS_cur = P.FRS{P.current_FRS_index} ;
            k = FRS_cur.k ;
            
            % get the desired speed and yaw rate
            w_des = full(msubs(FRS_cur.w_des,k,k_opt)) ;
            v_des = full(msubs(FRS_cur.v_des,k,k_opt)) ;

            % create the desired trajectory
            [T_out,U_out,Z_out] = make_turtlebot_braking_trajectory(FRS_cur.t_plan,...
                                    FRS_cur.t_f,FRS_cur.t_stop,w_des,v_des) ;
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
        
        function plot_at_time(P,~)
            P.vdisp('No plotting at current time defined.',10)
        end
    end
end