classdef turtlebot_PD_LLC < low_level_controller
    properties
        %% default values
        % lookahead time
        lookahead_time = 0 ;
        
        % feedback gains
        position_gain = 0 ;
        speed_gain = 3 ;
        yaw_gain = 0 ;
        yaw_from_position_gain = 0 ;
        
        % feedforward gains
        yaw_rate_gain = 1 ;
        accel_gain = 0 ;
        
        %% useful alternative values
        % lookahead_time = 0.5 ;
        % position_gain = 10 ;
        % speed_gain = 100 ;
        % yaw_gain = 3 ;
        % yaw_from_position_gain = 0.05 ;
        % yaw_rate_gain = 0 ;
        % accel_gain = 0 ;
    end
    
    methods
        %% constructor
        function LLC = turtlebot_PD_LLC(varargin)
            n_agent_states = 4 ;
            n_agent_inputs = 2 ;
            
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
        end
        
        %% get control inputs
        function U = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
            % get current state
            p_cur = z_cur(A.position_indices) ;
            h_cur = z_cur(A.heading_index) ;
            v_cur = z_cur(A.speed_index) ;
            
            % get time along traj to use for feedback
            t_lkhd = LLC.lookahead_time ;
            t_fdbk = min(t_cur + t_lkhd, T_des(end)) ;
            
            % get desired state and inputs (assumes zero-order hold)
            if isempty(Z_des)
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_des = match_trajectories(t_fdbk,T_des,U_des,'previous') ;
                p_des = z_cur(A.position_indices) ;
                v_des = 0 ;
                h_des = h_cur ;
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                [u_des,z_des] = match_trajectories(t_fdbk,T_des,U_des,T_des,Z_des,'previous') ;
                p_des = z_des(A.position_indices) ;
                v_des = z_des(A.speed_index) ;
                h_des = z_des(A.heading_index) ;
            end
            
            % get desired feedforward inputs
            w_des = u_des(1) ;
            a_des = u_des(2) ;
            
            % get gains
            k_p = LLC.position_gain ;
            k_v = LLC.speed_gain ;
            k_a = LLC.accel_gain ;
            k_h = LLC.yaw_gain ;
            k_w = LLC.yaw_rate_gain ;
            k_hp = LLC.yaw_from_position_gain ;
            
            % compute position error in current heading direction
            R = rotation_matrix_2D(-h_cur) ;
            p_err = R*(p_des - p_cur) ;
            px_err = p_err(1) ;
            
            % compute heading relative to desired position
            hp_err = atan2(p_err(2),p_err(1)) ;
            
            % compute unsaturated inputs (they get saturated by the agent)
            w_out = k_h*(h_des - h_cur) + k_w*w_des + k_hp*hp_err ;
            a_out = k_p*px_err + k_v*(v_des - v_cur) + k_a*a_des;
            
            % create output
            U = [w_out ; a_out] ;
        end
    end
end