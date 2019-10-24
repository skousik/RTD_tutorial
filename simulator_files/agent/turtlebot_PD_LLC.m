classdef turtlebot_PD_LLC < turtlebot_LLC
    methods
        %% constructor
        function LLC = turtlebot_PD_LLC(varargin)
            % set default lookahead properties
            lookahead_distance = 0.01 ; % m
            lookahead_time = 0.01 ; % s
            lookahead_type = 'time' ;
            
            % set default gains
            DG.position = 9 ;
            DG.speed = 12 ;
            DG.yaw = 1 ;
            DG.yaw_from_position = 0 ;
            DG.yaw_rate = 1 ;
            DG.acceleration = 1 ;
            
            % set gains for A.stop() method
            SG.position = 0 ;
            SG.speed = 0 ;
            SG.yaw = 0 ;
            SG.yaw_from_position = 0 ;
            SG.yaw_rate = 1 ;
            SG.acceleration = 1 ;
            
            % create low-level controller
            LLC@turtlebot_LLC('lookahead_distance',lookahead_distance,...
                'lookahead_time',lookahead_time,...
                'lookahead_type',lookahead_type,...
                'gains',DG,'default_gains',DG,'stop_gains',SG,...
                varargin{:}) ;            
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
            k_p = LLC.gains.position ;
            k_v = LLC.gains.speed ;
            k_a = LLC.gains.acceleration ;
            k_h = LLC.gains.yaw ;
            k_w = LLC.gains.yaw_rate ;
            k_hp = LLC.gains.yaw_from_position ;
            
            % compute position error in heading direction
            R = rotation_matrix_2D(h_cur) ;
            p_err = R*(p_des - p_cur) ;
            px_err = p_err(1) ;
            
            % compute heading relative to desired position
            hp_err = -atan2(p_err(2),p_err(1)) ;
            
            % compute unsaturated inputs (they get saturated by the agent)
            w_out = k_h*(h_des - h_cur) + k_w*w_des + k_hp*hp_err ;
            a_out = k_p*px_err + k_v*(v_des - v_cur) + k_a*a_des;
            
            % create output
            U = [w_out ; a_out] ;
        end
    end
end