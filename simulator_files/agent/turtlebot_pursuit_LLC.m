classdef turtlebot_pursuit_LLC < low_level_controller
% turtlebot_pursuit_LLC < low_level_controller
%
% This controller performs pursuit of a given desired trajectory. It is
% mildly inspired by the following paper:
%
% https://apps.dtic.mil/dtic/tr/fulltext/u2/a255524.pdf
%
% Author: Shreyas Kousik
% Created: 23 Oct 2019
% Updated: -
%
    %% properties
    properties
        % "carrot" distance
        lookahead_distance = 0.01 ; % m
        
        % feedback gains
        position_gain = 1 ;
        speed_gain = 3 ;
        yaw_gain = 10 ;
        yaw_pursuit_gain = 1 ;
        
        % feedforward gains
        accel_gain = 1 ;
        yaw_rate_gain = 1 ;
    end

    %% methods
    methods
        %% constructor
        function LLC = turtlebot_pursuit_LLC(varargin)
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
            
            % get desired state and inputs (assumes zero-order hold)
            if isempty(Z_des)
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_des = match_trajectories(t_cur,T_des,U_des,'previous') ;
                p_des = z_cur(A.position_indices) ;
                v_des = 0 ;
                h_des = h_cur ;
                d_cur = 0 ;
                d_des = 0 ;
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                
                [d_cur,~,d_along] = dist_point_on_polyline(p_cur,Z_des(1:2,:)) ;
                d_fdbk = min(d_cur + LLC.lookahead_distance, d_along(end)) ;
                
                [u_des,z_des] = match_trajectories(d_fdbk,d_along,U_des,d_along,Z_des,'previous') ;
                p_des = z_des(A.position_indices) ;
                v_des = z_des(A.speed_index) ;
                h_des = z_des(A.heading_index) ;
                d_des = match_trajectories(t_cur,T_des,d_along) ;
            end
            
            % get gains
            k_p = LLC.position_gain ;
            k_v = LLC.speed_gain ;
            k_h = LLC.yaw_gain ;
            k_hp = LLC.yaw_pursuit_gain ;
            
            k_w = LLC.yaw_rate_gain ;
            k_a = LLC.accel_gain ;
            
            % get desired feedforward inputs
            w_des = u_des(1) ;
            a_des = u_des(2) ;
            
            % compute heading relative to desired position
            p_err = p_des - p_cur ;
            hp_err = atan2(p_err(2),p_err(1)) ;
            
            % compute unsaturated inputs (they get saturated by the agent)
            w_out = k_h*(h_des - h_cur) + k_w*w_des + k_hp*hp_err ;
            a_out = k_p*(d_des - d_cur) + k_v*(v_des - v_cur) + k_a*a_des;
            
            % create output
            U = [w_out ; a_out] ;
        end
    end
end