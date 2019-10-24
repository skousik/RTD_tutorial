classdef turtlebot_pursuit_LLC < turtlebot_LLC
% turtlebot_pursuit_LLC < turtlebot_LLC
%
% This controller performs pursuit of a given desired trajectory. It is
% roughly based off the controller in the following paper:
%
% "Evaluating a PID, pure pursuit, and weighted steering controller for an
% autonomous land vehicle" by A.L. Rankin, C.D. Crane III, and D.G.
% Armstrong II
%
% Author: Shreyas Kousik
% Created: 23 Oct 2019
% Updated: -
%
    methods
        %% constructor
        function LLC = turtlebot_pursuit_LLC(varargin)
            % set default lookahead properties
            lookahead_distance = 0.01 ; % m
            lookahead_time = 0.01 ; % s
            lookahead_type = 'time' ;
            
            % set default gains
            DG.position = 9 ;
            DG.speed = 13 ;
            DG.yaw = 0 ;
            DG.yaw_pursuit = 1 ;
            DG.yaw_rate = 1 ;
            DG.acceleration = 1 ;
            
            % set gains for A.stop() method
            SG.position = 0 ;
            SG.speed = 0 ;
            SG.yaw = 0 ;
            SG.yaw_pursuit = 0 ;
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
        function u = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
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
                
                switch LLC.lookahead_type
                    case 'distance'
                        d_fdbk = min(d_cur + LLC.lookahead_distance, d_along(end)) ;

                        [u_des,z_des] = match_trajectories(d_fdbk,d_along,U_des,d_along,Z_des) ;
                        d_des = d_fdbk ;
                    case 'time'
                        t_fdbk = min(t_cur + LLC.lookahead_time, T_des(end)) ;
                        [u_des,z_des] = match_trajectories(t_fdbk,T_des,U_des,T_des,Z_des) ;
                        d_des = match_trajectories(t_fdbk,T_des,d_along) ;
                    otherwise
                        error('Please pick ''time'' or ''distance'' for LLC.lookahead_type.')
                end
                p_des = z_des(A.position_indices) ;
                h_des = z_des(A.heading_index) ;
                v_des = z_des(A.speed_index) ;
            end
            
            % get gains
            k_p = LLC.gains.position ;
            k_v = LLC.gains.speed ;
            k_a = LLC.gains.acceleration ;
            k_h = LLC.gains.yaw ;
            k_w = LLC.gains.yaw_rate ;
            k_hp = LLC.gains.yaw_pursuit ;
            
            % get desired feedforward inputs
            w_des = u_des(1) ;
            a_des = u_des(2) ;
            
            % compute heading relative to desired position
            p_err = p_des - p_cur ;
            hp_err = atan(p_err(2)) ;
            
            % compute unsaturated inputs (they get saturated by the agent)
            w_out = k_h*(h_des - h_cur) + k_w*w_des + k_hp*hp_err ;
            a_out = k_p*(d_des - d_cur) + k_v*(v_des - v_cur) + k_a*a_des ;
            
            % create output
            u = [w_out ; a_out] ;
        end
    end
end