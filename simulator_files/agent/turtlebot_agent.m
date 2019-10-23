classdef turtlebot_agent < RTD_agent_2D
% Class: turtlebot_agent < RTD_agent_2D < agent
%
% This implements the turtlebot robot with unicycle dynamics in the paper,
% "An Efficient Reachability-Based Framework for Provably Safe, Autonomous
% Navigation in Unknown Environments" explained here:
%   https://abajcsy.github.io/safe_navigation/
%
% The paper is available here: https://arxiv.org/abs/1905.00532
% The dynamics are in equation (13)
%
% Some hardware specs are here:
%   http://www.ros.org/reps/rep-0119.html
%   https://sceweb.sce.uhcl.edu/harman/CENG5435_ROS/CENG5435_WebFall18/TurtleBotGuideSeminar2_2_2018.pdf
%
% Note that we treat yaw rate as the first input and acceleration as the
% second input.
    
    properties
        % state limits
        max_speed = 2 ; % m/s (NOTE this is higher than in the specs, since
                        % the planner should not command speeds above its
                        % own limit, but the robot is not limited by the
                        % planning algorithm)
        
        % state indices
        speed_index = 4 ;
                        
        % input limits (NOTE these are higher than in the paper as well,
        % and are based on guesses of what the TurtleBot does in YouTube
        % videos, where it can spin and accelerate really fast)
        max_yaw_rate = 2.0 ; % rad/s     
        max_accel = 2.0 ; % m/s^2   
        
        % integrator type, to allow for fixed time step integration
        integrator_type = 'ode45' ; % choose 'ode45' or 'ode4' or 'ode113'
        integrator_time_discretization = 0.01 ; % for ode4
    end
    
    methods
        %% constructor
        function A = turtlebot_agent(varargin)
            % set up default superclass values
            name = 'turtlebot' ;
            default_footprint = 0.35/2 ;
            n_states = 4 ;
            n_inputs = 2 ;
            stopping_time = 3.75 ;
            sensor_radius = 4 ;
            LLC = turtlebot_PD_LLC ;
            
            % create agent
            A@RTD_agent_2D('name',name,...
                'footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,...
                'LLC',LLC,varargin{:}) ;
        end
        
        %% emergency stop
        % note, this ignores any previous trajectory the agent may have
        % been tracking; we have to define this different from the default
        % for the TurtleBot because it takes in acceleration as a control
        % input, as opposed to doing feedback about desired speed
        function stop(A,t_stop)
            if nargin < 2
                t_stop = A.stopping_time ;
            end
            
            % get the current speed
            v = A.state(A.speed_index,end) ;
            
            % get braking input
            u_stop = -A.max_accel ;
            
            if v == 0
                % if we are already stopped, stay stopped
                T_input = [0, t_stop] ;
                U_input = zeros(2) ;
            else
                % check how long it will take to come to a stop
                t_req_to_stop = v/A.max_accel ;

                if t_req_to_stop < t_stop
                    T_input = [0, t_req_to_stop, t_stop] ;
                    U_input = [0 0 0 ;
                               u_stop, 0, 0] ;
                else
                    T_input = [0, t_stop] ;
                    U_input = [0 0 ;
                               u_stop, 0] ;
                end
            end
            
            % call move method to perform braking
            A.move(t_stop,T_input,U_input) ;
        end
        
        %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % handle no desired trajectory input
            if nargin < 6
                Z = [] ;
            end
            
            % extract the states
            h = z(A.heading_index) ;
            v = z(A.speed_index) ;
            
            % get nominal control inputs
            u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            u(isnan(u)) = 0 ; % safety check
            w_des = u(1) ;
            a_des = u(2) ;
            
            % saturate the inputs
            w = bound_values(w_des,A.max_yaw_rate) ;
            a = bound_values(a_des,A.max_accel) ;
            
            % calculate the derivatives
            xd = v*cos(h) ;
            yd = v*sin(h) ;
            hd = w ;
            vd = a ;
            
            % return state derivative
            zd = [xd ; yd ; hd ; vd] ;
        end
        
        %% integrator options
        function [tout,zout] = integrator(A,fun,tspan,z0)
            switch A.integrator_type
                case 'ode45'
                    [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
                case 'ode113'
                    [tout,zout] = ode113(@(t,z) fun(t,z),tspan,z0(:)) ;
                case {'ode4','RK4'}
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    if tout(end) ~= tspan(end)
                        tout = [tout, tspan(end)] ;
                    end
                    zout = ode4(@(t,z) fun(t,z),tout,z0(:)) ;
                otherwise
                    error('Please set A.integrator_type to either ode45 or ode4')
            end
            tout = tout(:)' ;
            zout = zout' ;
        end
    end
end