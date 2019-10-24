classdef turtlebot_LLC < low_level_controller
    properties
        % lookahead properties
        lookahead_time
        lookahead_distance
        lookahead_type = 'time' ;
        
        % control gains structure
        gains = struct() ;
        default_gains = struct() ;
        stop_gains = struct() ;
    end
    
    methods
        %% constructor
        function LLC = turtlebot_LLC(varargin)
            n_agent_states = 4 ;
            n_agent_inputs = 2 ;
            
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
        end
    end
end