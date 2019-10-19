# Extra 3: Writing an RRT* Planner

[Go to tutorial home page](https://github.com/skousik/RTD_tutorial)

This is just a quick extra that makes use of the `simulator` built-in RRT* high-level planner, `RRT_star_HLP.m`. Since that outputs a path, we just need to convert it to a trajectory that can be used by the Turtlebot.

As with the [RTD planner extra](https://github.com/skousik/RTD_tutorial/tree/master/step_5_extras/extra_2_writing_an_RTD_planner), we're going to create a receding-horizon planner that has three methods: a constructor, a `setup` method, and a `replan` method. The result of this tutorial extra is `turtlebot_RRT_star_planner.m`.

Note that we aren't discussing how to actually write RRT* in this extra! We're just wrapping it in a planner!

## Extra 3.1: Constructor and Template

To start, open a new MATLAB file and make the following template:

```matlab
classdef turtlebot_RRT_star_planner < planner
	properties
		current_path = [] ;
    lookahead_distance = 1 ;
    desired_speed = 1 ;
	end % properties
    
	methods
		function P = turtlebot_RRT_star_planner(varargin)
			% set up default properties and construct planner
			P = parse_args(P,'buffer',0,varargin{:}) ;
			P.HLP = RRT_star_HLP() ;
		end % constructor

		function setup(P,agent_info,world_info)
			% WE WILL FILL THIS IN
		end % setup
		
		function [T,U,Z] = replan(P,agent_info,world_info)
			% WE WILL FILL THIS IN
		end % replan
	end % methods
end % classdef
```



Note that we already filled in the constructor, because it's straightforward. The one tricky part is that the syntax above sets the  `buffer` property to a default value of 0, but the user can override that with the `varargin`.

Most importantly, note that we plugged in the RRT* high-level planner to the `HLP` property!



## Extra 3.2: Setup

The `setup` method just needs to establish the HLP's properties so it can plan towards the goal.

```matlab
function setup(P,agent_info,world_info)
	P.HLP.goal = world_info.goal ;
	P.HLP.default_lookahead_distance = P.lookahead_distance ;
	P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;
	P.HLP.bounds = P.bounds ;
end
```



## Extra 3.3: Replan

This is where we wrap the high-level planner in a replan method. We need to do two things. First, run the RRT* until the planner's time limit (since we're writing a receding-horizon planner). Second, convert the output path into a trajectory.

We do so with the following replan method:

```matlab
function [T,U,Z] = replan(P,agent_info,world_info)
	% process obstacles
	O = world_info.obstacles ;
	O_buf = buffer_polygon_obstacles(O,P.buffer,2) ;

	% run RRT star
	P.HLP.plan_path(agent_info,O_buf,P.lookahead_distance) ;

	% get the current best path out
	X = P.HLP.plan ;

	% if the path is empty or too short, make sure it's long enough
	switch size(X,2)
		case 0
			X = repmat(agent_info.state(1:2,end),1,2) ;
		case 1
			X = [X X] ;
	end

	% the RRT* HLP outputs a path from end to beginning, so flip it
	X = X(:,end:-1:1) ;

	% convert X to a trajectory by assuming that we traverse it at
	% the given max speed
	s = P.desired_speed ;
	d = dist_polyline_cumulative(X) ;
	T = d./s ;

	% generate headings along trajectory
	dX = diff(X,[],2) ;
	h = atan2(dX(2,:),dX(1,:)) ;
	h = [h, h(end)] ;

	% generate trajectory and input
	N = size(X,2) ;
	Z = [X ; h ; s.*ones(1,N)] ;

	% generate inputs (these are dummy inputs, since we'll just track the
	% trajectory with feedback)
	U = zeros(2,N) ;
end
```

There's a little bit more going on inside the `turtlebot_RRT_star_planner.m` file if you look, but that's just to set up for plotting.

That wraps up the creation of the RRT* planner class!

## Extra 3.4: Running a Simulation

To run a simulation, let's first set up the world and planner with some user parameters (execute the following on the MATLAB command line):

```matlab
% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 7 ;
bounds = [-4,4,-2,2] ;
goal_radius = 0.5 ;

% planner
buffer = 0.1 ; % m
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;

% simulation
verbose_level = 5 ;
```



Now, we'll create an agent and modify it's low-level controller so it can do feedback about the trajectories produced by the RRT* planner.

```matlab
A = turtlebot_agent ;
A.LLC.yaw_gain = 10 ;
A.LLC.lookahead_time = 0.05 ;
```



Next, we set up the planner and world just as with the RTD simulations from the other parts of this tutorial.

```matlab
P = turtlebot_RRT_star_planner('verbose',verbose_level,'buffer',0.1,...
		't_plan',t_plan,'t_move',t_move) ;

W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',0.25,...
		'verbose',verbose_level,'goal_radius',goal_radius,...
		'obstacle_size_bounds',obstacle_size_bounds) ;
```



Finally, set up the simulator.

```matlab
S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
		'max_sim_time',30,'max_sim_iterations',60) ;
```



Now, run the simulator! You'll see the turtlebot track an RRT*-generated trajectory, and maybe crash, since it has no safety guarantees.

```
S.run() ;
```

