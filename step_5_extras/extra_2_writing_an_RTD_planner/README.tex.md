# Extras 2: Writing an RTD Planner

##### [Go to tutorial home page](https://github.com/skousik/RTD_tutorial)

Here, we take `example_10_trajectory_optimization.m` from [Step 4](https://github.com/skousik/RTD_tutorial/tree/master/step_4_online_planning) and turn it into a planner object for the [simulator](https://github.com/skousik/simulator). The file we're writing in particular is called `my_turtlebot_RTD_planner`, which is a barebones copy of `turtlebot_RTD_planner_static.m`. Both of these files are in the [simulator files](https://github.com/skousik/RTD_tutorial/tree/master/simulator_files) folder.

The `simulator` requires three things to run. An `agent`, which is a representation of a robot; a `planner`, which is described below; and a `world` full of obstacles that the agent/robot lives in, and the planner must deal with.



## Extra 2.1: Planner Overview

#### Planning Hierarchy

It's convenient to treat the control of an autonomous mobile robot as a three-tier hierarchy. At the highest tier, a **high-level planner** gives coarse route commands, like Google Maps. At the lower tier, a **low-level controller** actuates the robots' motors. In the middle, a **mid-level planner** or **trajectory planner** translates the coarse route commands into a **reference trajectory** or **desired trajectory** for the low-level controller to execute.

The high-level planner usually doesn't consider the robot's dynamics. The low-level controller usually can't consider the robot's environment (e.g., obstacles). Therefore, the mid-level planner is responsible for considering both the dynamics and the environment. In other words, _the mid-level planner is responsible for ensuring safety._

Autonomous robots usually plan with a receding-horizon strategy, where they make a short plan while executing a previously-found plan. The planner is responsible for making these plans (duh).

### The Planner Class

The `planner` class in the simulator framework encompasses the role of the mid-level planner. It is written as a [MATLAB class](https://www.mathworks.com/help/matlab/object-oriented-programming.html). If you're not familiar with MATLAB classes, then here's an opportunity to learn. The simulator and RTD repositories make heavy use of classes to prevent us from rewriting code over and over.

To peek inside the generic planner, first make sure the `simulator` repo is on your MATLAB path, then open the planner by running the following command on the MATLAB command line:

```matlab
open planner
```

You'll see that the planner inherits a handle, which makes it inheritable. Any planner used with `simulator` should inherit this planner, or a subclass of this planner such as `generic_RTD_planner`.

Any instance of `planner` must have the following methods:

1. the constructor
2. `setup`
3. `replan`

The constructor method just makes an instance of your particular planner. The `setup` method is called by the simulator before a simulation is run, and lets the planner use info from the agent and world. The `replan` method is the heart of the planner; this is where receding-horizon planning happens.

If you open some of the existing RTD planners, such as `turtlebot_RTD_planner_static_subclass`, `segway_RTD_planner_static`, or `generic_RTD_planner`, you'll find _tons_ of methods. However, these are all in support of the three important methods listed above, and all serve to reduce the amount of code that needs to be written when using RTD in MATLAB.

In the following, we'll write the constructor, the setup method, and the replan method for a TurtleBot planner. We'll add properties that we need as we go along.



## Extra 2.2: Writing the TurtleBot RTD Planner

Note that all of the code we'll go through here is in `turtlebot_RTD_planner_static.m`, as mentioned above.

Open a new MATLAB file, and make an outline for the planner we'll write together:

```matlab
classdef my_turtlebot_RTD_planner < planner
    properties
        % we'll fill this in
    end
    methods
        % we'll fill this in too
    end
end
```

Note, this tutorial will just explain each method and property you need to add. You're responsible for tacking on `end` statements and stuff (or you can just open the planner file and follow along).

### Constructor and Default Properties

The constructor will create a planner object, denoted `P`, and load up the FRS files that we need for online planning. To construct the planner, add the following method to your class file:

```matlab
function P = my_turtlebot_RTD_planner(varargin)
   % we'll fill this in
end
```

Notice that it takes in an arbitrary argument list. This is to allow us to create the planner with the following syntax:

```matlab
P = my_turtlebot_RTD_planner('property1',value1,'property2',value2,...)
```

The `planner` superclass has a few properties. In particular, these are:

```matlab
name % the planner's name, useful for when you have a bunch of these lying around
bounds % world bounds plus planner-specific buffer
buffer % minimum amount to buffer obstacles, given by the world
HLP % high level planner
current_plan ; % to be used by the replan method
current_obstacles ; % to be used by the replan method
verbose = 0 ; % higher number means more verbose
timeout = 1 ; % time allowed for "replan" function to execute
t_plan = 1 ; % same as timeout; just for notational purposes
t_move = 1 ;% amount of time the planner expects the agent to move
info % information structure to keep a log when planning
plot_data % data for current plot
plot_waypoints_flag = false ;
```

We will leave most of these alone for now, and just specify our own defaults for the name, buffer, and high-level planner by adding the following lines to our constructor method:

```matlab
name = 'My TurtleBot RTD planner' ;
buffer = 1 ; % we'll overwrite this if we need to
HLP = straight_line_HLP() ; % this is part of the simulator repo

P = parse_args(P,'name',name,'buffer',buffer,'HLP',HLP,varargin{:}) ;
```

The `parse_args` function is a utility function in `simulator` that just assigns each property, given by a keyword, to the value following that property. The first argument is the structure (or class, in this case) that the properties/values are being shoved into.

Now we need to load the FRS files. This is like what we did before, but we also need to plug them in to the planner's corresponding property. So, first, add the `FRS` as a property:

```matlab
properties
    FRS
end
```

This will be a cell array containing the data loaded from the FRSes that we computed for the TurtleBot. Now, add the following lines in the constructor method:

```matlab
FRS_data = cell(1,3) ;
FRS_data{1} = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;
FRS_data{2} = load('turtlebot_FRS_deg_10_v0_0.5_to_1.0.mat') ;
FRS_data{3} = load('turtlebot_FRS_deg_10_v0_1.0_to_1.5.mat') ;
P.FRS = FRS_data ;
```

With that, we have a working constructor method!



### Method 2: Setup

The `setup` method is called every time before running a simulation. This is because `simulator` can load up many worlds, and many planners, and run every planner in every world. Each planner needs a chance to get acquainted with the world (e.g., to understand the world boundaries) before every simulation. Hence, the setup method.

To start, add the following method after the constructor:

```matlab
function setup(P,agent_info,world_info)
    % we'll fill this in
end
```

For RTD, the setup method will do five things:

1. compute the point spacing used to discretize obstacles
2. set up the world boundaries as an obstacle
3. set up the high-level planner with the global goal
4. pre-process the FRS polynomial to make online planning real-time fast
5. initialize a `current_plan` property that will keep track of the plan we're generating

We'll add each of these in order now.



#### Compute the Point Spacing

First, we'll compute the point spacing as done in Example 9 and in [Section 6 of this monster paper](https://arxiv.org/abs/1809.06746). Add the following lines to your setup method:

```matlab
P.point_spacing = compute_turtlebot_point_spacings(agent_info.footprint,P.buffer) ;
```

You'll also have to update the planner's properties:

```matlab
properties
    FRS
    point_spacing
end
```



#### World Bounds as Obstacle

Now, let's make the world bounds into a polyline that we can treat as an obstacle:

```matlab
P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;

xlo = P.bounds(1) ; xhi = P.bounds(2) ;
ylo = P.bounds(3) ; yhi = P.bounds(4) ;

B = [xlo, xhi, xhi, xlo, xlo ; ylo, ylo, yhi, yhi, ylo] ;
B = [B, nan(2,1), 1.01.*B(:,end:-1:1)] ;

P.bounds_as_obstacle = B ;
```

Note that this defines the bound as a box with a hole in it that is the size of the free space given by `world_info`. To make this work, add the following property:

```matlab
properties
    FRS
    point_spacing
    bounds_as_obstacle
end
```



#### High-Level Planner Setup

This is pretty straightforward. Just add the following lines to give the high-level planner the information about our goal and lookahead distance:

```matlab
P.HLP.goal = world_info.goal ;
P.HLP.default_lookahead_distance = P.lookahead_distance ;
```

Look in the `straight_line_HLP` file to see how these are used. All high-level planners in the simulator framework can be written in similar ways (yay, object-oriented programming).

This requires adding a property:

```matlab
properties
    FRS
    point_spacing
    bounds_as_obstacle
    lookahead_distance = 1.5 ; % default value
end
```



#### Pre-Process FRS Polynomial

Recall that we [computed three FRSes](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation) for our TurtleBot. We'll pre-process them as done in [Example 10](https://github.com/skousik/RTD_tutorial/tree/master/step_4_online_planning#43-trajectory-optimization):

```matlab
P.FRS_polynomial_structure = cell(1,3) ;

for idx = 1:3
    I = P.FRS{idx}.FRS_polynomial - 1 ;
    z = P.FRS{idx}.z ;
    k = P.FRS{idx}.k ;
    P.FRS_polynomial_structure{idx} = get_FRS_polynomial_structure(I,z,k) ;
end
```

This requires you to add a property:

```matlab
properties
    FRS
    FRS_polynomial_structure
    point_spacing
    bounds_as_obstacle
    lookahead_distance = 1.5 ; % default value
end
```



#### Initialize Plan

Finally, add the following lines to the setup method to initialize the current plan as a structure:

```matlab
P.current_plan.T = [] ;
P.current_plan.U = [] ;
P.current_plan.Z = [] ;
```

Note that `current_plan` is a default property of `planner`.

These fields will contain a time array `T` with corresponding inputs `U` and desired trajectory `Z`. All three of these fields have data represented as columns. In other words `T` is a row vector of time, the columns of `U` are the TurtleBot's control inputs at each time in `T`, and the columns of `Z` are the desired state at each time in `T`. So, if `T` has `N` columns, then `U` is size 2-by-`N` and `Z` is size 4-by-`N`.



### Method 3: Replan

This is the Big One. This method gets called by the `simulator` in the loop to ask the planner to try to create a new plan (i.e., a new `T, U, Z`), within the planning timeout given by `P.t_plan`. The nice part about this method is that it more or less just puts all the code from `example_10_trajectory_optimization` into the turtlebot RTD planner class. So, a lot of this is a copy-paste job.

We'll write the replan method in six parts:

1. pick the current FRS to use based on the TurtleBot's speed
2. process obstacles to turn them into constraints for online planning
3. create the cost function for trajectory optimization
4. create the constraints for trajectory optimization
5. call `fmincon` to perform trajectory optimization
6. use the output of 5 to make a new plan

First, create the skeleton of the replan method by adding the following to your planner file's methods:

```matlab
function [T,U,Z] = replan(P,agent_info,world_info)
	start_tic = tic ; % to enforce the planning timeout P.t_plan

   % we'll fill the rest in
end
```

#### Pick the Current FRS

This is pretty straightforward, and uses the fact that `P.FRS` is a cell array of loaded info from three FRS .mat files. Add the following lines to your replan method:

```matlab
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
```



#### Process Obstacles

This is also pilfered straight from Example 10. Add the following lines to your replan method:

```matlab
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
```

This requires you to add the following properties to your planner:

```
current_obstacles_raw
current_obstacles_in_FRS_coords
```





#### Create the Cost Function

We'll use the same cost function as in Example 10, where we'll just try to get the TurtleBot to reach a waypoint. Add the following lines to your replan method:

```matlab
% make a waypoint
z_goal = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;

% put waypoint into FRS frame to use for planning
z_goal_local = world_to_local(agent_state(1:3),z_goal(1:2),0,0,1) ;

% create cost function
cost = @(k) turtlebot_cost_for_fmincon(k,FRS_cur,z_goal_local,start_tic,P.t_plan) ;
```

Notice that, unlike in Example 10, we're passing the `start_tic` timer to the cost function. This allows the cost function to do a timeout check. It will error (and cause `fmincon` to error) if the timeout is reached. We'll wrap `fmincon` in a try/catch block to handle this later.



#### Create the Constraints

Recall that we processed the obstacles above and made a variable `O_FRS`. We'll turn that into constraints exactly as in Example 10:

```matlab
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
```

Notice that, just like for the cost, `nonlcon` is handed the start tic. The function `turtlebot_nonlcon_for_fmincon` will error if the timeout `P.t_plan` is exceeded. This will be handled by a try/catch when we call `fmincon` later.

We now need to bound the space over which we're performing trajectory optimization, to make sure we don't exceed our limits on demanded change in velocity. Add the following lines to your replan method to accomplish this:

```matlab
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
```



#### Run Trajectory Optimization

Finally, we have everything in place to call `fmincon`. This is again pilfered from Example 10, but with a try/catch wrapped around `fmincon` to handle the fact that the cost and constraint functions will throw errors if they run out of time.

Add the following lines to your replan method to do trajectory optimization:

```matlab
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
```

We'll handle the exitflag being non-positive next, when we make the trajectory plan output.



#### Make a Trajectory Plan Output

So, optimistically, `fmincon` found a feasible solution. This means that `exitflag > 0`, yay! We'll handle this case by adding the following lines, which use the code way back in `example_3_braking_trajectory.m` to create a trajectory for the robot to execute. Add the following to your replan method:

```matlab
if exitflag > 0
    w_des = full(msubs(FRS_cur.w_des,FRS_cur.k,k_opt)) ;
    v_des = full(msubs(FRS_cur.v_des,FRS_cur.k,k_opt)) ;

    % create the desired trajectory
    [T,U,Z] = make_turtlebot_braking_trajectory(FRS_cur.t_plan,...
                  FRS_cur.t_f,FRS_cur.t_stop,w_des,v_des) ;
else
    % we'll fill this in
end
```

Now, in the failure case, we want to continue executing the previous plan, because we're doing receding horizon planning. First, we'll check just how much of the previous plan is left. Add this inside of the `else … end` part of the code above:

```matlab
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
```

Now, we'll handle the two cases. If there is enough of a plan left, then the robot will have moved for the duration `P.t_move`, so we'll just return everything left after `P.t_move` from the previous plan. Otherwise, we'll make up a plan that forces an emergency stop. Add the following lines:

```matlab
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
```

In the first case, we cycle through a duration `P.t_move` of the plan… but this doesn't guarantee that there still is a duration `t_move` left in the new plan `T`. So, let's tack on a bit of extra time to make sure that the plan is long enough:

```matlab
if T(end) < P.t_move
    T = [T, P.t_move] ;
    U = [U, zeros(2,1)] ;
    Z = [Z, [Z(1:3,end);0] ] ;
end
```

Finally, we have a plan for the robot to execute! Tada! Recall that all of this was nested in an `if exitflag > 0` block. Outside of that block, let's save our new plan:

```matlab
if exitflag > 0
   % blah blah blah
end

% save the new plan
P.current_plan.T = T ;
P.current_plan.U = U ;
P.current_plan.Z = Z ;
```

This concludes the `replan` method. Now, your planner is ready to roll!

### Method 4: Plotting

Note that we haven't defined any plotting methods for the planner we just wrote. The planner will still run fine, but it'll probably throw some warning messages. Copy/paste the `plot` method from `turtlebot_RTD_planner_static` into you planner to add some nice plotting and get rid of those warnings.



## Extra 2.3: Running Simulations

We can run a simulation as shown in the script `run_my_turtlebot_planner_simulation.m`. This is a copy of `run_turtlebot_simulation.m` with the planner swapped out for the planner that we just wrote above.

To use the planner you wrote above, first set some parameters:

```matlab
% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 7 ;
bounds = [-4,4,-2,2] ;
goal_radius = 0.5 ;

% planner
buffer = 0.05 ; % m
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;

% simulation
verbose_level = 10 ;
```

Now, create the agent, world, and planner:

```matlab
A = turtlebot_agent ;

P = my_turtlebot_RTD_planner('verbose',verbose_level,'buffer',buffer,...
                                 't_plan',t_plan,'t_move',t_move) ;

W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',0.25,...
                     'verbose',verbose_level,'goal_radius',goal_radius,...
                     'obstacle_size_bounds',obstacle_size_bounds) ;

```

Then, we can initialize and run the simulator with the following lines:

```matlab
S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',30,'max_sim_iterations',60) ;

S.run ;
```

This concludes the tutorial for how to write an RTD planner. Certainly you can get much fancier with it, but this should get you started.

##### [Go to tutorial home page](https://github.com/skousik/RTD_tutorial)

$\LaTeX$ compiled with TeXify.