# Extras 1: Writing an RTD Planner

Here, we take `example_10_trajectory_optimization.m` from [Step 4](https://github.com/skousik/RTD_tutorial/tree/master/step_4_online_planning) and turn it into a planner object for the [simulator](https://github.com/skousik/simulator). The file we're writing in particular is called `turtlebot_RTD_planner_static.m`, which is in the [simulator files](https://github.com/skousik/RTD_tutorial/tree/master/simulator_files) folder.

The `simulator` requires three things to run. An `agent`, which is a representation of a robot; a `planner`, which is described below; and a `world` full of obstacles that the agent/robot lives in, and the planner must deal with.



## Extra 1.1: Planner Overview

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



## Extra 1.2: Writing the TurtleBot RTD Planner

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

Coming soon!



### Method 3: Replan

Coming soon!

## Extra 1.3: Running Simulations

Coming soon!