# Extras 1: Simulator Overview

[Go to tutorial home page](https://github.com/skousik/RTD_tutorial)

Recall that the [simulator](https://github.com/skousik/simulator) repository is needed for online planning with RTD. This section presents a brief overview of that repository.

The `simulator` requires three things to run. An `agent`, which is a representation of a robot; a `world` full of obstacles that the agent lives in; and a `planner`, which generates a reference trajectory for the agent to avoid obstacles while trying to reach some goal location in the world.

Each agent, world, and planner is implemented as a MATLAB class. They should inherit the `agent`, `world`, and `planner` superclasses, respectively (or inherit subclasses thereof). In MATLAB pseudocode, the general way we use the simulator is as follows:

```matlab
% set up agent, world, and planner
A = agent_subclass('property1',value1,'property2',value2,...) ;
W = world_subclass('property1',value1,'property2',value2,...) ;
P = planner_subclass('property1',value1,'property2',value2,...) ;
S = simulator(A,W,P,'property1',value1,'property2',value2,...)

% run simulation
S.run()
```

We have built this framework to run simulation comparisons of various planner types in a variety of worlds, given a single robot. So, instead of just passing in one planner and one world, you can also pass in a cell array of planners and worlds. As long as they all share certain required properties and methods, the simulator will run just fine.

In particular, the planner is expected to be a **receding-horizon planner**. This means that the planner creates a new, short plan while the agent executes a previously-found plan for a small duration. This strategy lets the agent learn about the world around it (i.e., sense obstacles) as it replans.

This writeup will review the properties and methods of the simulator, agent, world, and planner classes. To see how these are used, check out the rest of the RTD tutorial. In particular, check out the `run_turtlebot_simulation.m` script in [Step 4: Online Planning](https://github.com/skousik/RTD_tutorial/tree/master/step_4_online_planning#44-running-a-simulation).



## Extras 1.1: Simulator

### Properties

Coming soon!

### Methods

Coming soon!



## Extras 1.2: Agent

### Properties

Coming soon!

### Methods

Coming soon!



## Extras 1.3: World

### Properties

Coming soon!

### Methods

Coming soon!



## Extras 1.4: Planner

### Properties

Coming soon!

### Methods

Coming soon!

