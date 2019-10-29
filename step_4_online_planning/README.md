**TL;DR**: Run the script `step_4_run_turtlebot_simulation.m`.

# Step 4: Online Planning

#### [Previous step: computing the FRS](https://github.com/skousik/RTD_tutorial/tree/master/step3_FRS_computation)

Note that there is an example in the [RTD repository](https://github.com/ramvasudevan/RTD) for a Segway robot, which is really similar to the TurtleBot. Also, make sure you have the latest [RTD](https://github.com/ramvasudevan/RTD) and [simulator](https://github.com/skousik/simulator) repositories so that all the functions in this step work.

## Summary

In this step, we use the FRS computed in the [previous step](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation) to plan trajectories for the TurtleBot online (i.e., at runtime). First, we take the entire FRS and "intersect" it with obstacles around the robot. This intersection results in all trajectory parameters that may cause a collision. Finally, we optimize over the remaining collision-free trajectory parameters. If none can be found, then we execute the fail-safe maneuver from the previous planning iteration.

### Mathy Overview

Recall that RTD uses a trajectory parameter space <img src="/step_4_online_planning/tex/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode&sanitize=true" align=middle width=15.13700594999999pt height=22.465723500000017pt/> to specify desired trajectories. These trajectories are chosen every <img src="/step_4_online_planning/tex/5ae1e561ec81f97d93ed9df0f76cab27.svg?invert_in_darkmode&sanitize=true" align=middle width=30.730753349999993pt height=20.221802699999984pt/> seconds in a receding-horizon way. Also recall that, in Step 3, we found a function <img src="/step_4_online_planning/tex/2aa485270dee077a664c5c167a652424.svg?invert_in_darkmode&sanitize=true" align=middle width=107.28262545pt height=22.648391699999998pt/> for which, if <img src="/step_4_online_planning/tex/6b8afc7d21c1f242a41f95fe80e39813.svg?invert_in_darkmode&sanitize=true" align=middle width=89.58900389999998pt height=24.65753399999998pt/>, then <img src="/step_4_online_planning/tex/a91b124f86681090d790147ababa8c23.svg?invert_in_darkmode&sanitize=true" align=middle width=76.18710329999999pt height=24.65753399999998pt/>.

Now, to pick a particular <img src="/step_4_online_planning/tex/3bf25d9d50c08c7ec96446a7abb1c024.svg?invert_in_darkmode&sanitize=true" align=middle width=44.30350649999999pt height=22.831056599999986pt/> at each planning iteration, given an arbitrary cost function <img src="/step_4_online_planning/tex/5aefecdd2b9b5de63115b522e5e53f46.svg?invert_in_darkmode&sanitize=true" align=middle width=76.97455094999998pt height=22.648391699999998pt/>, we attempt to solve the following optimization problem:
<p align="center"><img src="/step_4_online_planning/tex/274155ee3483dbb84753ce0180293402.svg?invert_in_darkmode&sanitize=true" align=middle width=366.54306149999996pt height=17.031940199999998pt/></p>



The good thing about this is, as we saw in the previous section, <img src="/step_4_online_planning/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> gives us a _conservative_ approximation of the FRS. So, as long as the constraint <img src="/step_4_online_planning/tex/6bbaa8705b2be50aefae5dc1f516edb5.svg?invert_in_darkmode&sanitize=true" align=middle width=95.44139549999998pt height=24.65753399999998pt/> is obeyed for all <img src="/step_4_online_planning/tex/f93ce33e511096ed626b4719d50f17d2.svg?invert_in_darkmode&sanitize=true" align=middle width=8.367621899999993pt height=14.15524440000002pt/> in any obstacle, then we can prove that <img src="/step_4_online_planning/tex/62a1d6ae808b6d855355def103c4971f.svg?invert_in_darkmode&sanitize=true" align=middle width=15.81055739999999pt height=22.831056599999986pt/> is a collision-free trajectory (see [Lemma 15 on page 10 of this paper](https://arxiv.org/abs/1809.06746)).

The tricky part here is that, if an obstacle is a subset of <img src="/step_4_online_planning/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/>, then probably contains infinitely many points (for example, if the obstacle is a polygon). But this means that we have to evaluate <img src="/step_4_online_planning/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> on an infinite number of points to find <img src="/step_4_online_planning/tex/62a1d6ae808b6d855355def103c4971f.svg?invert_in_darkmode&sanitize=true" align=middle width=15.81055739999999pt height=22.831056599999986pt/>. In fact, we can actually do that — check out [Section III-B here](https://arxiv.org/abs/1705.00091). However, doing this kind of evaluation is way too slow for real time planning.

To avoid this evaluation of an infinite number of points, we instead prescribe a way to discretize obstacles into a finite number of points. In other words, if <img src="/step_4_online_planning/tex/d6ebaab75d0719c06a34784e25e82aff.svg?invert_in_darkmode&sanitize=true" align=middle width=65.37894825pt height=22.465723500000017pt/> is an obstacle, we represent it with a set <img src="/step_4_online_planning/tex/7087d8bef1803bb878de559f4ef83d1b.svg?invert_in_darkmode&sanitize=true" align=middle width=143.95909934999997pt height=24.65753399999998pt/>.

This discretization is explained in excruciating detail in [Section 6 of this paper](https://arxiv.org/abs/1809.06746). The key takeaway is that, even though each obstacle is represented by only a finite number of points, _we keep the collision-free guarantee_ that was the whole point of computing the FRS with tracking error in the first place.

### Goals for This Step

To do online planning, we'll do the following:

1. Map obstacles to the trajectory parameter space in a single planning iteration
2. Solve a nonlinear optimization program to pick a collision-free trajectory in a single planning iteration
3. Put 1 and 2 together in a "planner" object that will run RTD online in the [simulator](https://github.com/skousik/simulator) framework



## 4.1 Mapping Obstacles to Trajectory Parameters

In this part, we will represent an obstacle in the trajectory parameter space. First, let's assume that obstacles are sensed and handed to us as **polygons**, which is reasonable for a sensor like a planar LIDAR. Then, we can use the discretization mentioned above in the mathy overview.

The important points of the discretization are as follows. First, we only need to discretize the obstacle's boundary, since we can't get into the obstacle without passing through the boundary. Second, we have to buffer the obstacle before discretizing, to compensate for the fact that our robot shouldn't pass between any two points of the discretized boundary. Third, we need to find a **point spacing**, <img src="/step_4_online_planning/tex/ac4347acd383a6b21d3794b0b621326e.svg?invert_in_darkmode&sanitize=true" align=middle width=38.009795999999994pt height=21.18721440000001pt/>, that tells us how finely to discretize the boundary. In other words, we want to make sure that the discrete points are spaced no farther than <img src="/step_4_online_planning/tex/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode&sanitize=true" align=middle width=7.87295519999999pt height=14.15524440000002pt/> apart along the boundary.

The code we'll go over here is all in `step_4_ex_1_map_obs_to_traj_params.m`. We'll just cover the highlights.

### Example 1

In this example, we'll make a single polygonal obstacle in front of the robot, then map it to the parameter space using the FRS polynomial <img src="/step_4_online_planning/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> that acts like an indicator function on the FRS.

First, load the FRS and create a TurtleBot. We'll use the 0.0 — 0.5 m/s FRS from Step 3.

```matlab
FRS = load('turtlebot_FRS_deg_10_v_0_0.0_to_0.5.mat') ;
A = turtlebot_agent ;
```

Recall that the robot is initially at the origin. Now, let's make a random obstacle:

```matlab
obstacle_location = [1 ; 0] ; % (x,y)
obstacle_scale = 1.0 ;
N_vertices = 5 ;
O = make_random_polygon(N_vertices,obstacle_location,obstacle_scale) ;
```



#### Example 1.1: Obstacle Buffering and Discretization

The polyline `O` represents the obstacle as a polygon. To discretize it, we first need to buffer it:

```matlab
obstacle_buffer = 0.05 ; % m
O_buf = buffer_polygon_obstacles(O,obstacle_buffer,2) ;
```

The input `2` to `buffer_polygon_obstacles` makes sure that the buffered polygon does not get any rounded edges (this is a conservative buffer method, but it'll do for this example).

Now, we can discretize just the boundary of the obstacle. First, let's set up the point spacing as per [Example 67 on pg. 35](https://arxiv.org/abs/1809.06746):

```matlab
b = obstacle_buffer ;
R = A.footprint ;
theta_1 = acos((R-b)/R) ;
r = 2*R*sin(theta_1) ;
```

Now, we discretize the obstacle:

```matlab
O_pts = interpolate_polyline_with_spacing(O_buf,r) ;
```

Let's see what this looks like:

```matlab
figure(1) ; hold on ; axis equal ;

plot(A)
patch(O_buf(1,:),O_buf(2,:),[1 0.5 0.6]) % buffered obs
patch(O(1,:),O(2,:),[1 0.7 0.8]) % actual obs
plot(O_pts(1,:),O_pts(2,:),'.','Color',[0.5 0.1 0.1],'MarkerSize',15) % discretized obs
```

You should see something like this (of course, your obstacle will be a different shape and size):

<img src="images/step_4_ex_1_img_1.png" width="400px"/>

#### Example 1.2: Mapping Obstacle to FRS Frame

Recall that the FRS is computed in a scaled and shfited coordinate frame, to make sure all the trajectories stay within the <img src="/step_4_online_planning/tex/ad2444f8273c6541c5dc1fc5b6445401.svg?invert_in_darkmode&sanitize=true" align=middle width=52.21473014999999pt height=26.76175259999998pt/> box. This was necessary because we computed things with polynomials that blow up to large numbers when evaluated on things greater than 1.

To use our discretized obstacle, we first have to scale, shift, and rotate it into the FRS frame:

```matlab
% get the shift and scaling
x0 = FRS.initial_x ;
y0 = FRS.initial_y ;
D = FRS.distance_scale ;

% transform the points
O_FRS = world_to_FRS(O_pts,A.state(:,end),x0,y0,D) ;
```

Note that the robot's state is passed in here as well. This is because the robot is always at 0 heading in the FRS frame. So, `world_to_FRS` first rotates all the points about the robot to 0 heading, then scales and shifts them.

Since we know our entire FRS lies within the <img src="/step_4_online_planning/tex/ad2444f8273c6541c5dc1fc5b6445401.svg?invert_in_darkmode&sanitize=true" align=middle width=52.21473014999999pt height=26.76175259999998pt/> box, we can also discard any discretized obstacle points that lie outside the box:

```matlab
O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;
```



#### Example 1.3: Map Discretized Obstacle to Trajectory Parameters

Now that we have our discretized obstacle, we can map it to the trajectory parameter space:

```matlab
% get FRS polynomial and variables
I = FRS.FRS_polynomial ;
k = FRS.k ;
z = FRS.z ;

% swap the speed and steer parameters for visualization purposes
I = subs(I,k,[k(2);k(1)]) ;

% evaluate FRS polynomial on obstacle points
I_k = msubs(I,z,O_FRS) ;
```



The variable `I_k` is a list of polynomials in <img src="/step_4_online_planning/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/>. For each of these polynomials, the 1-superlevel set contains all the trajectory parameters that could cause the TurtleBot to reach one of the discretized obstacle points. The nice part is that we can now use these `I_k` polynomials as constraints in a nonlinear optimization program over the trajectory parameters.

If you run `step_4_ex_1_map_obs_to_traj_params.m`, you'll see a plot like the following:

<img src="images/step_4_ex_1_img_2.png" width="700px"/>



Notice that there are artifacts near the boundaries of the <img src="/step_4_online_planning/tex/ad2444f8273c6541c5dc1fc5b6445401.svg?invert_in_darkmode&sanitize=true" align=middle width=52.21473014999999pt height=26.76175259999998pt/> box where the FRS polynomial starts to blow up in the middle subplot. We can get rid of those later on by ignoring the corners of the box, since those are definitely not reachable by the robot.

## 4.2 Trajectory Optimization

Now we'll solve the online trajectory optimization problem for a single planning iteration. Much of this is the same as Example 9 above. The code is in `example_10_trajectory_optimization.m`.

### Example 2

This example turns much of the previous example into functions, then calls MATLAB's generic nonlinear optimization tool, `fmincon`, to solve the following problem:
<p align="center"><img src="/step_4_online_planning/tex/4707fcdfb70730be9ebd54bddaa0149f.svg?invert_in_darkmode&sanitize=true" align=middle width=442.29279929999996pt height=46.68803205pt/></p>



The point <img src="/step_4_online_planning/tex/8b4f2d3e1de4d24f0d8d4b38af75c90b.svg?invert_in_darkmode&sanitize=true" align=middle width=31.02377849999999pt height=14.15524440000002pt/> is a desired location of the robot, and the point <img src="/step_4_online_planning/tex/297f41396e8158f6a9169cfb6d6d00a5.svg?invert_in_darkmode&sanitize=true" align=middle width=48.510364649999985pt height=24.65753399999998pt/> is the endpoint of a desired trajectory parameterized by <img src="/step_4_online_planning/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/>. We are using <img src="/step_4_online_planning/tex/a38d48f6e4e60fa5c567b920937dd86e.svg?invert_in_darkmode&sanitize=true" align=middle width=66.96320564999999pt height=22.465723500000017pt/> as the constraint to match the standard `fmincon` format for nonlinear inequality constraints.



#### Example 2.1: Setup

First create the robot and load the 0.5 — 1.0 m/s FRS:

```matlab
initial_speed = 0.5 ; % m/s
FRS = load('turtlebot_FRS_deg_10_v_0_0.5_to_1.0.mat') ;

A = turtlebot_agent ;
z_initial = [0;0;0] ; % initial (x,y,h)
A.reset([z_initial;initial_speed])
```

Now we'll create the desired waypoint:

```matlab
x_des = 0.75 ;
y_des = 0.5 ;
```

Also create an obstacle:

```matlab
obstacle_location = [1 ; 0] ; % (x,y)
obstacle_scale = 1.0 ;
N_vertices = 5 ;
obstacle_buffer = 0.05 ; % m
O = make_random_polygon(N_vertices,obstacle_location,obstacle_scale) ;
```



#### Example 2.2: Creating a Cost Function

We denote the cost function <img src="/step_4_online_planning/tex/4e0a330da614750eb556c58093edd4ae.svg?invert_in_darkmode&sanitize=true" align=middle width=179.73945329999998pt height=24.65753399999998pt/> . To get the point <img src="/step_4_online_planning/tex/297f41396e8158f6a9169cfb6d6d00a5.svg?invert_in_darkmode&sanitize=true" align=middle width=48.510364649999985pt height=24.65753399999998pt/> for any <img src="/step_4_online_planning/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/>, we precompute the endpoint parameterized by <img src="/step_4_online_planning/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/>, then plug it into a cost. This is done in the script `create_turtlebot_cost_function.m`, which also computes gradients of the cost function. This script produces two functions, named `turtlebot_cost` and `turtlebot_cost_grad`.

We then format the cost and gradient for use with `fmincon` as follows. Note, **don't execute the following lines!** These lines are used in the function `turtlebot_cost_for_fmincon`:

```matlab
function [c, gc] = turtlebot_cost_for_fmincon(k,FRS,wp_local,start_tic,timeout)
    % evaluate cost and gradient
    c = turtlebot_cost(k(1),k(2),FRS.w_max,FRS.v_range(2),wp_local(1),wp_local(2)) ;
    gc = turtlebot_cost_grad(k(1),k(2),FRS.w_max,FRS.v_range(2),wp_local(1),wp_local(2)) ;

    % perform timeout check
    if nargin > 3 && toc(start_tic) > timeout
        error('Timed out while evaluating cost function!')
    end
end
```



We pass this to `fmincon` in the example script as follows (run these lines):

```matlab
% create waypoint from desired location
z_goal = [x_des; y_des] ;

% transform waypoint to robot's local coordinates
z_goal_local = world_to_local(A.state(:,end),z_goal) ;

% use waypoint to make cost function
cost = @(k) turtlebot_cost_for_fmincon(k,FRS,z_goal_local) ;
```



#### Example 2.3: Creating the Constraints

Now we'll do the same obstacle discretization and evaluation of the FRS polynomial as in Example 9 above. Here, we've wrapped up much of the code into handy-dandy functions. First, discretize the obstacle:

```matlab
point_spacing = compute_turtlebot_point_spacings(A.footprint,obstacle_buffer) ;
[O_FRS, O_buf, O_pts] = compute_turtlebot_discretized_obs(O,...
                    A.state(:,end),obstacle_buffer,point_spacing,FRS) ;
```



Now, we'll break the FRS polynomial into a simpler representation than the `msspoly` that we were using above. This is because, for online planning, we need to compute the constraints faster than the spotless function `msubs` can operate. Also note that `fmincon` treats constraints as feasible when they are negative, hence the use of <img src="/step_4_online_planning/tex/4d1719a25e4b38db1fbea7ddf5f1a886.svg?invert_in_darkmode&sanitize=true" align=middle width=36.82636649999999pt height=22.465723500000017pt/> as noted above.

```matlab
% get FRS polynomial and variables
FRS_msspoly = FRS.FRS_polynomial - 1 ; % the -1 is really important!
k = FRS.k ;
z = FRS.z ;

% decompose polynomial into simplified structure (this speeds up the
% evaluation of the polynomial on obstacle points)
FRS_poly = get_FRS_polynomial_structure(FRS_msspoly,z,k) ;
```

This decomposition just turns the FRS polynomial into a matrix of powers and a matrix of coefficients. Then, polynomial evaluation just requires matrix operations that can run super fast.

Now we can create the nonlinear constraint function. Note that this uses some functions in the RTD repository that take advantage of the decomposed polynomial. Take a look in `turtlebot_nonlcon_for_fmincon.m` for more details.

```matlab
% swap the speed and steer parameters for visualization purposes
FRS_poly_viz = subs(FRS_msspoly,k,[k(2);k(1)]) ;

% evaluate the FRS polynomial structure input on the obstacle points to get
% the list of constraint polynomials
cons_poly = evaluate_FRS_polynomial_on_obstacle_points(FRS_poly,O_FRS) ;

% get the gradient of the constraint polynomials
cons_poly_grad = get_constraint_polynomial_gradient(cons_poly) ;

% create nonlinear constraint function for fmincon
nonlcon = @(k) turtlebot_nonlcon_for_fmincon(k,cons_poly,cons_poly_grad) ;
```



This gives us the nonlinear constraints. We also need to bound the space <img src="/step_4_online_planning/tex/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode&sanitize=true" align=middle width=15.13700594999999pt height=22.465723500000017pt/>. Recall that <img src="/step_4_online_planning/tex/5537cda6dac51f6d1e672a0f75f3746e.svg?invert_in_darkmode&sanitize=true" align=middle width=89.26936094999998pt height=26.76175259999998pt/>. However, we also chose a limit that <img src="/step_4_online_planning/tex/6957ec2c8c11110e80efe4f039ecd7a0.svg?invert_in_darkmode&sanitize=true" align=middle width=122.81051144999998pt height=24.65753399999998pt/> m/s, which we need to enforce in terms of upper and lower bounds on <img src="/step_4_online_planning/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/>. We do this as follows:

```matlab
% create bounds for yaw rate
k_1_bounds = [-1,1] ;

% create bounds for speed
v_0 = initial_speed ;
v_max = FRS.v_range(2) ;
v_des_lo = max(v_0 - FRS.delta_v, FRS.v_range(1)) ;
v_des_hi = min(v_0 + FRS.delta_v, FRS.v_range(2)) ;
k_2_lo = (v_des_lo - v_max/2)*(2/v_max) ;
k_2_hi = (v_des_hi - v_max/2)*(2/v_max) ;
k_2_bounds = [k_2_lo, k_2_hi] ;

% combine bounds
k_bounds = [k_1_bounds ; k_2_bounds] ;
```



#### Example 2.4: Trajectory Optimization

Now we can call `fmincon`! Note that we've chosen these options to help `fmincon` solve faster. You can read more about these [here](https://www.mathworks.com/help/optim/ug/fmincon.html).

```matlab
% create initial guess
initial_guess = zeros(2,1) ;

% create optimization options
options =  optimoptions('fmincon','MaxFunctionEvaluations',1e5,'MaxIterations',1e5,...
                'OptimalityTolerance',1e-3','CheckGradients',false,...
                'FiniteDifferenceType','central','Diagnostics','off',...
                'SpecifyConstraintGradient',true,...
                'SpecifyObjectiveGradient',true);

% call fmincon
[k_opt,~,exitflag] = fmincon(cost,...
                            initial_guess,...
                            [],[],... % linear inequality constraints
                            [],[],... % linear equality constraints
                            k_bounds(:,1),... % lower bounds
                            k_bounds(:,2),... % upper bounds
                            nonlcon,...
                            options) ;
                        
% check the exitflag
if exitflag < 0
    k_opt = [] ;
end
```

Depending on the random obstacle, the problem will either be feasible or not. In the case that it is, we can now use all the plotting stuff from Example 9 and see what things look like. You'll see something like this:

<img src="images/step_4_ex_2_img_1.png" width="700px"/>

Now that we can do a single planning iteration, we can wrap everything up to run in the loop for online planning.



## 4.3 Running a Simulation

You can run a simulation with `run_turtlebot_simulation.m`. We'll briefly walk through the code here. More details on the simulator framework are in the tutorial [extras](https://github.com/skousik/RTD_tutorial/tree/master/step_5_extras/extra_2_writing_an_RTD_planner).

We have wrapped up the trajectory optimization procedure above in a `planner` class that inherits the generic RTD planner class as follows (don't run this line):

```matlab
turtlebot_RTD_planner_static_subclass < generic_RTD_planner
```



There's a lot going on in this class, but the gist is that its `replan` method gets called at every planning iteration to attempt trajectory optimization. Let's set up a simulation to see this working. First, the setup:

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

P = turtlebot_RTD_planner_static_subclass('verbose',verbose_level,'buffer',buffer,...
                                 't_plan',t_plan,'t_move',t_move) ;

W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',0.25,...
                     'verbose',verbose_level,'goal_radius',goal_radius,...
                     'obstacle_size_bounds',obstacle_size_bounds) ;
```

Finally, create the simulator, and run a simulation:

```matlab
S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',30,'max_sim_iterations',60) ;
              
S.run ;
```

You should see the TurtleBot move around in a box-shaped world with randomly-placed box obstacles. It will try to reach the goal in the green circle.

Note that this sets the `allow_replan_errors` property to `true` for the simulator. In other words, if the planner errors, the simulation will come screeching to a halt with an error. This is super useful for debugging, but should be set to `false` for, e.g., running hundreds of simulations.

There's a _lot_ going on behind the scenes in the simulator. Explanatory details are in the [extras](https://github.com/skousik/RTD_tutorial/tree/master/step_5_extras/extra_1_simulator_overview). The gist of the simulation loop is as follows, in pseudocode:

```C++
planner.old_plan <-- "agent stays stopped" // initialize old plan

while agent not at goal or crashed
    agent_info <-- agent.get_info() // info such as the current state
    world_info <-- world.get_info(agent_info) // info such as obstacles

    try // try to find a new plan with trajectory optimization as above
        new_plan <-- planner.replan(agent_info,world_info,planning_timeout)
        planner.old_plan <- new_plan
    catch // the planner errors if it can't plan within the timeout
        new_plan <-- planner.old_plan
    end

agent.move(new_plan)
end
```



That ends this tutorial about Reachability-based Trajectory Design. Thanks for reading!

##### [**Go back to tutorial home page**](https://github.com/skousik/RTD_tutorial).

##### [Learn how to write an RTD planner in excruciating detail](https://github.com/skousik/RTD_tutorial/tree/master/step_5_extras/extra_2_writing_an_RTD_planner)

<img src="/step_4_online_planning/tex/ac4bb7b235a967476d0ec663bb9588c2.svg?invert_in_darkmode&sanitize=true" align=middle width=4.5662248499999905pt height=14.15524440000002pt/>