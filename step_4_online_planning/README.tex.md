# Step 4: Online Planning

#### [Previous step: computing the FRS](https://github.com/skousik/RTD_tutorial/tree/master/step3_FRS_computation)

Details and code coming soon! Note that there is an example in the [RTD repository](https://github.com/ramvasudevan/RTD) for a Segway robot, which is really similar to the TurtleBot.

## 4.1 Summary

In this step, we use the computed FRS in the [previous step](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation) to plan trajectories for the TurtleBot online (i.e., at runtime). First, we take the entire FRS and "intersect" it with obstacles around the robot. This intersection results in all trajectory parameters that would cause a collision. Finally, we optimize over the remaining collision-free trajectory parameters. If none can be found, then we execute the fail-safe maneuver from the previous planning iteration.

### Mathy Overview

Recall that RTD uses a trajectory parameter space $K$ to specify desired trajectories. These trajectories are chosen every $t_{\mathrm{plan}}$ seconds in a receding-horizon way. Also recall that, in Step 3, we found a function $I: Z \times K \to \mathbb{R}$ for which, if $(z,k) \in \mathrm{FRS}$, then $I(z,k) \geq 1$.

Now, to pick a particular $k \in K$ at each planning iteration, given an arbitrary cost function $J: K \to \mathbb{R}$, we attempt to solve the following optimization problem:
$$
\begin{align}
k^* = \mathrm{argmin}_k \{J(k)~|~I(z,k) < 1\ \forall\ z \in \mathrm{obstacles}\}
\end{align}.
$$



The good thing about this is, as we saw in the previous section, $I$ gives us a _conservative_ approximation of the FRS. So, as long as the constraint $I(z,k^*) < 1$ is obeyed for all $z$ in any obstacle, then we can prove that $k^*$ is a collision-free trajectory (see [Lemma 15 on page 10 of this paper](https://arxiv.org/abs/1809.06746)).

The tricky part here is that, if an obstacle is a subset of $Z$, then probably contains infinitely many points (for example, if the obstacle is a polygon). But this means that we have to evaluate $I$ on an infinite number of points to find $k^*$. In fact, we can actually do that — check out [Section III-B here](https://arxiv.org/abs/1705.00091). However, doing this kind of evaluation is way too slow for real time planning.

To avoid this evaluation of an infinite number of points, we instead prescribe a way to discretize obstacles into a finite number of points. This discretization is explained in excruciating detail in [Section 6 of this paper](https://arxiv.org/abs/1809.06746). The key takeaway is that, even though each obstacle is represented by only a finite number of points, _we keep the collision-free guarantee_ that was the whole point of computing the FRS with tracking error in the first place.

### Goals for This Step

To do online planning, we'll do the following:

1. Map obstacles to the trajectory parameter space in a single planning iteration
2. Solve a nonlinear optimization program to pick a collision-free trajectory in a single planning iteration
3. Put 1 and 2 together in a "planner" object that will run RTD online in the [simulator](https://github.com/skousik/simulator) framework



## 4.2 Mapping Obstacles to Trajectory Parameters

In this part, we will represent an obstacle in the trajectory parameter space. First, let's assume that obstacles are sensed and handed to us as **polygons**, which is reasonable for a sensor like a LIDAR. Then, we can use the discretization mentioned above in the mathy overview.

The important points of the discretization are as follows. First, we only need to discretize the obstacle's boundary, since we can't get into the obstacle without passing through the boundary. Second, we have to buffer the obstacle before discretizing, to compensate for the fact that our robot shouldn't pass between any two points of the discretized boundary. Third, we need to find a **point spacing**, denoted $r$, that tells us how finely to discretize the boundary. In other words, we want to make sure that the discrete points are spaced no farther than $r$ apart along the boundary.

The code we'll go over here is all in `example_9_map_obs_to_traj_params.m`. We'll just cover the highlights.

### Example 9

In this example, we'll make a single polygonal obstacle in front of the robot, then map it to the parameter space using the FRS polynomial $I$ that acts like an indicator function on the FRS.

First, load the FRS and create a TurtleBot. We'll use the 0.0 — 0.5 m/s FRS as in Step 3.

```matlab
FRS = load('turtlebot_FRS_deg_10_v0_0.0_to_0.5.mat') ;
A = turtlebot_agent ;
```

Recall that the robot is initially at the origin. Now, let's make a random obstacle:

```matlab
obstacle_location = [1 ; 0] ; % (x,y)
obstacle_scale = 1.0 ;
N_vertices = 5 ;
O = make_random_polygon(N_vertices,obstacle_location,obstacle_scale) ;
```

#### Example 9.1: Obstacle Buffering and Discretization

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

<img src="images/image_1_for_example_9.png" width="400px"/>

#### Example 9.2: Mapping Obstacle to FRS Frame

Recall that the FRS is computed in a scaled and shfited coordinate frame, to make sure all the trajectories stay within the $[-1,1]^2$ box. This was necessary because we computed things with polynomials that blow up to large numbers when evaluated on things greater than 1.

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

Since we know our entire FRS lies within the $[-1,1]^2$ box, we can also discard any discretized obstacle points that lie outside the box:

```matlab
O_FRS = crop_points_outside_region(0,0,O_FRS,1) ;
```



#### Example 9.3: Map Discretized Obstacle to Trajectory Parameters

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



The variable `I_k` is a list of polynomials in $k$. For each of these polynomials, the 1-superlevel set contains all the trajectory parameters that could cause the TurtleBot to reach one of the discretized obstacle points. The nice part is that we can now use these `I_k` polynomials as constraints in a nonlinear optimization program over the trajectory parameters.

If you run the entire `example_9_map_obs_to_traj_params.m` script, you'll see a plot like the following:

<img src="images/image_2_for_example_9.png" width="700px"/>

Notice that there are artifacts near the boundaries of the $[-1,1]^2$ box where the FRS polynomial starts to blow up. We can get rid of those later on by ignoring the corners of the box, since those are definitely not reachable by the robot.

## 4.3 Trajectory Optimization

Coming soon!

## 4.4 Making a Planner Class for Simulation

Coming soon!

## Appendix 4.A: Planner Class Overview

Coming soon!