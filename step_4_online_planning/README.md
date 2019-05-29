# Step 4: Online Planning

#### [Previous step: computing the FRS](https://github.com/skousik/RTD_tutorial/tree/master/step3_FRS_computation)

Details and code coming soon! Note that there is an example in the [RTD repository](https://github.com/ramvasudevan/RTD) for a Segway robot, which is really similar to the TurtleBot.

## 4.1 Summary

In this step, we use the computed FRS in the [previous step](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation) to plan trajectories for the TurtleBot online (i.e., at runtime). First, we take the entire FRS and "intersect" it with obstacles around the robot. This intersection results in all trajectory parameters that would cause a collision. Finally, we optimize over the remaining collision-free trajectory parameters. If none can be found, then we execute the fail-safe maneuver from the previous planning iteration.

### Mathy Overview

Recall that RTD uses a trajectory parameter space <img src="/step_4_online_planning/tex/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode&sanitize=true" align=middle width=15.13700594999999pt height=22.465723500000017pt/> to specify desired trajectories. These trajectories are chosen every <img src="/step_4_online_planning/tex/5ae1e561ec81f97d93ed9df0f76cab27.svg?invert_in_darkmode&sanitize=true" align=middle width=30.730753349999993pt height=20.221802699999984pt/> seconds in a receding-horizon way. Also recall that, in Step 3, we found a function <img src="/step_4_online_planning/tex/2aa485270dee077a664c5c167a652424.svg?invert_in_darkmode&sanitize=true" align=middle width=107.28262545pt height=22.648391699999998pt/> for which, if <img src="/step_4_online_planning/tex/6b8afc7d21c1f242a41f95fe80e39813.svg?invert_in_darkmode&sanitize=true" align=middle width=89.58900389999998pt height=24.65753399999998pt/>, then <img src="/step_4_online_planning/tex/a91b124f86681090d790147ababa8c23.svg?invert_in_darkmode&sanitize=true" align=middle width=76.18710329999999pt height=24.65753399999998pt/>.

Now, to pick a particular <img src="/step_4_online_planning/tex/3bf25d9d50c08c7ec96446a7abb1c024.svg?invert_in_darkmode&sanitize=true" align=middle width=44.30350649999999pt height=22.831056599999986pt/> at each planning iteration, given an arbitrary cost function <img src="/step_4_online_planning/tex/5aefecdd2b9b5de63115b522e5e53f46.svg?invert_in_darkmode&sanitize=true" align=middle width=76.97455094999998pt height=22.648391699999998pt/>, we attempt to solve the following optimization problem:
<p align="center"><img src="/step_4_online_planning/tex/07e2eb627d1f673638dca7bf9b879965.svg?invert_in_darkmode&sanitize=true" align=middle width=354.84585675pt height=16.438356pt/></p>



The good thing about this is, as we saw in the previous section, <img src="/step_4_online_planning/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> gives us a _conservative_ approximation of the FRS. So, as long as the constraint <img src="/step_4_online_planning/tex/cb92288bfbb4e7352d7a04c79c0f8e99.svg?invert_in_darkmode&sanitize=true" align=middle width=83.74419239999997pt height=24.65753399999998pt/> is obeyed for all <img src="/step_4_online_planning/tex/f93ce33e511096ed626b4719d50f17d2.svg?invert_in_darkmode&sanitize=true" align=middle width=8.367621899999993pt height=14.15524440000002pt/> in any obstacle, then we can prove that <img src="/step_4_online_planning/tex/62a1d6ae808b6d855355def103c4971f.svg?invert_in_darkmode&sanitize=true" align=middle width=15.81055739999999pt height=22.831056599999986pt/> is a collision-free trajectory (see [Lemma 15 on page 10 of this paper](https://arxiv.org/abs/1809.06746)).

The tricky part here is that, if an obstacle is a subset of <img src="/step_4_online_planning/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/>, then probably contains infinitely many points (for example, if the obstacle is a polygon). But this means that we have to evaluate <img src="/step_4_online_planning/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> on an infinite number of points to find <img src="/step_4_online_planning/tex/62a1d6ae808b6d855355def103c4971f.svg?invert_in_darkmode&sanitize=true" align=middle width=15.81055739999999pt height=22.831056599999986pt/>. In fact, we can actually do that — check out [Section III-B here](https://arxiv.org/abs/1705.00091). However, doing this kind of evaluation is way too slow for real time planning.

To avoid this evaluation of an infinite number of points, we instead prescribe a way to discretize obstacles into a finite number of points. This discretization is explained in excruciating detail in [Section 6 of this paper](https://arxiv.org/abs/1809.06746). The key takeaway is that, even though each obstacle is represented by only a finite number of points, _we keep the collision-free guarantee_ that was the whole point of computing the FRS with tracking error in the first place.

### Goals for This Step

To do online planning, we'll do the following:

1. Map obstacles to the trajectory parameter space in a single planning iteration
2. Solve a nonlinear optimization program to pick a collision-free trajectory in a single planning iteration
3. Put 1 and 2 together in a "planner" object that will run RTD online in the [simulator](https://github.com/skousik/simulator) framework



## 4.2 Mapping Obstacles to Trajectory Parameters

In this part, we will represent an obstacle in the trajectory parameter space. First, let's assume that obstacles are sensed and handed to us as **polygons**, which is reasonable for a sensor like a LIDAR. Then, we can use the discretization mentioned above in the mathy overview.

The important points of the discretization are as follows. First, we only need to discretize the obstacle's boundary, since we can't get into the obstacle without passing through the boundary. Second, we have to buffer the obstacle before discretizing, to compensate for the fact that our robot shouldn't pass between any two points of the discretized boundary. Third, we need to find a **point spacing**, denoted <img src="/step_4_online_planning/tex/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode&sanitize=true" align=middle width=7.87295519999999pt height=14.15524440000002pt/>, that tells us how finely to discretize the boundary. In other words, we want to make sure that the discrete points are spaced no farther than <img src="/step_4_online_planning/tex/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode&sanitize=true" align=middle width=7.87295519999999pt height=14.15524440000002pt/> apart along the boundary.

The code we'll go over here is all in `example_9_map_obs_to_traj_params.m`. We'll just cover the highlights.

### Example 9

In this example, we'll make a single polygonal obstacle in front of the robot, then map it to the parameter space using the FRS polynomial <img src="/step_4_online_planning/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> that acts like an indicator function on the FRS.

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



The variable `I_k` is a list of polynomials in <img src="/step_4_online_planning/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/>. For each of these polynomials, the 1-superlevel set contains all the trajectory parameters that could cause the TurtleBot to reach one of the discretized obstacle points. The nice part is that we can now use these `I_k` polynomials as constraints in a nonlinear optimization program over the trajectory parameters.

If you run the entire `example_9_map_obs_to_traj_params.m` script, you'll see a plot like the following:

<img src="images/image_2_for_example_9.png" width="700px"/>

Notice that there are artifacts near the boundaries of the <img src="/step_4_online_planning/tex/ad2444f8273c6541c5dc1fc5b6445401.svg?invert_in_darkmode&sanitize=true" align=middle width=52.21473014999999pt height=26.76175259999998pt/> box where the FRS polynomial starts to blow up. We can get rid of those later on by ignoring the corners of the box, since those are definitely not reachable by the robot.

## 4.3 Trajectory Optimization

Coming soon!

## 4.4 Making a Planner Class for Simulation

Coming soon!

## Appendix 4.A: Planner Class Overview

Coming soon!