**TL;DR**: Run `example_3_braking_trajectory.m`. The other examples are pretty straightforward too.

# Step 1: Picking a Trajectory-Producing Model

#### [Previous: tutorial summary](https://github.com/skousik/RTD_tutorial)

For the purposes of RTD, we refer a robot's state space model as a **high-fidelity model**, because we expect it to accurately describe the robot's motion. Recall that the Turtlebot's high-fidelity model has four states and two control inputs. So, including time, this model has seven **dimensions**. But, generating *correct* trajectories for systems with this many dimensions is often too slow for online planning. To avoid this issue, we use a **trajectory-producing model** that has fewer dimensions, but can still produce **desired trajectories** for the high-fidelity model to track.



## 1.1 Summary

In this step, we pick the trajectory-producing model. The goal is to reduce the number of dimensions as much as possible while still producing trajectories that the robot can track closely. In addition, remember that our definition of correct for the TurtleBot means that we care about avoiding obstacles (which exist in <img src="/step_1_desired_trajectories/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> and <img src="/step_1_desired_trajectories/tex/deceeaf6940a8c7a5a02373728002b0f.svg?invert_in_darkmode&sanitize=true" align=middle width=8.649225749999989pt height=14.15524440000002pt/>, but not in the robot's other states). So, our ideal trajectory-producing model should only have position as its states. To get to such a model, we'll do two things:
1. Get rid of the speed state, and pretend that it's a control input, to reduce the dimension by one
2. Replace control inputs with **trajectory parameters**

The last thing we'll do is make sure that the desired trajectories are of long enough duration for the robot to execute a **fail-safe maneuver** to bring it to a known-correct state (which means coming to a stop for the TurtleBot).



## 1.2 Dubins' Car

Recall that we use a unicycle model for the TurtleBot. If we get rid of the speed dimension, and replace acceleration with speed as our second control input, we end up with a model called a **Dubins' car**:
<p align="center"><img src="/step_1_desired_trajectories/tex/a417f6050fede18cdce43b0f0bc57919.svg?invert_in_darkmode&sanitize=true" align=middle width=145.87832655pt height=59.1786591pt/></p>



Note that <img src="/step_1_desired_trajectories/tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode&sanitize=true" align=middle width=8.17352744999999pt height=22.831056599999986pt/> is written as `h` in the code (for "heading"). We pick this as our trajectory-producing model because it's very similar to our high-fidelity model, but lower-dimensional, and because it can create meaningful and diverse paths for the TurtleBot to track in arbitrary environments.



## 1.3 Trajectory Parameters

Note that the control inputs can vary with time and the robot's states. In the case of the high-fidelity mode, this lets the robot track a desired trajectory using closed-loop feedback. But, if the trajectory-producing model can have any combination of speed and yaw rate at any time, the set of all possible desired trajectories is really big - and this can make computing the FRS tricky.

So, we want to reduce the "size" of the control input space for the trajectory-producing model. We do this by introducing **trajectory parameters**, which is why we say that RTD uses *parameterized* trajectories. We denote these parameters by <img src="/step_1_desired_trajectories/tex/8dd6d948c7b7a6bef7250f4b3437d094.svg?invert_in_darkmode&sanitize=true" align=middle width=82.94898149999999pt height=24.65753399999998pt/>, which give us desired yaw rate and speed, respectively. Then, we rewrite our Dubins' car model as follows:
<p align="center"><img src="/step_1_desired_trajectories/tex/b7fd75d9e63fb571e34c17e52105fb14.svg?invert_in_darkmode&sanitize=true" align=middle width=153.25283654999998pt height=78.9048876pt/></p>


Notice that the parameters are fixed (since their time derivative is 0). Of course, it doesn't make sense for them to be fixed for _all time_, since the robot probably won't do anything useful if its speed and yaw rate are fixed forever. Instead, we fix the control parameters over the finite planning time horizon of duration <img src="/step_1_desired_trajectories/tex/c0213ff9cc036054ae01949a656bca82.svg?invert_in_darkmode&sanitize=true" align=middle width=13.63596464999999pt height=20.221802699999984pt/>.

### Example 1

To understand how these trajectories look, let's create a desired trajectory with <img src="/step_1_desired_trajectories/tex/af6c80e33a666ccbd4ad31540ac57c74.svg?invert_in_darkmode&sanitize=true" align=middle width=53.744179499999994pt height=21.18721440000001pt/> rad/s and <img src="/step_1_desired_trajectories/tex/004273cb240d969e5bce673799ff9da5.svg?invert_in_darkmode&sanitize=true" align=middle width=51.48011549999999pt height=21.18721440000001pt/> m/s, starting from the initial state <img src="/step_1_desired_trajectories/tex/654de0324269221927923d504c5b6713.svg?invert_in_darkmode&sanitize=true" align=middle width=127.58738024999998pt height=24.65753399999998pt/>. Run the following code in the MATLAB command window:

```matlab
t_f = 1 ;
w_des = 0.5 ; % rad/s
v_des = 1.0 ; % m/s
[T,U,Z] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;
```

The desired trajectory is specified as a time array `T` (1-by-N), an input vector `U` (2-by-N), and the desired trajectory `Z` (4-by-N). We can look at this trajectory with the following code:

```
% get the x and y positions of the trajectory
x = Z(1,:) ;
y = Z(2,:) ;

% plot
figure(1) ;
plot(x,y,'b--','LineWidth',1.5)
axis equal
```

You should see the following:

<img src="images/image_for_example_1.png" width="500px"/>

This code is in `example_1_desired_trajectory.m` as well.



## 1.4 Tracking Desired Trajectories

Now, we can try tracking this desired trajectory! The following code is in `example_2_trajectory_tracking.m`.

### Example 2

First, let's create a TurtleBot:

```matlab
A = turtlebot_agent() ;
```

Note that this requires you to have the [simulator](github.com/skousik/simulator) repository on your MATLAB path. To use this 
agent" representation of the TurtleBot, first give it an initial condition of <img src="/step_1_desired_trajectories/tex/a362f086bf1a268c373d882b17f36d6b.svg?invert_in_darkmode&sanitize=true" align=middle width=171.76163069999998pt height=24.65753399999998pt/>:

```matlab
A.reset([0;0;0;0.5])
```

We can also take a look at the robot:

```
figure(1) ; clf ; axis equal ;
plot(A)
```

The robot has a circular **footprint**, and its heading is indicated by a dark arrow. It can track a desired trajectory by using its `move` method. First, create a desired trajectory:

```
t_f = 1 ;
w_des = 0.5 ; % rad/s
v_des = 1.0 ; % m/s
[T,U,Z] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% plot
hold on
plot(Z(1,:),Z(2,:),'b--','LineWidth',1.5)
```

Now, move the agent:

```
t_total = 0.75 ;
A.move(t_total,T,U,Z)
```

To see it in action, let's animate it:

```
A.animate()
```

You should see something like this once the animation completes:

<img src="images/image_for_example_2.png" width="500px"/>

Notice that the robot did not perfectly track the desired trajectory. This is because its initial speed is different from its desired speed. Note that the agent uses a low-level controller for trajectory tracking that is explained a bit more below, in Appendix 1.B.



## 1.5 Fail-Safe Maneuver

There is one key thing still missing. In every desired trajectory, we also need to include a **fail-safe maneuver**, which brings the robot to a known correct state. For the Turtlebot, staying stationary is always correct, since we only care about static obstacles. So, our fail-safe maneuver is braking to a stop. This means we need to include the distance required to stop in every desired trajectory.

Code for this part is in `step1_trajectories/scripts/compute_planning_time.m`. Since that script is pretty involved, we just point out the highlights here.

To create the fail-safe maneuver, we first need to understand how long it takes the robot to brake to a stop from its max speed, which we have picked as 1.5 m/s. We can command the robot to stop as follows:

```matlab
max_speed = 1.5 ;
t_brake = 5 ;
A = turtlebot_agent ; % get a fresh copy
A.reset([0;0;0;max_speed])
A.stop(t_brake)
```

Here, `t_brake` is the amount of time to apply the `stop` method, which commands 0 desired speed and yaw rate. It turns out that it takes about 2.61 s for the robot to come to a stop (which we count as a speed of < 1 mm/s).

Now, we can transform a regular desired trajectory into a desired trajectory with braking:

```matlab
% create the regular desired trajectory
t_f = 0.95 ; % s (see compute_planning_time.m for where this choice of t_f comes from)
w_des = 0.5 ;
v_des = 1.0 ;
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% add braking
t_plan = 0.5 ; % s
t_stop = 2.61 ; % s
[T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;
```

Let's see what happens when we apply this to the TurtleBot:

```matlab
% run the braking maneuver from an initial speed of 0.75 m/s
z0 = [0;0;0;0.75] ; % initial (x,y,h,v)
A.reset(z0)
A.move(T_brk(end),T_brk,U_brk,Z_brk)

% plot
figure(1) ; clf ; hold on ; axis equal ;
plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)
plot(A)

% animate
A.animate()
```

You should see the TurtleBot start to track the desired trajectory, then brake to a stop.

### Example 3

See `example_3_braking_trajectory.m`, which basically does everything that is written above. You can use that example code like this:

```
initial_speed = 1.2 ;
w_des = 0.5 ;
v_des = 0.7 ;
example_3_braking_trajectory(w_des,v_des,initial_speed)
```



Now that we have the robot tracking desired trajectories and braking, we can move on to computing a **tracking error function**.

#### [Next: computing tracking error](https://github.com/skousik/RTD_tutorial/tree/master/step_2_error_function)



## Appendix 1.A: Rigid Body Dynamics

Most robots aren't just point masses. However, the dynamics we write for them, and the desired trajectories we create, are often defined for point masses. To make a point mass trajectory feasible for a robot with a **body** (which has nonzero volume), we then need to use tricks like dilating obstacles before we plan the trajectory.

With RTD, we try to get around this issue by representing the motion of the robot's entire body in the desired trajectory. Suppose that <img src="/step_1_desired_trajectories/tex/d5ce7af04a5f115d28ff733f2c68ecc4.svg?invert_in_darkmode&sanitize=true" align=middle width=115.60669395pt height=24.65753399999998pt/> are differentiable trajectories of the robot's center of mass, and <img src="/step_1_desired_trajectories/tex/63e302efe61ebe533fe9bf97f51c439a.svg?invert_in_darkmode&sanitize=true" align=middle width=98.43014444999997pt height=24.65753399999998pt/> is a differentiable trajectory of the robot's heading. Then, assuming the robot is a rigid body, we can describe the motion of any point <img src="/step_1_desired_trajectories/tex/bf4645e786baf289adfe68fe608d3e69.svg?invert_in_darkmode&sanitize=true" align=middle width=47.35926029999999pt height=24.7161288pt/> on the robot's body with the following differential equation:
<p align="center"><img src="/step_1_desired_trajectories/tex/c3cd2c7b7f2c9024a22d57bd8af387c0.svg?invert_in_darkmode&sanitize=true" align=middle width=293.07016035pt height=40.960527299999995pt/></p>
Ideally, we would want to write down the dynamics of _every_ point on the robot's body this way, but, since you can think of a robot's body as a set in Euclidean space, its body typically contains an infinite number of points. In other words, we would need an infinite number of differential equations.

However, in our [reachability analysis (Step 3)](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation), we are able to compute the motion of every point on the robot's body by rewriting the Dubins' car in the following way. First, suppose that, at <img src="/step_1_desired_trajectories/tex/477a717e18587a5e8605780ca167c322.svg?invert_in_darkmode&sanitize=true" align=middle width=36.07293689999999pt height=21.18721440000001pt/>, the robot's center of mass is at <img src="/step_1_desired_trajectories/tex/a903f939f6fb649debe1561ecbb18c66.svg?invert_in_darkmode&sanitize=true" align=middle width=52.29465614999999pt height=24.65753399999998pt/>, and the robot's initial heading is <img src="/step_1_desired_trajectories/tex/bab17f1c90dbcda2fd355c88f6baacb1.svg?invert_in_darkmode&sanitize=true" align=middle width=45.22819289999998pt height=22.831056599999986pt/>. Then, the following differential equation will produce the trajectory of any point <img src="/step_1_desired_trajectories/tex/bf4645e786baf289adfe68fe608d3e69.svg?invert_in_darkmode&sanitize=true" align=middle width=47.35926029999999pt height=24.7161288pt/> on the body:
<p align="center"><img src="/step_1_desired_trajectories/tex/a58aa8b39e323167ed70b4ca0562c7f8.svg?invert_in_darkmode&sanitize=true" align=middle width=205.44055455pt height=39.452455349999994pt/></p>



This trick works because this differential equation describes a circular flow field about the rigid body's center of rotation, where the body is rotating at the (constant) rate <img src="/step_1_desired_trajectories/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> and the center of mass is initially traveling at the speed <img src="/step_1_desired_trajectories/tex/6c4adbc36120d62b98deef2a20d5d303.svg?invert_in_darkmode&sanitize=true" align=middle width=8.55786029999999pt height=14.15524440000002pt/> in the positive <img src="/step_1_desired_trajectories/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> direction.

Since the position and heading of the robot can be treated as starting at 0 for every planning iteration, we can use these circular flow field dynamics to create Dubins paths for the *entire body* of the robot in <img src="/step_1_desired_trajectories/tex/7392a8cd69b275fa1798ef94c839d2e0.svg?invert_in_darkmode&sanitize=true" align=middle width=38.135511149999985pt height=24.65753399999998pt/> coordinates, no matter what the shape of the robot is. This lets us compute the reachable set of the entire body, regardless of its shape, in Step 3. As a bonus, we got rid of the <img src="/step_1_desired_trajectories/tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode&sanitize=true" align=middle width=8.17352744999999pt height=22.831056599999986pt/> dimension!



## Appendix 1.B: Low-Level Controller

In `simulator`, each `agent` has the option of using a low-level controller (LLC), specified by the `agent.LLC` property. The Turtlebot agent in particular uses the `turtlebot_PD_LLC.m` LLC, which you can find in the [`simulator_files/`](https://github.com/skousik/turtlebot_RTD/tree/master/simulator_files) directory. This controller does PD (proportional-derivative) control about a desired trajectory's speed and yaw rate. The closed-loop system can be written:
<p align="center"><img src="/step_1_desired_trajectories/tex/6d46dad425660c882873584aa5b756b3.svg?invert_in_darkmode&sanitize=true" align=middle width=276.83134545pt height=78.9048876pt/></p>
given the desired speed and yaw rate commands. You can find this low-level controller in the agent with the following code:

```
A = turtlebot_agent ;
A.LLC % displays the LLC's properties
```

The default gains are: <img src="/step_1_desired_trajectories/tex/83c7347fe075f7e3998685dff1f80070.svg?invert_in_darkmode&sanitize=true" align=middle width=225.85052489999995pt height=22.831056599999986pt/>. This is expressed in the code as:

```
A.LLC.yaw_gain = 0 ;
A.LLC.yaw_rate_gain = 1 ;
A.LLC.speed_gain = 3 ;
A.LLC.accel_gain = 0 ;
```

Note that the acceleration gain <img src="/step_1_desired_trajectories/tex/eb4513540706477e80b47eb048eeaa9d.svg?invert_in_darkmode&sanitize=true" align=middle width=15.68825939999999pt height=22.831056599999986pt/> and yaw rate gain <img src="/step_1_desired_trajectories/tex/59edb4731438111eb9b889285d97715b.svg?invert_in_darkmode&sanitize=true" align=middle width=17.25200069999999pt height=22.831056599999986pt/>  determine the feedforward of the TurtleBot's acceleration and yaw rate inputs. You can play with the gains to make the TurtleBot track the desired trajectories really well, but we're leaving them as is to make sure the TurtleBot has some tracking error.

#### [Next step: computing tracking error](https://github.com/skousik/RTD_tutorial/tree/master/step_2_error_function)

