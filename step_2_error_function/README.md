**TL;DR**: Run the script `step_2_compute_tracking_error_function.m`.

# Step 2: The Tracking Error Function

#### [Previous step: picking a trajectory-producing model](https://github.com/skousik/RTD_tutorial/tree/master/step_1_desired_trajectories)

The point of RTD is to perform receding-horizon planning in a provably correct way despite tracking error. To make such a guarantee, we need to hold on to the tracking error in some way. In this tutorial, we do so by representing the tracking error as a function of time.



## Summary

In this step, we use sampling to compute a tracking error function for the TurtleBot. Other methods, such as Sums-of-Squares or Hamilton-Jacobi reachability, could also be used to compute this error function in a more sophisticated way. However, for the purposes of this tutorial, we'll just sample.

### Mathy Overview

Recall that the TurtleBot is described by a **high-fidelity model**, and robot tracks trajectories created by the **trajectory-producing model**. The TurtleBot can't perfectly track the desired trajectories, because it has different dynamics from the robot. But, we can try to bound its tracking error with a **tracking error function**, which we'll denote <img src="/step_2_error_function/tex/ee06addff870fbf13ba2fc1625c447e4.svg?invert_in_darkmode&sanitize=true" align=middle width=78.01341569999998pt height=26.76175259999998pt/>. We want <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> to bound the absolute difference between these two models in the position states:
<p align="center"><img src="/step_2_error_function/tex/35569fab70bcbeac8b22b2fd3c4533cd.svg?invert_in_darkmode&sanitize=true" align=middle width=336.96491564999997pt height=128.36623635pt/></p>



where <img src="/step_2_error_function/tex/469bfc52df7f220df76c05b4b2a9badb.svg?invert_in_darkmode&sanitize=true" align=middle width=82.29823964999999pt height=24.65753399999998pt/> is the tracking error in each position state and <img src="/step_2_error_function/tex/7fafceb198a08f911de62716356f2a63.svg?invert_in_darkmode&sanitize=true" align=middle width=42.77285264999999pt height=14.15524440000002pt/> are controllers that do feedback about the trajectory parameterized by <img src="/step_2_error_function/tex/8dd6d948c7b7a6bef7250f4b3437d094.svg?invert_in_darkmode&sanitize=true" align=middle width=82.94898149999999pt height=24.65753399999998pt/>. In particular, these controllers are explained in Appendix 1.B of [the previous section](https://github.com/skousik/RTD_tutorial/tree/master/step_1_desired_trajectories).

We don't care about explicitly bounding the error in the <img src="/step_2_error_function/tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode&sanitize=true" align=middle width=8.17352744999999pt height=22.831056599999986pt/> or <img src="/step_2_error_function/tex/6c4adbc36120d62b98deef2a20d5d303.svg?invert_in_darkmode&sanitize=true" align=middle width=8.55786029999999pt height=14.15524440000002pt/> states, since obstacles only exist in <img src="/step_2_error_function/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> and <img src="/step_2_error_function/tex/deceeaf6940a8c7a5a02373728002b0f.svg?invert_in_darkmode&sanitize=true" align=middle width=8.649225749999989pt height=14.15524440000002pt/>. Also notice that both <img src="/step_2_error_function/tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode&sanitize=true" align=middle width=8.17352744999999pt height=22.831056599999986pt/> and <img src="/step_2_error_function/tex/6c4adbc36120d62b98deef2a20d5d303.svg?invert_in_darkmode&sanitize=true" align=middle width=8.55786029999999pt height=14.15524440000002pt/> influence the position, so <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> bounds them implicitly.

This tracking error function is useful as follows. First, assume that the high-fidelity model and trajectory-producing model have the same initial position and heading at the beginning of each planning iteration. Then, all spatial error between the two models can be bounded by the integral of <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> over time. This lets us use <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> in the [FRS computation](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation).

More formal explanations are available in the papers cited in the [references](https://github.com/skousik/RTD_tutorial#references).

### Goals for This Step

The big takeaway is, for any desired trajectory, the tracking error depends on two things: the robot's initial condition, and the choice of trajectory parameters. Recall that the initial condition of the high-fidelity and trajectory-producing model is assumed to be the same for everything but speed, so only the initial condition in speed affects the tracking error.

In this step, we'll do the following:

1. Inspect the tracking error for a single initial condition of the Turtlebot while tracking a single trajectory
2. Estimate the **tracking error function** <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> by looking at a variety of initial conditions and trajectories



## 2.1 A Single Instance of Tracking Error

First, we'll see how the initial speed affects tracking error. As you vary the initial speed relative to the commanded speed `v_des`, you'll see that the robot does better or worse at tracking.

### Example 1

This code is in `step_2_ex_1_tracking_error_single_traj.m`.

#### Example 1.1: An Example Trajectory

First, let's set up the situation and the robot

```matlab
% initial condition
initial_speed = 0.75 ; % m/s

% command bounds
w_des = 1.0 ; % rad/s ;
v_des = 1.0 ; % m/s

% create the turtlebot
A = turtlebot_agent() ;
A.reset([0;0;0;initial_speed])
```

Now, make the desired trajectory, and include braking for the robot.

```matlab
[T_des,U_des,Z_des] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des) ;
```

Track the desired trajectory:

```
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;
```

Now we can visualize the tracking error similar to how we did in Step 1:

```matlab
figure(1) ; hold on ; axis equal ;
plot_path(Z_des(1:2,:),'b--','LineWidth',1.5)
plot_path(Z,'b','LineWidth',1.5)
plot(A)
```

<img src="images/step_2_ex_1_img_1.png" width="500px"/>



#### Example 1.2: Computing the Tracking Error

We compute the tracking error by directly comparing the executed trajectory to the planned trajectory:

```matlab
% get the executed position trajectory
T = A.time ;
Z = A.state(A.position_indices,:) ;

% interpolate the executed trajectory to match the braking traj timing
pos = match_trajectories(T_des,T,Z) ;

% get the desired trajectory
pos_des = Z_des(1:2,:) ;

% compute the tracking error
pos_err = abs(pos - pos_des) ;
x_err = pos_err(1,:) ;
y_err = pos_err(2,:) ;
```

If we plot the tracking error, we see that it's under 2cm for the whole trajectory duration:

<img src="images/step_2_ex_1_img_2.png" width="500px"/>



#### Remark

If we're doing things with dynamic obstacles, we must include the fail-safe maneuver explicitly in the desired trajectory, because the FRS has to include time. We have written [a paper that shows how to do this](https://arxiv.org/abs/1902.02851).



## 2.2 Computing the Tracking Error Function

First, we'll compute the tracking error function for just a single initial speed, by varying our choice of commanded speed and yaw rate. Then, we'll compute the tracking error function over a range of speeds.

### Example 2

This code is available in `step_2_ex_2_tracking_error_single_init_speed.m`. It is nearly identical to the previous example, except we sample over the space of <img src="/step_2_error_function/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> and <img src="/step_2_error_function/tex/6c4adbc36120d62b98deef2a20d5d303.svg?invert_in_darkmode&sanitize=true" align=middle width=8.55786029999999pt height=14.15524440000002pt/> as follows. First, set up some bounds:

```matlab
% command bounds
w_min = -1.0 ; % rad/s
w_max =  1.0 ; % rad/s
delta_v = 0.25 ; % m/s

% number of samples in w and v
N_samples = 4 ;
```

Notice that we have chosen `delta_v` as 0.25 m/s. This means that, at any planning iteration, we are specifying that our desired speed must be within 0.25 m/s of our current speed. Since `t_plan` is 0.5 s, this is an acceleration constraint.

Now, we can create yaw and speed commands:

```matlab
% create yaw commands
w_vec = linspace(w_min,w_max,N_samples) ;

% create the feasible speed commands from the initial condition
v_vec = linspace(initial_speed - delta_v, initial_speed + delta_v, N_samples) ;
```

Finally, we iterate over the `w_vec` and `v_vec` and compute tracking error for each command. See the example script for this code. It produces the following figure:

<img src="images/step_2_ex_2_img_1.png" width="500px"/>

The black dashed lines are individual tracking error trajectories. We fit the tracking error function <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> as a polynomial such that the integral of <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> over time is greater than the max over all of the trajectories in <img src="/step_2_error_function/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> and in <img src="/step_2_error_function/tex/deceeaf6940a8c7a5a02373728002b0f.svg?invert_in_darkmode&sanitize=true" align=middle width=8.649225749999989pt height=14.15524440000002pt/>.



### Computing the Tracking Error Function

Now, we can compute the tracking error over a a range of initial speeds, which is done by the script `step_2_compute_tracking_error_function.m`. First, the user specifies a range of initial speeds:

```matlab
v_0_min = 1.0 ; % m/s
v_0_max = 1.5 ; % m/s
```

We sample this range of speeds, and compute the tracking error by taking the max of all tracking errors across all samples. The output looks like this:

<img src="images/step_2.2_img_1.png" width="500px"/>



This script fits the tracking error function <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> as a polynomial to upper bound the data. It then saves the polynomial coefficients and command bound data to the file `turtlebot_error_functions_v_0_1.0_to_1.5.mat` by default. The filename changes depending on the range of initial speeds. In the `step2_error_function/data/` folder, <img src="/step_2_error_function/tex/3cf4fbd05970446973fc3d9fa3fe3c41.svg?invert_in_darkmode&sanitize=true" align=middle width=8.430376349999989pt height=14.15524440000002pt/> has been precomputed for three ranges: 0.0 - 0.5 m/s, 0.5 - 1.0 m/s, and 1.0 - 1.5 m/s.

We'll use the tracking error function to compute an FRS over each initial speed range next.

#### [Next: computing the FRS](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation)

