# Step 3: Forward Reachable Set Computation

#### [Previous step: computing tracking error](https://github.com/skousik/RTD_tutorial/tree/master/step_2_error_function)

Now that we have a [dynamic model](https://github.com/skousik/RTD_tutorial/tree/master/step1_desired_trajectories) and a [tracking error function](https://github.com/skousik/RTD_tutorial/tree/master/step2_error_function), we can compute a Forward-Reachable Set (FRS) for the TurtleBot. Note that there are also simple examples of the FRS computation in the [RTD repository](https://github.com/ramvasudevan/RTD/tree/master/examples/offline_FRS_computation).

## 3.1 Summary

In this step, we use sums-of-squares (SOS) programming to conservatively approximate an indicator function the FRS of the TurtleBot. We could also use other reachability methods, but this one lets us represent the FRS with polynomials, which end up being useful for online planning in the next step.

### Mathy Overview

Note, you can skip this mathy bit if you want. The important takeaways are listed in the "Goals for This Step" below.

We define the FRS as follows. Consider all points in space that the TurtleBot's body can reach while tracking any of our parameterized trajectories. The FRS, denoted $F$, is all such points, paired with the corresponding trajectory parameters that caused those points to be reached. In other words, if $(x,y)$ is a point that _any point_ on the robot reaches while tracking $k$, then the point $(x,y,k)$ is in $F$. We write more formally next.

First, let $f: T \times Z \times K \to \R^2$ denote the trajectory-producing model. The space $T$ is time as before. The space $Z$ is the Cartesian plane containing points denoted $(x,y)$. The space $K$ is the trajectory parameter space, so we can write $k = (k_1,k_2) \in K$. Then, the model is:
$$
f(t,z,k) = \begin{bmatrix} k_2 - k_1\cdot(y - y_0) \\ k_2\cdot(x - x_0)\end{bmatrix}
$$


as in [Appendix 1.A](https://github.com/skousik/RTD_tutorial/tree/master/step1_desired_trajectories), where we used the fact that the desired yaw rate $k_1$ is fixed over $T$ to get rid of the heading dimension $\theta$. This model works for _any point_ on the robot and expresses the robot's rigid body motion. The point $(x_0,y_0) \in Z$ is the robot's center of mass at the beginning of any desired trajectory (i.e., when $t = 0$).

Now we can write a definition for $F$. Recall that the tracking error function is denoted $g: T \to \R^2$, and we write it $g = (g_x,g_y)$ for each of the position states. Let $d_x, d_y: T \to [-1,1]$ denote arbitrary "disturbance" functions that are _absolutely integrable_:
$$
\int_T |d_x(t)| dt < \infty\quad\mathrm{and}\quad\int_T |d_y(t)| dt < \infty
$$


We can use $d_x$ and $d_y$ to add the tracking error to $f$, which lets use define the FRS:
$$
\begin{align}
F = \Big\{(z,k)\ |\ &\exists\ t \in T,\ d_x,\ d_y,\ \mathrm{and}\ z_0 \in Z_0\ \mathrm{s.t} \\
&\dot{\tilde{z}} = f + g\circ [d_x, d_y]^\top~\mathrm{and}~\tilde{z}(t) = z\Big\},
\end{align}
$$


where $Z_0$ is the set containing the robot's body at $t = 0$, and the notation $\circ$ denotes the elementwise product:
$$
g\circ[d_x,d_y]^\top = \begin{bmatrix} g_x\cdot d_x \\ g_y \cdot d_y\end{bmatrix}.
$$


### Goals for This Step

The key takeaways from the math are the following. We use $f$ to denote the trajectory-producing model, and $Z_0$ to denote the robot's body or "footprint" at the beginning of any planning iteration. We use $K$ to denote the space of trajectory parameters, and $Z$ to denote the plane of $(x,y)$ points. We proceed as follows:

1. Compute an FRS for a single desired trajectory (i.e. a single $k \in K$) without tracking error
2. Compute the FRS for all possible desired trajectories, plus tracking error

The first computation is to illustrate what the FRS looks like. The second computation gives us the FRS itself, to use for online planning.



## 3.2 FRS for a Single Trajectory

Now we'll compute the FRS for a single desired trajectory, to illustrate how the FRS computation is written as a SOS program with spotless and MOSEK. The following code is all in `example_6_FRS_with_fixed_traj_params.m`. We'll walk through it in a slightly different order here.

### Example 6

First, let's figure out what SOS program we even want to solve. The generic program is Program $(D)$ on page 10 of this [super-duper long paper](https://arxiv.org/pdf/1809.06746.pdf). That program computes a polynomial $w: Z \times K \to \mathbb{R}$ for which, if $(z,k)$ is in the FRS $F$, then $w(z,k) \geq 1$. We want to do the same thing, but for just one $k \in K$, which will make a much smaller computation that can run on a laptop. So, let's first pick $k$ by running the following in the MATLAB command window:

```matlab
% chosen command
w_des = 0.5 ; % rad/s
v_des = 1.0 ; % m/s
```



#### Example 6.1: Scaling the SOS Program

Since we're computing with SOS polynomials, the first thing we need to do is scale the entire problem so that each variable lives in the domain $[-1,1]$, and so that time lives in $[0, 1]$. Otherwise, the problem can become numerically unstable and won't converge nicely (think about what happens when you evaluate $x^6$ on $x > 1$). To scale the problem down, we'll figure out how "far" out trajectory-producing model travels given the command above. 

First, we'll scale time to $[0,1]$. Recall that, in [Step 1.5](https://github.com/skousik/RTD_tutorial/tree/master/step1_desired_trajectories), we found $t_f = 0.95$ s as our time horizon to include a fail-safe maneuver. So, if we multiply $f$ by $t_f$, then the dynamics are scaled by time, so that they'll go as far over $1$ normalized second as they would've originally over $0.95$ s. Run the following lines to set up the timing variables:

```matlab
t_plan = 0.5 ;
t_f = 0.95 ;
t_stop = 2.61 ;
```

You could also just run `load('turtlebot_timing.mat')` in the command window to get these into your workspace.

Now, let's get the desired trajectory for the given command, to figure out how to scale the rest of the problem:

```matlab
% make the trajectory we are computing an FRS for; notice that the dynamics
% are time-scaled by t_f, then ode45 is called over the time horizon [0,1],
% since we are going to normalize time in the FRS computation
T = [0, 1] ;
U = [w_des, w_des ; v_des, v_des] ;
z0 = zeros(3,1) ;
[~,Z] = ode45(@(t,z) t_f.*turtlebot_trajectory_producing_model(t,z,T,U),T,z0) ;
Z = Z' ; % transpose to get column trajectory format
```

We can figure out how far this desired trajectory traveled, including the robot's footprint:

```matlab
% get the robot's footprint
A = turtlebot_agent ;
footprint = A.footprint ;

% figure out how far the robot traveled in x and y, and add the footprint
dx = max(Z(1,:)) - min(Z(1,:)) + footprint ;
dy = max(Z(2,:)) - min(Z(2,:)) + footprint ;
```

Finally, we'll scale and shift the problem to be in $[-1,1]$ in the $x$ and $y$ coordinates (i.e., in the space $Z$):

```matlab
% pick a scaling factor that makes the larger of dx and dy equal to 1, then
% make the center-of-mass dynamics start at (-0.5,0) when scaled
time_scale = t_f ;
distance_scale = max(dx,dy) ;
initial_x = -0.5 ;
initial_y =  0.0 ;
```



#### Example 6.2: Little FRS Computation Program

For this example, we don't care about tracking error. So, we just want to find everywhere the trajectory-producing model can reach while going at the desired yaw rate and speed. We'll do so with the following program:
$$
\begin{array}{clll}
{\underset{V, I}{\inf}} & {\int_{Z} I(z) d \lambda_{Z}} & {} & {}\\
{\text { s.t. }} & {\mathcal{L}_fV(t,z) \leq 0} & {} & {\forall~(t,z) \in T\times Z} \\
{} & {V(0,z) \leq 0} & {} & {\forall~z \in Z} \\
{} & {I(z) \geq 0} & {} & {\forall~z \in Z} \\
{} & {V(t,z) + I(z) - 1 \geq 0} & {} & {\forall~(t,z) \in T\times Z}
\end{array}
$$


The decision variables in this program are the functions $V$ and $I$. This is different from the notation $(v,w)$ for the same decision variables in the [paper](https://arxiv.org/pdf/1809.06746.pdf) that we're referencing, to avoid confusion with the state $v$ and control input $\omega$. We're using $V$ to denote what is very similar to a [Lyapunov function](https://en.wikipedia.org/wiki/Lyapunov_function) for our dynamics, and $I$ to denote what is very similar to an [indicator function](https://en.wikipedia.org/wiki/Indicator_function) on our FRS.

Also, note that the _total derivative_ of $V$ is given by:
$$
\mathcal{L}_fV(t,z) = \frac{\partial}{\partial t} V(t,z) + \frac{\partial}{\partial x}V(t,z)\cdot f_x(z,k) + \frac{\partial}{\partial x}V(t,z)\cdot f_y(z,k),
$$


where $f = [f_x, f_y]^\top$ for notational convenience, and $k$ is the chosen desired trajectory above. So, the first constraint in the program above tells us that $V$ must be decreasing along trajectories of $f$.

Note, this isn't a SOS program yet! This is an infinite-dimensional program, since the constraints have to hold for uncountably many $t \in T$ and $(x,y) \in Z$. In addition, the decision variables as written are just generic functions. To turn this into a finite-dimensional SOS program, we specify as polynomials of finite degree on $T\times Z$ and $Z$ respectively. This lets use represent the infinite number of constraints with a finite number of polynomial coefficients using a [beautiful math trick]([https://en.wikipedia.org/wiki/Stengle%27s_Positivstellensatz](https://en.wikipedia.org/wiki/Stengle's_Positivstellensatz)). Luckily, all that representation stuff is taken care of us in the background by spotless.



#### Example 6.3: Setting Stuff Up for the SOS Program

The first step to creating a SOS program with spotless is to set up the "indeterminates," which are not the decision variables, but rather are the variables that the decision variables are made out of:

```matlab
t = msspoly('t', 1) ; % time t
z = msspoly('z', 2) ; % states z = (x,y)
x = z(1) ;
y = z(2) ;
```

The second step is to specify the spaces $T$ and $Z$ as semi-algebraic sets. In other words, we want to specify polynomials $h_T$ and $h_Z$ that are positive on $T$ and $Z$:
$$
\begin{align}
T &= \{t \in \mathbb{R}~|~h_T(t) \geq 0\} \\
Z &= \{(x,y) \in \mathbb{R}^2~|~h_Z(x,y) \geq 0\}
\end{align}
$$


But, we also want to scale $T$ to $[0,1]$ and $Z$ to $[-1,1]^2 \subset \mathbb{R}^2$! So, we'll specify the sets like this:

```matlab
hT = t * (1 - t) ;
hZ = (z - [-1;-1]) .* ([1;1] - z) ;
```

Basically, we just defined quadratic functions that are positive where we want them to be. We'll also do something similar for $Z_0$, which is a circle containing the robot's entire footprint at $t = 0$:

```matlab
% create a circular footprint for the initial condition
h_Z0 = 1 - ((x - initial_x)/(footprint/distance_scale)).^2 + ...
         - ((y - initial_y)/(footprint/distance_scale)).^2 ;
```

This is a paraboloid in $Z$ that is positive on $Z_0$.

The last thing to do is make the trajectory-producing model scaled down for the domain we're considering:

```matlab
% create trajectory-producing model
scale = (time_scale/distance_scale) ;
f = scale*[v_des - w_des*(y - initial_y) ;
    + w_des*(x - initial_x)] ;
```



#### Example 6.4: Constructing the SOS Program Itself

Now, we can use our indeterminates and semi-algebraic set definitions to create the spotless program. First, initialize the program and indeterminates, and specify the polynomial degree to use:

```matlab
% initialize program and indeterminate variables
prog = spotsosprog;
prog = prog.withIndeterminate(t) ;
prog = prog.withIndeterminate(z) ;

degree = 6 ;
```

Now we'll create $V$ and $I$ as our decision variables:

```matlab
% create monomials for decision variable polynomials; v is like a Lyapunov
% function and w is like an indicator function on the FRS
V_mon = monomials([t;z], 0:degree) ;
I_mon = monomials(z, 0:degree) ;

% create the decision variable polynomials
[prog, V, ~] = prog.newFreePoly(V_mon) ;
[prog, I, I_coeff] = prog.newFreePoly(I_mon) ;
```

Next, we need to create the constraints of the SOS program. Luckily, spotless makes this super easy:

```matlab
% create variables for the constraints of the program (D)
t0 = 0 ;
V0 = subs(V,t,t0) ;
dVdt = diff(V,t) ;
dVdz = diff(V,z) ;
LfV = dVdt + dVdz*f ;

% define each constraint from program (D), ignoring the ones containing q
% (which represents the tracking error)

% -LfV > 0 on T x Z
prog = sosOnK(prog, -LfV, [t;z], [h_T; h_Z], degree) ;

% -V(0,.) > 0 on Z0
prog = sosOnK(prog, -V0, z, h_Z0, degree) ;

% I > 0 on Z
prog = sosOnK(prog, I, z, h_Z, degree) ;

% V(t,.) + I - 1 > 0 on T x Z
prog = sosOnK(prog, V + I - 1, [t;z], [h_T; h_Z], degree) ;
```

The last constraint is what forces $I$ to be greater than or equal to $1$ on the FRS. Recall that $V$ is decreasing along trajectories, and, by the second constraint, it also has to be negative on the set of initial conditions. So, by the last constraint, $I(z) > 1 - V(t,z)$ means that $I(z) \geq 1$ on points that are reached by trajectories of the system (i.e., the FRS).

Finally, we create the cost function. Notice that it's just the integral of $I$ over $Z$; in other words, our SOS program is trying to "shrinkwrap" $I$ to fit around the FRS.

```matlab
% define the cost function (the integral of I over the domain Z)
int_Z = boxMoments(z, [-1;-1], [1;1]) ; % this integrates I over Z
obj = int_Z(I_mon)' * I_coeff ; 
```



#### Example 6.5: Solving the SOS Program

Really, all the effort is spent setting stuff up. To solve the program, first create options for the solver:

```matlab
options = spot_sdp_default_options() ;
options.verbose = 1 ;
options.domain_size = 1;
options.solveroptions = [];
```

Now, solve it with MOSEK:

```matlab
sol = prog.minimize(obj, @spot_mosek, options) ;
```

This takes about $0.46$ s to solve on a 2016 Macbook Pro 15" laptop.



#### Example 6.6: Results

We care about using $I$, so let's get it:

```matlab
I_sol = sol.eval(I) ;
```

We want to see that $I$ is greater than or equal to $1$ where the trajectory-producing model went. So, let's create the desired trajectory:

```matlab
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;
```

It'll also be nice to see how close the robot gets while tracking this desired trajectory from a variety of initial speeds. Set that up as follows:

```matlab
initial_speed = 0.75 ; % m/s

% create the initial condition
z0 = [0;0;0;initial_speed] ; % (x,y,h,v)

% create the braking trajectory (i.e., include the fail-safe maneuver)
[T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

% move the robot
A.reset(z0)
A.move(T_brk(end),T_brk,U_brk,Z_brk) ;
```

Now, we can visualize the output:

```matlab
figure(1) ; clf ; axis equal ; hold on ;

% plot the initial condition
plot_2D_msspoly_contour(h_Z0,z,0,'LineWidth',1.5,'Color','b',...
    'Offset',-[initial_x;initial_y],'Scale',distance_scale)

% plot the FRS
plot_2D_msspoly_contour(I_sol,z,1,'LineWidth',1.5,'Color',[0.1 0.8 0.3],...
    'Offset',-[initial_x;initial_y],'Scale',distance_scale)

% plot the desired trajectory
plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)

% plot the agent
plot(A)
```

You should see something like this:

<img src="images/image_for_example_6.png" width="600px"/>

The green contour is the level set $I(z) \geq 1$. The dark blue circle at $(0,0)$ is the initial condition set $Z_0$. Notice that both the FRS and the initial condition set are unscaled by `distance_scale` and unshifted by `initial_x` and `initial_y` by using the `'Scale'` and `'Offset'` arguments in the plotting function.

The desired trajectory for $k = (0.5\ \mathrm{rad/s}, 1.0\ \mathrm{m/s})$ is shown as the blue dashed line. Notice that it fits entirely inside the FRS contour. In other words, the trajectory-producing model is indeed inside the FRS.

The blue circle with an arrow is the robot, which executes a trajectory with braking. Notice that it just barely fits inside the FRS. If you vary the `initial_speed` variable and run the robot again, it'll end up somewhere else. Some initial conditions will cause the robot to leave the FRS contour (try `initial_speed = 1.5`). This means we definitely need to include tracking error.



## 3.3 Computing the FRS

Coming soon!



#### [Next step: online planning](https://github.com/skousik/RTD_tutorial/tree/master/step_4_online_planning)
