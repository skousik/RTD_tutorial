# Step 3: Forward Reachable Set Computation

#### [Previous step: computing tracking error](https://github.com/skousik/RTD_tutorial/tree/master/step2_error_function)

Now that we have a [dynamic model](https://github.com/skousik/RTD_tutorial/tree/master/step1_desired_trajectories) and a [tracking error function](https://github.com/skousik/RTD_tutorial/tree/master/step2_error_function), we can compute a Forward-Reachable Set (FRS) for the TurtleBot. Note that there are also simple examples of the FRS computation in the [RTD repository](https://github.com/ramvasudevan/RTD/tree/master/examples/offline_FRS_computation).

## 3.1 Summary

In this step, we use sums-of-squares (SOS) programming to conservatively approximate an indicator function the FRS of the TurtleBot. We could also use other reachability methods, but this one lets us represent the FRS with polynomials, which end up being useful for online planning in the next step.

### Mathy Overview

Note, you can skip this mathy bit if you want. The important takeaways are listed in the "Goals for This Step" below.

We define the FRS as follows. Consider all points in space that the TurtleBot's body can reach while tracking any of our parameterized trajectories. The FRS, denoted <img src="/step3_FRS_computation/tex/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode&sanitize=true" align=middle width=12.85392569999999pt height=22.465723500000017pt/>, is all such points, paired with the corresponding trajectory parameters that caused those points to be reached. In other words, if <img src="/step3_FRS_computation/tex/7392a8cd69b275fa1798ef94c839d2e0.svg?invert_in_darkmode&sanitize=true" align=middle width=38.135511149999985pt height=24.65753399999998pt/> is a point that _any point_ on the robot reaches while tracking <img src="/step3_FRS_computation/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/>, then the point <img src="/step3_FRS_computation/tex/340224300a4ba094f276aad0e7964f95.svg?invert_in_darkmode&sanitize=true" align=middle width=54.51675569999999pt height=24.65753399999998pt/> is in <img src="/step3_FRS_computation/tex/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode&sanitize=true" align=middle width=12.85392569999999pt height=22.465723500000017pt/>. We write more formally next.

First, let <img src="/step3_FRS_computation/tex/91f17f2d1c679df5b3e3d782966d5b8d.svg?invert_in_darkmode&sanitize=true" align=middle width=130.678845pt height=26.76175259999998pt/> denote the trajectory-producing model. The space <img src="/step3_FRS_computation/tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode&sanitize=true" align=middle width=11.889314249999991pt height=22.465723500000017pt/> is time as before. The space <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/> is the Cartesian plane containing points denoted <img src="/step3_FRS_computation/tex/7392a8cd69b275fa1798ef94c839d2e0.svg?invert_in_darkmode&sanitize=true" align=middle width=38.135511149999985pt height=24.65753399999998pt/>. The space <img src="/step3_FRS_computation/tex/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode&sanitize=true" align=middle width=15.13700594999999pt height=22.465723500000017pt/> is the trajectory parameter space, so we can write <img src="/step3_FRS_computation/tex/8ca62ba4029ee91ab7699cb6729b59d6.svg?invert_in_darkmode&sanitize=true" align=middle width=118.17712499999998pt height=24.65753399999998pt/>. Then, the model is:
<p align="center"><img src="/step3_FRS_computation/tex/25a99f7aac484a2091ea7007d2d42cef.svg?invert_in_darkmode&sanitize=true" align=middle width=220.6505334pt height=39.452455349999994pt/></p>


as in [Appendix 1.A](https://github.com/skousik/RTD_tutorial/tree/master/step1_desired_trajectories), where we used the fact that the desired yaw rate <img src="/step3_FRS_computation/tex/aa90653a26bc63b138fb304972d81589.svg?invert_in_darkmode&sanitize=true" align=middle width=15.11042279999999pt height=22.831056599999986pt/> is fixed over <img src="/step3_FRS_computation/tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode&sanitize=true" align=middle width=11.889314249999991pt height=22.465723500000017pt/> to get rid of the heading dimension <img src="/step3_FRS_computation/tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode&sanitize=true" align=middle width=8.17352744999999pt height=22.831056599999986pt/>. This model works for _any point_ on the robot and expresses the robot's rigid body motion. The point <img src="/step3_FRS_computation/tex/5396bb939ac02109c36b48c29f7c9ce1.svg?invert_in_darkmode&sanitize=true" align=middle width=84.78306869999999pt height=24.65753399999998pt/> is the robot's center of mass at the beginning of any desired trajectory (i.e., when <img src="/step3_FRS_computation/tex/477a717e18587a5e8605780ca167c322.svg?invert_in_darkmode&sanitize=true" align=middle width=36.07293689999999pt height=21.18721440000001pt/>).

Now we can write a definition for <img src="/step3_FRS_computation/tex/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode&sanitize=true" align=middle width=12.85392569999999pt height=22.465723500000017pt/>. Recall that the tracking error function is denoted <img src="/step3_FRS_computation/tex/2d01c15095f2f5dbf50b0a3c967497a1.svg?invert_in_darkmode&sanitize=true" align=middle width=61.575138899999985pt height=26.76175259999998pt/>, and we write it <img src="/step3_FRS_computation/tex/69a8d98b45d37ba8a111815cbbce31b0.svg?invert_in_darkmode&sanitize=true" align=middle width=82.29823964999999pt height=24.65753399999998pt/> for each of the position states. Let <img src="/step3_FRS_computation/tex/9ea8a8d8d55375d9e1893a59f2241094.svg?invert_in_darkmode&sanitize=true" align=middle width=137.41609529999997pt height=24.65753399999998pt/> denote arbitrary "disturbance" functions that are _absolutely integrable_:
<p align="center"><img src="/step3_FRS_computation/tex/ea725162aae1293f076631e147b7f5cf.svg?invert_in_darkmode&sanitize=true" align=middle width=301.2497631pt height=37.3519608pt/></p>


We can use <img src="/step3_FRS_computation/tex/64ddb675e322bf1281650681445f5914.svg?invert_in_darkmode&sanitize=true" align=middle width=16.01033609999999pt height=22.831056599999986pt/> and <img src="/step3_FRS_computation/tex/885839bf3fa334609583ddbd937afdfb.svg?invert_in_darkmode&sanitize=true" align=middle width=15.63556994999999pt height=22.831056599999986pt/> to add the tracking error to <img src="/step3_FRS_computation/tex/190083ef7a1625fbc75f243cffb9c96d.svg?invert_in_darkmode&sanitize=true" align=middle width=9.81741584999999pt height=22.831056599999986pt/>, which lets use define the FRS:
<p align="center"><img src="/step3_FRS_computation/tex/7713ebefc1497d7f70135f405da06c46.svg?invert_in_darkmode&sanitize=true" align=middle width=354.10553475pt height=65.7540015pt/></p>


where <img src="/step3_FRS_computation/tex/db85bd6dfbbcc6817fc7960910b43296.svg?invert_in_darkmode&sanitize=true" align=middle width=17.77402769999999pt height=22.465723500000017pt/> is the set containing the robot's body at <img src="/step3_FRS_computation/tex/477a717e18587a5e8605780ca167c322.svg?invert_in_darkmode&sanitize=true" align=middle width=36.07293689999999pt height=21.18721440000001pt/>, and the notation <img src="/step3_FRS_computation/tex/c0463eeb4772bfde779c20d52901d01b.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=14.611911599999981pt/> denotes the elementwise product:
<p align="center"><img src="/step3_FRS_computation/tex/1ae6831711d8e1f0131be221eb2de1aa.svg?invert_in_darkmode&sanitize=true" align=middle width=176.17549125pt height=39.452455349999994pt/></p>


### Goals for This Step

The key takeaways from the math are the following. We use <img src="/step3_FRS_computation/tex/190083ef7a1625fbc75f243cffb9c96d.svg?invert_in_darkmode&sanitize=true" align=middle width=9.81741584999999pt height=22.831056599999986pt/> to denote the trajectory-producing model, and <img src="/step3_FRS_computation/tex/db85bd6dfbbcc6817fc7960910b43296.svg?invert_in_darkmode&sanitize=true" align=middle width=17.77402769999999pt height=22.465723500000017pt/> to denote the robot's body or "footprint" at the beginning of any planning iteration. We use <img src="/step3_FRS_computation/tex/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode&sanitize=true" align=middle width=15.13700594999999pt height=22.465723500000017pt/> to denote the space of trajectory parameters, and <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/> to denote the plane of <img src="/step3_FRS_computation/tex/7392a8cd69b275fa1798ef94c839d2e0.svg?invert_in_darkmode&sanitize=true" align=middle width=38.135511149999985pt height=24.65753399999998pt/> points. We proceed as follows:

1. Compute an FRS for a single desired trajectory (i.e. a single <img src="/step3_FRS_computation/tex/3bf25d9d50c08c7ec96446a7abb1c024.svg?invert_in_darkmode&sanitize=true" align=middle width=44.30350649999999pt height=22.831056599999986pt/>) without tracking error
2. Compute the FRS for all possible desired trajectories, plus tracking error

The first computation is to illustrate what the FRS looks like. The second computation gives us the FRS itself, to use for online planning.



## 3.2 FRS for a Single Trajectory

Now we'll compute the FRS for a single desired trajectory, to illustrate how the FRS computation is written as a SOS program with spotless and MOSEK. The following code is all in `example_6_FRS_with_fixed_traj_params.m`. We'll walk through it in a slightly different order here.

### Example 6

First, let's figure out what SOS program we even want to solve. The generic program is Program <img src="/step3_FRS_computation/tex/3aec9ee517c47acbb4dc2bb509d150af.svg?invert_in_darkmode&sanitize=true" align=middle width=26.851664399999994pt height=24.65753399999998pt/> on page 10 of this [super-duper long paper](https://arxiv.org/pdf/1809.06746.pdf). That program computes a polynomial <img src="/step3_FRS_computation/tex/5b5ca9b785667ee30100b8fce0f46933.svg?invert_in_darkmode&sanitize=true" align=middle width=110.97750674999997pt height=22.648391699999998pt/> for which, if <img src="/step3_FRS_computation/tex/d5110898628d6270c66a13ba784e9ff1.svg?invert_in_darkmode&sanitize=true" align=middle width=37.53429734999999pt height=24.65753399999998pt/> is in the FRS <img src="/step3_FRS_computation/tex/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode&sanitize=true" align=middle width=12.85392569999999pt height=22.465723500000017pt/>, then <img src="/step3_FRS_computation/tex/beacfec95d202f984345b778da4cf36a.svg?invert_in_darkmode&sanitize=true" align=middle width=79.8819846pt height=24.65753399999998pt/>. We want to do the same thing, but for just one <img src="/step3_FRS_computation/tex/3bf25d9d50c08c7ec96446a7abb1c024.svg?invert_in_darkmode&sanitize=true" align=middle width=44.30350649999999pt height=22.831056599999986pt/>, which will make a much smaller computation that can run on a laptop. So, let's first pick <img src="/step3_FRS_computation/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/> by running the following in the MATLAB command window:

```matlab
% chosen command
w_des = 0.5 ; % rad/s
v_des = 1.0 ; % m/s
```



#### Example 6.1: Scaling the SOS Program

Since we're computing with SOS polynomials, the first thing we need to do is scale the entire problem so that each variable lives in the domain <img src="/step3_FRS_computation/tex/699628c77c65481a123e3649944c0d51.svg?invert_in_darkmode&sanitize=true" align=middle width=45.66218414999998pt height=24.65753399999998pt/>, and so that time lives in <img src="/step3_FRS_computation/tex/e88c070a4a52572ef1d5792a341c0900.svg?invert_in_darkmode&sanitize=true" align=middle width=32.87674994999999pt height=24.65753399999998pt/>. Otherwise, the problem can become numerically unstable and won't converge nicely (think about what happens when you evaluate <img src="/step3_FRS_computation/tex/fee38d3614300eba71fc667002031f27.svg?invert_in_darkmode&sanitize=true" align=middle width=15.94753544999999pt height=26.76175259999998pt/> on <img src="/step3_FRS_computation/tex/e3950ac13bd76a194c14a76a72834d33.svg?invert_in_darkmode&sanitize=true" align=middle width=39.53182859999999pt height=21.18721440000001pt/>). To scale the problem down, we'll figure out how "far" out trajectory-producing model travels given the command above. 

First, we'll scale time to <img src="/step3_FRS_computation/tex/acf5ce819219b95070be2dbeb8a671e9.svg?invert_in_darkmode&sanitize=true" align=middle width=32.87674994999999pt height=24.65753399999998pt/>. Recall that, in [Step 1.5](https://github.com/skousik/RTD_tutorial/tree/master/step1_desired_trajectories), we found <img src="/step3_FRS_computation/tex/9f070416fd3580cf4c34c63f1690716a.svg?invert_in_darkmode&sanitize=true" align=middle width=65.59935854999999pt height=21.18721440000001pt/> s as our time horizon to include a fail-safe maneuver. So, if we multiply <img src="/step3_FRS_computation/tex/190083ef7a1625fbc75f243cffb9c96d.svg?invert_in_darkmode&sanitize=true" align=middle width=9.81741584999999pt height=22.831056599999986pt/> by <img src="/step3_FRS_computation/tex/c0213ff9cc036054ae01949a656bca82.svg?invert_in_darkmode&sanitize=true" align=middle width=13.63596464999999pt height=20.221802699999984pt/>, then the dynamics are scaled by time, so that they'll go as far over <img src="/step3_FRS_computation/tex/034d0a6be0424bffe9a6e7ac9236c0f5.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=21.18721440000001pt/> normalized second as they would've originally over <img src="/step3_FRS_computation/tex/9b527e843dd83a6485c635f8c4366f78.svg?invert_in_darkmode&sanitize=true" align=middle width=29.22385289999999pt height=21.18721440000001pt/> s. Run the following lines to set up the timing variables:

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

Finally, we'll scale and shift the problem to be in <img src="/step3_FRS_computation/tex/699628c77c65481a123e3649944c0d51.svg?invert_in_darkmode&sanitize=true" align=middle width=45.66218414999998pt height=24.65753399999998pt/> in the <img src="/step3_FRS_computation/tex/332cc365a4987aacce0ead01b8bdcc0b.svg?invert_in_darkmode&sanitize=true" align=middle width=9.39498779999999pt height=14.15524440000002pt/> and <img src="/step3_FRS_computation/tex/deceeaf6940a8c7a5a02373728002b0f.svg?invert_in_darkmode&sanitize=true" align=middle width=8.649225749999989pt height=14.15524440000002pt/> coordinates (i.e., in the space <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/>):

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
<p align="center"><img src="/step3_FRS_computation/tex/c06ec8be67f776ce40276258790f38a1.svg?invert_in_darkmode&sanitize=true" align=middle width=347.95044405pt height=104.82191114999999pt/></p>


The decision variables in this program are the functions <img src="/step3_FRS_computation/tex/a9a3a4a202d80326bda413b5562d5cd1.svg?invert_in_darkmode&sanitize=true" align=middle width=13.242037049999992pt height=22.465723500000017pt/> and <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/>. This is different from the notation <img src="/step3_FRS_computation/tex/7676df17927aeb8cf3c0529814b86278.svg?invert_in_darkmode&sanitize=true" align=middle width=40.86000434999999pt height=24.65753399999998pt/> for the same decision variables in the [paper](https://arxiv.org/pdf/1809.06746.pdf) that we're referencing, to avoid confusion with the state <img src="/step3_FRS_computation/tex/6c4adbc36120d62b98deef2a20d5d303.svg?invert_in_darkmode&sanitize=true" align=middle width=8.55786029999999pt height=14.15524440000002pt/> and control input <img src="/step3_FRS_computation/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/>. We're using <img src="/step3_FRS_computation/tex/a9a3a4a202d80326bda413b5562d5cd1.svg?invert_in_darkmode&sanitize=true" align=middle width=13.242037049999992pt height=22.465723500000017pt/> to denote what is very similar to a [Lyapunov function](https://en.wikipedia.org/wiki/Lyapunov_function) for our dynamics, and <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> to denote what is very similar to an [indicator function](https://en.wikipedia.org/wiki/Indicator_function) on our FRS.

Also, note that the _total derivative_ of <img src="/step3_FRS_computation/tex/a9a3a4a202d80326bda413b5562d5cd1.svg?invert_in_darkmode&sanitize=true" align=middle width=13.242037049999992pt height=22.465723500000017pt/> is given by:
<p align="center"><img src="/step3_FRS_computation/tex/02adb0ec50c9f20dc425f22afae58cdb.svg?invert_in_darkmode&sanitize=true" align=middle width=473.64332069999995pt height=33.81208709999999pt/></p>


where <img src="/step3_FRS_computation/tex/f4868f51fc6d4d737c16f9c81af27293.svg?invert_in_darkmode&sanitize=true" align=middle width=90.72113984999999pt height=27.91243950000002pt/> for notational convenience, and <img src="/step3_FRS_computation/tex/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode&sanitize=true" align=middle width=9.075367949999992pt height=22.831056599999986pt/> is the chosen desired trajectory above. So, the first constraint in the program above tells us that <img src="/step3_FRS_computation/tex/a9a3a4a202d80326bda413b5562d5cd1.svg?invert_in_darkmode&sanitize=true" align=middle width=13.242037049999992pt height=22.465723500000017pt/> must be decreasing along trajectories of <img src="/step3_FRS_computation/tex/190083ef7a1625fbc75f243cffb9c96d.svg?invert_in_darkmode&sanitize=true" align=middle width=9.81741584999999pt height=22.831056599999986pt/>.

Note, this isn't a SOS program yet! This is an infinite-dimensional program, since the constraints have to hold for uncountably many <img src="/step3_FRS_computation/tex/2b2595d381c04d836f219b7837ded4c2.svg?invert_in_darkmode&sanitize=true" align=middle width=37.916549549999985pt height=22.465723500000017pt/> and <img src="/step3_FRS_computation/tex/d3f4e894021bcc0d88debbc6c3145196.svg?invert_in_darkmode&sanitize=true" align=middle width=70.62392369999999pt height=24.65753399999998pt/>. In addition, the decision variables as written are just generic functions. To turn this into a finite-dimensional SOS program, we specify as polynomials of finite degree on <img src="/step3_FRS_computation/tex/0992ba4780d729a0f09b874a1e8fae95.svg?invert_in_darkmode&sanitize=true" align=middle width=44.377776299999994pt height=22.465723500000017pt/> and <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/> respectively. This lets use represent the infinite number of constraints with a finite number of polynomial coefficients using a [beautiful math trick]([https://en.wikipedia.org/wiki/Stengle%27s_Positivstellensatz](https://en.wikipedia.org/wiki/Stengle's_Positivstellensatz)). Luckily, all that representation stuff is taken care of us in the background by spotless.



#### Example 6.3: Setting Stuff Up for the SOS Program

The first step to creating a SOS program with spotless is to set up the "indeterminates," which are not the decision variables, but rather are the variables that the decision variables are made out of:

```matlab
t = msspoly('t', 1) ; % time t
z = msspoly('z', 2) ; % states z = (x,y)
x = z(1) ;
y = z(2) ;
```

The second step is to specify the spaces <img src="/step3_FRS_computation/tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode&sanitize=true" align=middle width=11.889314249999991pt height=22.465723500000017pt/> and <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/> as semi-algebraic sets. In other words, we want to specify polynomials <img src="/step3_FRS_computation/tex/044056ec7f1f68545fd44d8e6c1a1bd3.svg?invert_in_darkmode&sanitize=true" align=middle width=19.00483364999999pt height=22.831056599999986pt/> and <img src="/step3_FRS_computation/tex/5d0af6d512d3896687daa7411001431e.svg?invert_in_darkmode&sanitize=true" align=middle width=19.23369194999999pt height=22.831056599999986pt/> that are positive on <img src="/step3_FRS_computation/tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode&sanitize=true" align=middle width=11.889314249999991pt height=22.465723500000017pt/> and <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/>:
<p align="center"><img src="/step3_FRS_computation/tex/4b647d9397449f7c2f8099db64815621.svg?invert_in_darkmode&sanitize=true" align=middle width=232.07965769999998pt height=43.1343561pt/></p>


But, we also want to scale <img src="/step3_FRS_computation/tex/2f118ee06d05f3c2d98361d9c30e38ce.svg?invert_in_darkmode&sanitize=true" align=middle width=11.889314249999991pt height=22.465723500000017pt/> to <img src="/step3_FRS_computation/tex/acf5ce819219b95070be2dbeb8a671e9.svg?invert_in_darkmode&sanitize=true" align=middle width=32.87674994999999pt height=24.65753399999998pt/> and <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/> to <img src="/step3_FRS_computation/tex/02365d9a1eafb4fa93ec35acf8c9025d.svg?invert_in_darkmode&sanitize=true" align=middle width=93.37899944999998pt height=26.76175259999998pt/>! So, we'll specify the sets like this:

```matlab
hT = t * (1 - t) ;
hZ = (z - [-1;-1]) .* ([1;1] - z) ;
```

Basically, we just defined quadratic functions that are positive where we want them to be. We'll also do something similar for <img src="/step3_FRS_computation/tex/db85bd6dfbbcc6817fc7960910b43296.svg?invert_in_darkmode&sanitize=true" align=middle width=17.77402769999999pt height=22.465723500000017pt/>, which is a circle containing the robot's entire footprint at <img src="/step3_FRS_computation/tex/477a717e18587a5e8605780ca167c322.svg?invert_in_darkmode&sanitize=true" align=middle width=36.07293689999999pt height=21.18721440000001pt/>:

```matlab
% create a circular footprint for the initial condition
h_Z0 = 1 - ((x - initial_x)/(footprint/distance_scale)).^2 + ...
         - ((y - initial_y)/(footprint/distance_scale)).^2 ;
```

This is a paraboloid in <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/> that is positive on <img src="/step3_FRS_computation/tex/db85bd6dfbbcc6817fc7960910b43296.svg?invert_in_darkmode&sanitize=true" align=middle width=17.77402769999999pt height=22.465723500000017pt/>.

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

Now we'll create <img src="/step3_FRS_computation/tex/a9a3a4a202d80326bda413b5562d5cd1.svg?invert_in_darkmode&sanitize=true" align=middle width=13.242037049999992pt height=22.465723500000017pt/> and <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> as our decision variables:

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

The last constraint is what forces <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> to be greater than or equal to <img src="/step3_FRS_computation/tex/034d0a6be0424bffe9a6e7ac9236c0f5.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=21.18721440000001pt/> on the FRS. Recall that <img src="/step3_FRS_computation/tex/a9a3a4a202d80326bda413b5562d5cd1.svg?invert_in_darkmode&sanitize=true" align=middle width=13.242037049999992pt height=22.465723500000017pt/> is decreasing along trajectories, and, by the second constraint, it also has to be negative on the set of initial conditions. So, by the last constraint, <img src="/step3_FRS_computation/tex/c9dd335ebf8f6df377ef056642179a06.svg?invert_in_darkmode&sanitize=true" align=middle width=127.53411329999999pt height=24.65753399999998pt/> means that <img src="/step3_FRS_computation/tex/6b78aa0639dad385795b3d822af3ce7d.svg?invert_in_darkmode&sanitize=true" align=middle width=59.805858749999985pt height=24.65753399999998pt/> on points that are reached by trajectories of the system (i.e., the FRS).

Finally, we create the cost function. Notice that it's just the integral of <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> over <img src="/step3_FRS_computation/tex/5b51bd2e6f329245d425b8002d7cf942.svg?invert_in_darkmode&sanitize=true" align=middle width=12.397274999999992pt height=22.465723500000017pt/>; in other words, our SOS program is trying to "shrinkwrap" <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> to fit around the FRS.

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

This takes about <img src="/step3_FRS_computation/tex/ecd7bf9509ecd1263b6534b1081bdb95.svg?invert_in_darkmode&sanitize=true" align=middle width=29.22385289999999pt height=21.18721440000001pt/> s to solve on a 2016 Macbook Pro 15" laptop.



#### Example 6.6: Results

We care about using <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/>, so let's get it:

```matlab
I_sol = sol.eval(I) ;
```

We want to see that <img src="/step3_FRS_computation/tex/21fd4e8eecd6bdf1a4d3d6bd1fb8d733.svg?invert_in_darkmode&sanitize=true" align=middle width=8.515988249999989pt height=22.465723500000017pt/> is greater than or equal to <img src="/step3_FRS_computation/tex/034d0a6be0424bffe9a6e7ac9236c0f5.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=21.18721440000001pt/> where the trajectory-producing model went. So, let's create the desired trajectory:

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

The green contour is the level set <img src="/step3_FRS_computation/tex/6b78aa0639dad385795b3d822af3ce7d.svg?invert_in_darkmode&sanitize=true" align=middle width=59.805858749999985pt height=24.65753399999998pt/>. The dark blue circle at <img src="/step3_FRS_computation/tex/e660f3b58b414524ec6f827411021073.svg?invert_in_darkmode&sanitize=true" align=middle width=36.52973609999999pt height=24.65753399999998pt/> is the initial condition set <img src="/step3_FRS_computation/tex/db85bd6dfbbcc6817fc7960910b43296.svg?invert_in_darkmode&sanitize=true" align=middle width=17.77402769999999pt height=22.465723500000017pt/>. Notice that both the FRS and the initial condition set are unscaled by `distance_scale` and unshifted by `initial_x` and `initial_y` by using the `'Scale'` and `'Offset'` arguments in the plotting function.

The desired trajectory for <img src="/step3_FRS_computation/tex/f063d7f0ae9a5f520626fda6c9fa41bc.svg?invert_in_darkmode&sanitize=true" align=middle width=170.94760814999998pt height=24.65753399999998pt/> is shown as the blue dashed line. Notice that it fits entirely inside the FRS contour. In other words, the trajectory-producing model is indeed inside the FRS.

The blue circle with an arrow is the robot, which executes a trajectory with braking. Notice that it just barely fits inside the FRS. If you vary the `initial_speed` variable and run the robot again, it'll end up somewhere else. Some initial conditions will cause the robot to leave the FRS contour (try `initial_speed = 1.5`). This means we definitely need to include tracking error.



## 3.3 Computing the FRS

Coming soon!



#### [Next step: online planning](https://github.com/skousik/RTD_tutorial/tree/master/step4_online_planning)

