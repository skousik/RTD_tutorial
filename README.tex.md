**TL;DR**: [Start here with Step 1](https://github.com/skousik/RTD_tutorial/tree/master/step_1_desired_trajectories) if you don't want to read the overview.

# RTD Tutorial

This tutorial implements [Reachability-based Trajectory Design](https://github.com/ramvasudevan/RTD) (RTD) on a [TurtleBot](https://www.turtlebot.com/turtlebot2/) in simulation. RTD has been developed at the University of Michigan's [ROAHM Lab](http://www.roahmlab.com/).

Our choice of the TurtleBot was inspired by the work of our friends at UC Berkeley on Hamilton-Jacobi reachability, which can be found [here](https://abajcsy.github.io/safe_navigation/). In this work, we instead use Sums-of-Squares (SOS) programming for our reachability analysis. Our approach is detailed [here](https://arxiv.org/abs/1809.06746).

[Start here with Step 1](https://github.com/skousik/RTD_tutorial/tree/master/step_1_desired_trajectories) if you don't want to read the overview.

### Authors
Shreyas Kousik (skousik at umich.edu)

### License
This project is licensed under the MIT License.



## Installation Requirements

To run the code in this repository, you will need the following:
* MATLAB, ideally R2018a or newer
* [MOSEK](https://www.mosek.com/) for the offline FRS computation
* [simulator](https://github.com/skousik/simulator) for the online planning examples
* [spotless](https://github.com/spot-toolbox/spotless) for the offline FRS computation and the online examples
* [RTD](https://github.com/ramvasudevan/RTD) for the offline computation and the online examples

# Overview
RTD is a way to control a robot correctly, meaning within constraints that we specify as engineers. For the TurtleBot, we define **correct** as not crashing into static obstacles while obeying speed, acceleration, and yaw rate limits.

## 1. Modeling the TurtleBot
To control a robot, RTD uses a **state space model**. In the case of the Turtlebot, we use a **unicycle model**, which has position $(x,y)$, heading $\theta$, and speed $v$ as its **states**. These states have the following **dynamics**, expressed as an ordinary differential equation (ODE):
$$
\begin{align}
\frac{d}{dt}\begin{bmatrix}x \\ y \\ \theta \\ v \end{bmatrix} = \begin{bmatrix} v\cos\theta \\ v \sin\theta \\ \omega \\ a\end{bmatrix}.
\end{align}
$$



where the **control inputs** are $\omega$ (yaw rate) and $a$ (acceleration). Note that $\theta$ is written as `h` in the code, to represent "heading."



## 2. Receding-Horizon Planning

RTD performs **receding-horizon planning**, where the robot executes a short plan while computing a new plan. This strategy is used because robots have limited sensors. So, they have to move around to gain more information for planning. We denote the duration of each plan as an interval of time $[t_0,t_f]$. Without loss of generality, we can assume that $t_0 = 0$ at the beginning of each plan. So, we call $t_f$ the **planning time horizon**.

Now, suppose a plan is of duration $t_f$, and the robot waits until it executes the entire plan to begin executing its next plan. But, what if it can't find a new plan? It might end up crashing into stuff! To avoid this issue, we use a **planning timeout** $t_{\mathrm{plan}} < t_f$, which is the amount of time per planning iteration that the robot has to find a new plan. Then, to make sure the robot operates correctly, every plan has to include a **fail-safe maneuver** over the time interval $[t_{\mathrm{plan}}, t_f]$.

### Planning Hierarchy

Most autonomous mobile robots break down receding-horizon planning into a three-level hierarchy:
1. A **high-level planner** takes a coarse task description, like 'go to the end of the hall,' and breaks it down into smaller tasks, like 'go to this nearby waypoint'. An example is Google Maps' turn-by-turn info. The high-level planner typically ignores the robot's dynamics and immediate environment (such as obstacles), so that it can quickly provide high-level instructions.
2. A **trajectory planner** transforms the high-level instructions into dynamically-feasible *desired trajectories* for the robot to follow. This is usually specified as control inputs (like steering or throttle) and the desired trajectory in the robot's states.
3. A **low-level controller** that tracks the desired trajectory as best it can, by actuating the robot. An example is a proportional-integral-derivative controller (PID).

Since the high-level planner doesn't care about the robot's dynamics, and the low-level controller typically doesn't care about the robot's surroundings (like obstacles), the trajectory planner is responsible for ensuring that the robot doesn't run into stuff. It does this by creating desired trajectories that are correct. This is hard for two reasons. First, the state space model of a robot usually has many states, which means that creating desired trajectories takes a long time, but planning correctly requires planning in real time. Second, the low-level controller typically can't follow desired trajectories perfectly, so the trajectory planner has to compensate for **tracking error**.



## 3. RTD Summary

RTD is a trajectory planner that creates correct trajectories despite tracking error. It can handle other types of uncertainty, too, but for now we just focus on tracking error as an illustrative example. To do this, RTD first does an *offline* computation of a robot's **Forward-Reachable Set** (FRS), which is all points in space that a robot can reach while tracking a parameterized space of desired trajectories. Then, online, the FRS is used to identify all correct trajectories.



## 4. Tutorial Contents
This tutorial walks through each of the steps of RTD:
1. [Pick a **trajectory-producing model**](https://github.com/skousik/RTD_tutorial/tree/master/step_1_desired_trajectories), which defines the parameterized space of desired trajectories.
2. [Compute the robot's **tracking error**](https://github.com/skousik/RTD_tutorial/tree/master/step_2_error_function), since it can't always follow the desired trajectories perfectly.
3. [Compute the **FRS**](https://github.com/skousik/RTD_tutorial/tree/master/step_3_FRS_computation).
4. [Use the FRS online](https://github.com/skousik/RTD_tutorial/tree/master/step_4_online_planning) to represent obstacles as constraints for **trajectory optimization**.



## References

You can read more about the mathy details of RTD in the following papers:
1. S. Kousik$^*$, S. Vaskov$^*$, F. Bu, M. Johnson-Roberson, R. Vasudevan. "Bridging the Gap Between Safety and Real-Time Performance in Receding-Horizon Trajectory Design for Mobile Robots." [Link](https://arxiv.org/abs/1809.06746).
2. S. Vaskov, U. Sharma, S. Kousik, M. Johnson-Roberson, R. Vasudevan. "Guaranteed Safe Reachability-based Trajectory Design for a High-Fidelity Model of an Autonomous Passenger Vehicle." [Link](https://arxiv.org/abs/1902.01786)
3. S. Vaskov$^*$, S. Kousik$^*$, H. Larson, F. Bu, J. Ward, S. Worrall, M. Johnson-Roberson, R. Vasudevan. "Towards Provably Not-at-Fault Control of Autonomous Robots in Arbitrary Dynamic Environments." [Link](https://arxiv.org/abs/1902.02851)

$^*$ These authors contributed equally to this work



## Acknowledgements/Miscellaneous

Special thanks to Sean Vaskov and Ram Vasudevan for helping develop RTD, and writing much of the code in the RTD and simulator repositories. Also thanks to Nils Smit-Anseeuw and Pengcheng Zhao for many, many productive discussions over the past four years.

This tutorial is written in [Typora]([https://typora.io](https://typora.io/)) for Markdown editing. $\LaTeX$ support is provided by [TeXify](https://github.com/apps/texify). Special tip: when writing Markdown with LaTeX to be compiled by TeXify, make sure to add an extra newline after any math block, otherwise the LaTeX will be spaced out weirdly.