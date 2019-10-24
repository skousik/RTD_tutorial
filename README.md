**TL;DR**: [Start here with Step 1](https://github.com/skousik/RTD_tutorial/tree/master/step_1_desired_trajectories) if you don't want to read the overview.

# RTD Tutorial

This tutorial implements [Reachability-based Trajectory Design](https://github.com/ramvasudevan/RTD) (RTD) on a [TurtleBot](https://www.turtlebot.com/turtlebot2/) in simulation. RTD has been developed at the University of Michigan's [ROAHM Lab](http://www.roahmlab.com/).

Our choice of the TurtleBot was inspired by the work of our friends at UC Berkeley on Hamilton-Jacobi reachability, which can be found [here](https://abajcsy.github.io/safe_navigation/). In this work, we instead use Sums-of-Squares (SOS) programming for our reachability analysis. Our approach is detailed [here](https://arxiv.org/abs/1809.06746).

### Authors
Shreyas Kousik (skousik at umich.edu).

### License
This project is licensed under the BSD 3-Clause License.



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
To control a robot, RTD uses a **state space model**. In the case of the Turtlebot, we use a **unicycle model**, which has position <img src="/tex/7392a8cd69b275fa1798ef94c839d2e0.svg?invert_in_darkmode&sanitize=true" align=middle width=38.135511149999985pt height=24.65753399999998pt/>, heading <img src="/tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode&sanitize=true" align=middle width=8.17352744999999pt height=22.831056599999986pt/>, and speed <img src="/tex/6c4adbc36120d62b98deef2a20d5d303.svg?invert_in_darkmode&sanitize=true" align=middle width=8.55786029999999pt height=14.15524440000002pt/> as its **states**. These states have the following **dynamics**, expressed as an ordinary differential equation (ODE):
<p align="center"><img src="/tex/fe5ae23c6c1a6c32f63039eddf1fe044.svg?invert_in_darkmode&sanitize=true" align=middle width=145.87832655pt height=78.9048876pt/></p>



where the **control inputs** are <img src="/tex/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode&sanitize=true" align=middle width=10.82192594999999pt height=14.15524440000002pt/> (yaw rate) and <img src="/tex/44bc9d542a92714cac84e01cbbb7fd61.svg?invert_in_darkmode&sanitize=true" align=middle width=8.68915409999999pt height=14.15524440000002pt/> (acceleration). Note that <img src="/tex/27e556cf3caa0673ac49a8f0de3c73ca.svg?invert_in_darkmode&sanitize=true" align=middle width=8.17352744999999pt height=22.831056599999986pt/> is written as `h` in the code, to represent "heading."



## 2. Receding-Horizon Planning

RTD performs **receding-horizon planning**, where the robot executes a short plan while computing a new plan. This strategy is used because robots have limited sensors. So, they have to move around to gain more information for planning. We denote the duration of each plan as an interval of time <img src="/tex/e0393984691e829b97a04ed01dc2113e.svg?invert_in_darkmode&sanitize=true" align=middle width=41.50124384999999pt height=24.65753399999998pt/>. Without loss of generality, we can assume that <img src="/tex/c9d7adaf70649e6eac596cb7eba2b33e.svg?invert_in_darkmode&sanitize=true" align=middle width=43.44739739999999pt height=21.18721440000001pt/> at the beginning of each plan. So, we call <img src="/tex/2db35060f2b6f146752157657cfb5d5a.svg?invert_in_darkmode&sanitize=true" align=middle width=10.930443149999991pt height=20.221802699999984pt/> the **planning time horizon**.

Now, suppose a plan is of duration <img src="/tex/2db35060f2b6f146752157657cfb5d5a.svg?invert_in_darkmode&sanitize=true" align=middle width=10.930443149999991pt height=20.221802699999984pt/>, and the robot waits until it executes the entire plan to begin executing its next plan. But, what if it can't find a new plan? It might end up crashing into stuff! To avoid this issue, we use a **planning timeout** <img src="/tex/562282b5face354edcd2f160fd3941e0.svg?invert_in_darkmode&sanitize=true" align=middle width=64.40072759999998pt height=20.221802699999984pt/>, which is the amount of time per planning iteration that the robot has to find a new plan. Then, to make sure the robot operates correctly, every plan has to include a **fail-safe maneuver** over the time interval <img src="/tex/bbe77024f3f059a59d705d15a7827e6c.svg?invert_in_darkmode&sanitize=true" align=middle width=59.74333859999999pt height=24.65753399999998pt/>.

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
5. [Explore the nitty gritty details](https://github.com/skousik/RTD_tutorial/tree/master/step_5_extras) with bonus content.



## References (How to Cite This Work)

You can read more about the mathy details of RTD in the following papers. Please cite the ones relevant to your usage of our code.
1. S. Kousik<img src="/tex/cdcac8939f3840cd8cddf40059a4cf58.svg?invert_in_darkmode&sanitize=true" align=middle width=6.735194399999992pt height=22.63846199999998pt/>, S. Vaskov<img src="/tex/cdcac8939f3840cd8cddf40059a4cf58.svg?invert_in_darkmode&sanitize=true" align=middle width=6.735194399999992pt height=22.63846199999998pt/>, F. Bu, M. Johnson-Roberson, R. Vasudevan. "Bridging the Gap Between Safety and Real-Time Performance in Receding-Horizon Trajectory Design for Mobile Robots." [Link](https://arxiv.org/abs/1809.06746).
2. S. Vaskov, U. Sharma, S. Kousik, M. Johnson-Roberson, R. Vasudevan. "Guaranteed Safe Reachability-based Trajectory Design for a High-Fidelity Model of an Autonomous Passenger Vehicle." [Link](https://arxiv.org/abs/1902.01786)
3. S. Vaskov<img src="/tex/cdcac8939f3840cd8cddf40059a4cf58.svg?invert_in_darkmode&sanitize=true" align=middle width=6.735194399999992pt height=22.63846199999998pt/>, S. Kousik<img src="/tex/cdcac8939f3840cd8cddf40059a4cf58.svg?invert_in_darkmode&sanitize=true" align=middle width=6.735194399999992pt height=22.63846199999998pt/>, H. Larson, F. Bu, J. Ward, S. Worrall, M. Johnson-Roberson, R. Vasudevan. "Towards Provably Not-at-Fault Control of Autonomous Robots in Arbitrary Dynamic Environments." [Link](https://arxiv.org/abs/1902.02851)

<img src="/tex/7c74eeb32158ff7c4f67d191b95450fb.svg?invert_in_darkmode&sanitize=true" align=middle width=8.219209349999991pt height=15.296829900000011pt/> These authors contributed equally to this work



## Acknowledgements/Miscellaneous

Special thanks to Sean Vaskov and Ram Vasudevan for helping develop RTD, and writing much of the code in the RTD and simulator repositories. Also thanks to Nils Smit-Anseeuw and Pengcheng Zhao for many, many productive discussions over the past four years.

This tutorial is written in [Typora]([https://typora.io](https://typora.io/)) for Markdown editing. <img src="/tex/87181ad2b235919e0785dee664166921.svg?invert_in_darkmode&sanitize=true" align=middle width=45.69716744999999pt height=22.465723500000017pt/> support is provided by [TeXify](https://github.com/apps/texify). Special tip: when writing Markdown with LaTeX to be compiled by TeXify, make sure to add an extra newline after any math block, otherwise the LaTeX will be spaced out weirdly.