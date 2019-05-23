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

Coming soon!

## 4.3 Trajectory Optimization

Coming soon!

## 4.4 Making a Planner Class for Simulation

Coming soon!

## Appendix 4.A: Planner Class Overview

Coming soon!