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

Coming soon!

## 4.3 Trajectory Optimization

Coming soon!

## 4.4 Making a Planner Class for Simulation

Coming soon!

## Appendix 4.A: Planner Class Overview

Coming soon!