# Step 4: Online Planning

#### [Previous step: computing the FRS](https://github.com/skousik/RTD_tutorial/tree/master/step3_FRS_computation)

Details and code coming soon! Note that there are examples in the [RTD repository](https://github.com/ramvasudevan/RTD) for a Segway robot, which is really similar to the TurtleBot.

Here's a quick overview of RTD online planning. Recall that RTD uses a trajectory parameter space $K$ to specify desired trajectories. These trajectories are chosen every $t_{\mathrm{plan}}$ seconds in a receding-horizon way. Also recall that, in Step 3, we found a function $w: X \times K \to \mathbb{R}$ for which, if $(x,k) \in \mathrm{FRS}$, then $w(x,k) \geq 1$.

Now, to pick a particular $k \in K$ at each planning iteration, given an arbitrary cost function $J: K \to \mathbb{R}$, we attempt to solve the following optimization problem:
$$
\begin{align}
k^* = \mathrm{argmin}_k \{J(k)~|~w(x,k) < 1\ \forall\ x \in \mathrm{obstacles}\}
\end{align}.
$$

