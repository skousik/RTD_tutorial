# Step 4: Online Planning

#### [Previous step: computing the FRS](https://github.com/skousik/RTD_tutorial/tree/master/step3_FRS_computation)

Details and code coming soon! Note that there are examples in the [RTD repository](https://github.com/ramvasudevan/RTD) for a Segway robot, which is really similar to the TurtleBot.

Here's a quick overview of RTD online planning. Recall that RTD uses a trajectory parameter space <img src="/step4_online_planning/tex/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode&sanitize=true" align=middle width=15.13700594999999pt height=22.465723500000017pt/> to specify desired trajectories. These trajectories are chosen every <img src="/step4_online_planning/tex/5ae1e561ec81f97d93ed9df0f76cab27.svg?invert_in_darkmode&sanitize=true" align=middle width=30.730753349999993pt height=20.221802699999984pt/> seconds in a receding-horizon way. Also recall that, in Step 3, we found a function <img src="/step4_online_planning/tex/9e293798cbade2bec567cf94b0aa0218.svg?invert_in_darkmode&sanitize=true" align=middle width=113.4889041pt height=22.648391699999998pt/> for which, if <img src="/step4_online_planning/tex/2b90bd95c73be2f030b51798e34e8adb.svg?invert_in_darkmode&sanitize=true" align=middle width=90.61637309999999pt height=24.65753399999998pt/>, then <img src="/step4_online_planning/tex/ce9da239b7dcd437e459cf571edf5e70.svg?invert_in_darkmode&sanitize=true" align=middle width=80.90935214999999pt height=24.65753399999998pt/>.

Now, to pick a particular <img src="/step4_online_planning/tex/3bf25d9d50c08c7ec96446a7abb1c024.svg?invert_in_darkmode&sanitize=true" align=middle width=44.30350649999999pt height=22.831056599999986pt/> at each planning iteration, given an arbitrary cost function <img src="/step4_online_planning/tex/5aefecdd2b9b5de63115b522e5e53f46.svg?invert_in_darkmode&sanitize=true" align=middle width=76.97455094999998pt height=22.648391699999998pt/>, we attempt to solve the following optimization problem:
<p align="center"><img src="/step4_online_planning/tex/a8809f9da005d084b0d81cc1704a2c6f.svg?invert_in_darkmode&sanitize=true" align=middle width=360.5954187pt height=16.438356pt/></p>

