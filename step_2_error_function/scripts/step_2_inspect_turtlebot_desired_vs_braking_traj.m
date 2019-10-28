%% description
% This script is to check that, for any desired trajectory and initial
% condition, the associated braking trajectory travels less far.
%
% Author: Shreyas Kousik
% Created: 28 Oct 2019
% Updated: -
%
%% user parameters
% desired trajectory parameters
w_des = 1 ;
v_des = 1 ;

% initial condition
v_0 = 1.25 ;

%% automated from here
% make bot
A = turtlebot_agent() ;

% get timing
t_plan = 0.5 ;
t_f = get_t_f_from_v_0(v_0) ;
t_stop = v_des / A.max_accel ;

% get desired traj
[T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

% get braking traj
[T_brk,U_brk,Z_brk] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des) ;

% track braking traj
A.reset([0;0;0;v_0]) ;
% A.move(T_brk(end),T_brk,U_brk,Z_brk) ;
A.move(T_go(end),T_go,U_go,Z_go) ;

%% plotting
figure(1) ; clf ; hold on ; axis equal ;

plot(A)
h1 = plot_path(Z_go,'b--','LineWidth',1.5) ;
h2 = plot_path(Z_brk,'r--','LineWidth',1.5) ;
legend([h1 h2],'desired traj','braking traj')
xlabel('x [m]')
ylabel('y [m]')
set(gca,'FontSize',15)