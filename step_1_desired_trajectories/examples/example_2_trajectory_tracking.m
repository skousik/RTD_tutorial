%% run the trajectory tracking
A = turtlebot_agent() ;
v0 = 0.5 ;
A.reset([0;0;0;0.5])

t_f = 1 ;
w_des = 0.5 ; % rad/s
v_des = 1.0 ; % m/s
[T,U,Z] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

t_total = 1 ;
A.move(t_total,T,U,Z)

%% plotting
figure(1) ; clf ; axis equal ; hold on ; set(gca,'FontSize',15)

plot(Z(1,:),Z(2,:),'b--','LineWidth',1.5)
plot(A)
A.animate()
