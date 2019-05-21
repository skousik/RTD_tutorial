function example_3_braking_trajectory(w_des,v_des,v0)
% example_3_braking_trajectory(w_des,v_des,initial_speed)
%
% Make an example desired trajectory for the TurtleBot, and show the robot
% trying to track it, then brake along it.
%
% Author: Shreyas Kousik
% Date: 15 May 2019

%% setup
    % default inputs
    if nargin < 3
        v0 = 1.0 ; % m/s
    end
    
    if nargin < 2
        v_des = 1.0 ;
    end
    
    if nargin < 1
        w_des = 0.5 ;
    end
    
    % load timing
    try
        disp('Loading turtlebot RTD planner timing info.')
        timing_info = load('turtlebot_timing.mat') ;
        t_plan = timing_info.t_plan ;
        t_stop = timing_info.t_stop ;
        t_f = timing_info.t_f ;
    catch
        disp('Could not find timing MAT file. Setting defaults!')
        t_plan = 0.5 ;
        t_stop = 2.61 ;
        t_f = 0.95 ;
    end

    % make robot
    A = turtlebot_agent ;

%% trajectory
    % make desired trajectory
    [T_go,U_go,Z_go] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;

    % convert the desired trajectory to a braking trajectory that can be used
    % by the TurtleBot with the PD controller that we're using for RTD (NOTE
    % we don't have to use PD for RTD, but we've chosen it for this example)
    [T_brk,U_brk,Z_brk] = make_turtlebot_RTD_braking_traj(t_plan,t_stop,T_go,U_go,Z_go) ;

    % reset the agent to its initial speed
    z0 = [0;0;0;v0] ;
    A.reset(z0) ;

    % execute desired trajectory with braking
    A.move(T_brk(end),T_brk,U_brk,Z_brk)
    
%% plotting
    close all
    figure(1)
    hold on
    axis equal
    
    plot(A.state(1,:),A.state(2,:),'LineWidth',1.5)
    plot(Z_go(1,:),Z_go(2,:),'b--','LineWidth',1.5)
    plot(A)
    
    % set axes
    xlo = min(A.state(1,:)) - 0.5 ;
    xhi = max(A.state(1,:)) + 0.5 ;
    ylo = min(A.state(2,:)) - 0.5 ;
    yhi = max(A.state(2,:)) + 0.5 ;
    axis([xlo,xhi,ylo,yhi])
    
    % label plot
    xlabel('x [m]')
    ylabel('y [m]')
    legend('executed traj.','desired traj.')
    set(gca,'FontSize',15)

    % animate agent
    A.animate
end