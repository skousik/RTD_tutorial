%% description
% This script considers how the Turtlebot's braking distance increases as a
% function of its initial speed, and uses that information to determine the
% time horizon required for reachability analysis.
%
% We also consider that, at each receding-horizon planning iteration, we
% are able to choose a commanded speed of our current speed plus or minus
% some change, delta_v:
%
%   v_cmd = v_0 +/- delta_v
%
% This means that we could be braking from a higher speed than our initial
% speed at the beginning of the planning iteration. Therefore, we need to
% increase our braking time estimate to compensate.
%
% Author: Shreyas Kousik
% Created: 24 Oct 2019
% Updated: 25 Oct 2019
%
%% user parameters
% speed range to consider
v_min = 0.1 ; % needs to be > 0 for the braking traj to make sense
v_max = 1.5 ;

% get the allowed commandable change in the turtlebot's speed
delta_v = 0.25 ; % m/s

%% automated from here
% create vector of initial speeds
v_0_vec = v_min:0.1:v_max ;

% set up turtlebot agent
A = turtlebot_agent() ;

% get vector for distances traveled
d_brk = nan(size(v_0_vec)) ;

% for each initial speed, generate a braking trajectory and then execute it
t_plan = 0.5 ;
idx = 1 ;
for v_0 = v_0_vec
    % get stopping time for current t_stop
    t_stop = v_0/A.max_accel ;
    
    % set up braking trajectoryt
    w_des = 0 ;
    v_des = v_0 ;
    [T_brk,U_brk,Z_brk] = make_turtlebot_braking_trajectory(t_plan,t_stop,w_des,v_des) ;
    
    % track braking trajectory
    z0 = [0;0;0;v_0] ;
    A.reset(z0)
    A.move(T_brk(end),T_brk,U_brk,Z_brk) ;
    
    % find the total distance traveled during braking
    T = A.time ;
    d_at_t_plan = match_trajectories(t_plan,A.time,A.state(1,:)) ;
    d_brk(idx) = A.state(1,end) - d_at_t_plan ;
    idx = idx + 1 ;
end

%% compute t_f for each of the max initial speed ranges
% get t_stop for all the braking distances
t_stop_vec = d_brk ./ v_0 ;

% round up to the nearest 0.1 s to preempt numerical errors
t_stop_vec = ceil(10*t_stop_vec)./10 ;

% get t_f for the three FRSes we plan to compute later
v_0_all = [0.5 1.0 1.5] ;
v_begin_braking = v_0_all + delta_v.*[1 1 0] ;
t_f_all = match_trajectories(v_begin_braking,v_0_vec,t_stop_vec) + t_plan ;

%% save timing
save('turtlebot_timing.mat','t_plan','t_f_all',...
    'v_0_all','v_begin_braking','delta_v') ;

%% plot v_0 vs d
figure(1) ; clf ; hold on ; grid on ;
plot(v_0_vec,d_brk,'LineWidth',1.5)
xlabel('v_0 [m/s]')
ylabel('braking distance [m]')
set(gca,'FontSize',15)