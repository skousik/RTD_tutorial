%% description
% This script computes the tracking error function "g" for the turtlebot.
% See the paper "Bridging the Gap Between Safety and Real-Time Performance
% in Receding-Horizon Trajectory Design for Mobile Robots" for an
% explanation of the error function in Section 2.2.2. In particular, see
% Assumption 10 that defines the tracking error function.
%
% The paper is available here: https://arxiv.org/abs/1809.06746
%
% Author: Shreyas Kousik
% Created: 15 May 2019
% Updated: 25 Oct 2019

%% user parameters
% initial condition bounds
v_0_min = 1.0 ; % m/s
v_0_max = 1.5 ; % m/s

% command bounds
v_max = 1.5 ;
w_min = -1.0 ; % rad/s
w_max =  1.0 ; % rad/s
delta_v = 0.25 ; % m/s

% number of samples in v_0, w, and v
N_samples = 4 ;

%% automated from here
% create turtlebot
A = turtlebot_agent ;

% create initial condition vector
v_0_vec = linspace(v_0_min,v_0_max,N_samples) ;

% create yaw commands
w_vec = linspace(w_min,w_max,N_samples) ;

% get time horizon of desired trajectory
t_f = get_t_f_from_v_0(v_0_min) ;

% initialize arrays for saving tracking error data
x_err = [] ;
y_err = [] ;

%% tracking eror computation loop
disp('Computing tracking error')

tic
for v_0 = v_0_vec
    disp(['v_0 = ',num2str(v_0,'%0.2f')])
    
    % create the initial condition
    z_0 = [0;0;0;v_0] ; % (x,y,h,v)
    
    % create the feasible speed commands from the initial condition
    v_des_min = max(0, v_0 - delta_v) ;
    v_des_max = min(v_max, v_0 + delta_v) ;
    v_vec = linspace(v_des_min,v_des_max, N_samples) ;
    
    for w_des = w_vec
        for v_des = v_vec
            % make the desired trajectory
            [T_des,U_des,Z_des] = make_turtlebot_desired_trajectory(t_f,w_des,v_des) ;
            
            % reset the robot
            A.reset(z_0)
            
            % track the desired trajectory
            A.move(T_des(end),T_des,U_des,Z_des) ;
            
            % get the realized position trajectory
            T = A.time ;
            X = A.state(A.position_indices,:) ;
            
            % interpolate the desired and realized trajectory to match
            X_des = Z_des(1:2,:) ;
            X = match_trajectories(T_des,T,X) ;
            
            % compute the tracking error
            pos_err = X - X_des ;
            
            % collect the data
            x_err = [x_err ; pos_err(1,:)] ;
            y_err = [y_err ; pos_err(2,:)] ;
            
            % % FOR DEBUGGING:
            % figure(1) ; clf ; hold on ; axis equal; grid on ;
            % plot_path(X,'b--','LineWidth',1.5) ;
            % plot(A)
            % figure(2) ; clf ; plot(pos_err')
        end
    end
end
toc


%% fit tracking error function
% get max of absolute tracking error
x_err = abs(x_err) ;
y_err = abs(y_err) ;
x_max = max(x_err,[],1) ;
y_max = max(y_err,[],1) ;

% fit polynomial to the data
int_g_x_coeffs = polyfit(T_des,x_max,4) ;
int_g_y_coeffs = polyfit(T_des,y_max,4) ;

% take the time derivative of these to get the g functions in x and y
g_x_coeffs = polyder(int_g_x_coeffs) ;
g_y_coeffs = polyder(int_g_y_coeffs) ;

%% correct the fit to make it greater than the data
% evaluate g
int_g_x_coeffs = polyint(g_x_coeffs) ;
int_g_y_coeffs = polyint(g_y_coeffs) ;
int_g_x_vals = polyval(int_g_x_coeffs,T_des) ;
int_g_y_vals = polyval(int_g_y_coeffs,T_des) ;

% figure out the maximum ratio of the error data to the int g values
r_x_err = x_max ./ int_g_x_vals ;
r_x_max = max([1,r_x_err]) ;
r_y_err = y_max ./ int_g_y_vals ;
r_y_max = max([1,r_y_err]) ;

% multiply the g_x and g_y coefficients by the error data ratio
g_x_coeffs = r_x_max .* g_x_coeffs ;
g_y_coeffs = r_y_max .* g_y_coeffs ;

% re-integrate g with the new coefficients
int_g_x_coeffs = polyint(g_x_coeffs) ;
int_g_y_coeffs = polyint(g_y_coeffs) ;
int_g_x_vals = polyval(int_g_x_coeffs,T_des) ;
int_g_y_vals = polyval(int_g_y_coeffs,T_des) ;

%% save data
filename = ['turtlebot_error_functions_v_0_',...
            num2str(v_0_min,'%0.1f'),'_to_',...
            num2str(v_0_max,'%0.1f'),'.mat'] ;
save(filename,'g_x_coeffs','g_y_coeffs','v_max','w_min','w_max',...
     'delta_v','v_0_min','v_0_max') ;

%% plotting
figure(1) ; clf ;

% plot x error
subplot(2,1,1) ; hold on ;
plot(T_des,x_err','k--')
g_x_handle =  plot(T_des,int_g_x_vals,'r-','LineWidth',1.5) ;
title('tracking error vs. time')
ylabel('x error [m]')
legend(g_x_handle,'\int g_x(t) dt','Location','NorthWest')
axis([0, t_f, 0, 0.03])
set(gca,'FontSize',15)

% plot y error
subplot(2,1,2) ; hold on ;
plot(T_des,y_err','k--')
g_y_handle = plot(T_des,int_g_y_vals,'r-','LineWidth',1.5) ;
xlabel('time [s]')
ylabel('y error [m]')
legend(g_y_handle,'\int g_y(t) dt','Location','NorthWest')
axis([0, t_f, 0, 0.01])
set(gca,'FontSize',15)