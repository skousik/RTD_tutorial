%% description
% This script demonstrates how to use the distance scales correctly to get
% points in and out of the FRS frame. The functions being checked are:
%   world_to_FRS_frame
%   FRS_to_world_frame
%
% Note that this script requires the FRS .mat files computed in Step 3.
%
% Author: Shreyas Kousik
% Date: 22 Oct 2019
%
%% user parameters
% agent pose in SE(2)
pose = [0;0.5;1] ; % (x,y,h)

% test point in world frame to move to FRS frame
p_world = [0;1] ;

% test point in FRS frame to move to world frame
p_FRS = [0;0] ;

% FRS to use
FRS = load('turtlebot_FRS_deg_10_v0_0.5_to_1.0.mat') ;

%% automated from here
% create turtlebot
A = turtlebot_agent ;
A.reset([pose;0])

%% move the world footprint to the FRS frame
% get some points from the agent footprint
P_fp_world = A.footprint_vertices(:,1:5:end) ;

% move the agent footprint to the global frame
N_fp = size(P_fp_world,2) ;
P_fp_world = P_fp_world + repmat(A.state(A.position_indices,end),1,N_fp) ;

% move actual footprint to FRS frame
x0 = FRS.initial_x ;
y0 = FRS.initial_y ;
Dx = FRS.distance_scale_x ;
Dy = FRS.distance_scale_x ;
P_fp_FRS = world_to_FRS(P_fp_world,A.state(:,end),x0,y0,Dx,Dy) ;

%% move the FRS footprint to the world frame
% get footprint in FRS frame
h_Z0 = FRS.h_Z0 ;
z = FRS.z ;
P_Z0_FRS = get_2D_contour_points(h_Z0,z,0) ;

% move FRS frame footprint to world frame
P_Z0_world = FRS_to_world(P_Z0_FRS(:,1:5:end),A.state(:,end),x0,y0,Dx,Dy) ;

%% move the world point to the FRS frame and back
p_world_to_FRS = world_to_FRS(p_world,A.state(:,end),x0,y0,D) ;
p_world_to_FRS_to_world = FRS_to_world(p_world_to_FRS,A.state(:,end),x0,y0,Dx,Dy) ;

%% move the FRS point to the world frame and back
p_FRS_to_world = FRS_to_world(p_FRS,A.state(:,end),x0,y0,Dx,Dy) ;
p_FRS_to_world_to_FRS = world_to_FRS(p_FRS_to_world,A.state(:,end),x0,y0,Dx,Dy) ;

%% plotting (to verify)
figure(1) ; clf ;

%% plot FRS frame on left
subplot(1,2,1) ; hold on ; axis equal ; grid on ;

% plot FRS frame footprint
plot(P_Z0_FRS(1,:),P_Z0_FRS(2,:),'b','LineWidth',1.5)

% plot world_to_local footprint
plot(P_fp_FRS(1,:),P_fp_FRS(2,:),'b*')

% plot the FRS point
plot(p_FRS(1,:),p_FRS(2,:),'r*','MarkerSize',12)
plot(p_FRS_to_world_to_FRS(1,:),p_FRS_to_world_to_FRS(2,:),'ro','MarkerSize',12)

% plot the world point moved into the FRS frame
plot(p_world_to_FRS(1,:),p_world_to_FRS(2,:),'k*','MarkerSize',12)

% labeling
title('FRS Frame')
set(gca,'FontSize',15)

%% plot world frame on right
subplot(1,2,2) ; hold on ; axis equal ; grid on ;

% plot agent
plot(A)

% plot footprint from FRS frame
plot(P_Z0_world(1,:),P_Z0_world(2,:),'b*')

% plot the world point
plot(p_world(1,:),p_world(2,:),'k*','MarkerSize',12)
plot(p_world_to_FRS_to_world(1,:),p_world_to_FRS_to_world(2,:),'ko','MarkerSize',12)

% plot the FRS point moved into the world frame
plot(p_FRS_to_world(1,:),p_FRS_to_world(2,:),'r*','MarkerSize',12)

% labeling
title('World Frame')
set(gca,'FontSize',15)