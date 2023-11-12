% Test for function [T200IK]
clc;clear;close all
 
%%
%%%%%%%%%%%%%%%%%%%%%%%% Information definition %%%%%%%%%%%%%%%%%%%%%%%%%%%
% *define the geometric dimension of robot [TriMule200], which are constant
% in the whole program
dw = 132.5; % distance between point [C] and point [Q] [scalar,mm] %%%%%%%%【】
dv = 141.89; % distance between point [P] and point [Q] [scalar,mm] %%%%%%%【】
e = 233.5; % distance between point [A5] and point [P] [scalar,mm] %%%%%%%%【】

% *define the [rc] and [Rc], namely the position and orientation of TCP
% w.r.t. the inertial coordinate system
rc = [300,-93.36,895.22]'; %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
% position of TCP projected in inertial coordinate system [col,mm]
z64=[0;sqrt(2)/2;-sqrt(2)/2]; % 电主轴的位姿 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
x64=[0;sqrt(2)/2;sqrt(2)/2]; % 电主轴的位姿 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
y64=[1;0;0]; % 电主轴的位姿 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
Rc=[x64,y64,z64]; 
% orientation matrix of TCP projected in the inertial coordinate system [3-order matrix]

% *define the [vc] and [w], namely the linear and angular velocity of
% robotic end-effector w.r.t. the inertial coordinate system
vc = [100,0,0]'; % linear velocity of TCP [3d col,mm/s] %%%%%%%%%%%%%%%%%%%【】
w = [0,0,0]'; % angular velocity of TCP [3d col,rad/s] %%%%%%%%%%%%%%%%%%%%【】

% *define the [ac] and [alpha], namely the spatial acceleration and angular
% acceleration of the robotic end-effector w.r.t. the inertial coordinate
% system
ac = [100,0,0]'; % spatial acceleration of TCP [3d col,mm/s^2] %%%%%%%%%%%%【】
alpha = [0 0 0]'; % angular acceleration of TCP [3d col,rad/s^2] %%%%%%%%%%【】

%%

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Robot definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Object_TriMule200 = TriMule200(dw,dv,e); % define a robot [TriMule200] [object]


%%
%%%%%%%%%%%%%%%%%%%% Inverse kinematics calculation %%%%%%%%%%%%%%%%%%%%%%%
JointVariable = Object_TriMule200.T200InverseKinematics(rc,Rc);
disp(JointVariable)

%%
%%%%%%%%%%%%%%%%%% inverse solution of velocity issue %%%%%%%%%%%%%%%%%%%%%
JointVelocity = Object_TriMule200.T200InverseVelocity(vc,w);
disp(JointVelocity)

%%
%%%%%%%%%%%%%%%%%% inverse solution of acceleration issue %%%%%%%%%%%%%%%%%%%%%
[JointAcceleration,DrivingAccelerations] = Object_TriMule200.T200InverseAcceleration(ac,alpha);
disp(JointAcceleration)

%%
%%%%%%%%%%%%%%%%%% inverse solution of acceleration issue %%%%%%%%%%%%%%%%%%%%%
[~,~,~,DrivingForces] = Object_TriMule200.T200Dynamics(DrivingAccelerations);
disp(DrivingForces)







