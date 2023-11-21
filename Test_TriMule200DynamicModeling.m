%% Test for dynamic modeling of class [TriMule200]
clc;clear;close all

%%
%%%%%%%%%%%%%%%%%%%%%%%% Fundamental information%% %%%%%%%%%%%%%%%%%%%%%%%%
% *define the geometric dimension of robot [TriMule200], which are constant
% in the whole program
dw = 132.5; % distance between point [C] and point [Q] [scalar,mm] %%%%%%%%【】
dv = 141.89; % distance between point [P] and point [Q] [scalar,mm] %%%%%%%【】
e = 233.5; % distance between point [A5] and point [P] [scalar,mm] %%%%%%%%【】

%%
%%%%%%%%%%%%%%%%%%%%%%% Trajectory definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% a trajectory defining [pose]-[velocity]-[acceleration] of TCP is
% determined here

% *(1) Constant elements definition
% *define the [Rc], namely the orientation of TCP w.r.t. the inertial coordinate system
z64=[0;sqrt(2)/2;-sqrt(2)/2]; % 电主轴的位姿 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
x64=[0;sqrt(2)/2;sqrt(2)/2]; % 电主轴的位姿 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
y64=[1;0;0]; % 电主轴的位姿 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
Rc=[x64,y64,z64]; 
% orientation matrix of TCP in the inertial coordinate system [3-order matrix]

% *define the [w], namely the angular velocity of TCP w.r.t. the inertial coordinate system
w = [0,0,0]'; % angular velocity of TCP [3d col,rad/s] %%%%%%%%%%%%%%%%%%%%【】

% *define the [alpha], namely the angular acceleration of the TCP w.r.t. the inertial coordinate system
alpha = [0 0 0]'; % angular acceleration of TCP [3d col,rad/s^2] %%%%%%%%%%【】

% (2) Time-varying elements definition
% *position [rc], velocity [vc], and acceleration [ac] in trajectory are 
% time-varying elements. They are determined for each specific instant.
% *A straight trajectory with the duration of [3s] is defined in this part,
% acceleration is in the first [1s] [-300 mm/s^2], in the middle [1s] [0
% mm/s^2], and in the last [1s] [300 mm/s^2]

% *Origins definition
rc = zeros(3,61); % register for 末端参考点C的位矢
vc = zeros(3,61); % register for 末端参考点C的速度矢量
ac =zeros(3,61); % register for 末端参考点C的加速度矢量
a = 300; % acceleration in motion, [scalar mm/s^2]
rc(:,1) = [300;-93.36;895.22]; % original position of TCP [3d col,mm]
vc(:,1) = [0;0;0]; % origianl velocity of TCP [3d col,mm/s]
ac(:,1) = [-a;0;0]; % original acceleration of TCP [3d col,mm/s^2]

% *Sequences of [rc], [vc] and [ac] generation
t = 0; % original time [scalar,s]
for i = 1:60
    t = t+0.05; 
    % the duration of [3s] is divided into [60] steps, with the length of [0.05s]
    if t<=1
       S=0.5*a*t^2;V=a*t;A=a;
    elseif 1<t&&t<=2
       S=150+(t-1)*300;V=300;A=0;
    else
       S=600-0.5*a*(3-t)^2;V=300-a*(t-2);A=-a;
    end
    rc(:,i+1) = rc(:,1)+[-S;0;0];
    vc(:,i+1) = vc(:,1)+[-V;0;0];
    ac(:,i+1) = [-A;0;0];
end

%%
%%%%%%%%%%%%%%%%%% Robot definition and motion simulation %%%%%%%%%%%%%%%%%
Object_TriMule200 = TriMule200(dw,dv,e); % define a robot [TriMule200] [object]
time=zeros(1,61);%%运动3s，步长0.05s，共61个点
%create cells to save data
qudingli1=zeros(1,61);%%6个主动关节的速度，时间0-3s，步长0.05s
qudingli2=zeros(1,61);
qudingli3=zeros(1,61);
qudingli4=zeros(1,61);
qudingli5=zeros(1,61);
qudingli6=zeros(1,61);

JointVariable=cell(1,61); 
JointVelocity=cell(1,61);
JointAcceleration=cell(1,61);
DrivingAccelerations=cell(1,61);
DrivingForces=cell(1,61);

for i=0:60
    time(1,i+1)=0.05*i;%%步长为0.05s，从0一直到3
    JointVariable{1,i+1} = Object_TriMule200.T200InverseKinematics(rc(:,i+1),Rc); 
    JointVelocity{1,i+1} = Object_TriMule200.T200InverseVelocity(vc(:,i+1),w);
    [JointAcceleration{1,i+1},DrivingAccelerations{1,i+1}] = Object_TriMule200.T200InverseAcceleration(ac(:,i+1),alpha);
    [~,~,~,DrivingForces{1,i+1}] = Object_TriMule200.T200Dynamics(DrivingAccelerations{1,i+1});    
end
Forces=cell2mat(DrivingForces);
DrivingAccelerations_matrix = cell2mat(DrivingAccelerations);

for i=0:60
    qudingli1(1,i+1)=Forces(1,1+i);
    qudingli2(1,i+1)=Forces(2,1+i);
    qudingli3(1,i+1)=Forces(3,1+i);
    qudingli4(1,i+1)=Forces(4,1+i);
    qudingli5(1,i+1)=Forces(5,1+i);
    qudingli6(1,i+1)=Forces(6,1+i);   

end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Motion display %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);%%第一主动支链推杆推力
plot(time,qudingli1/10^6,'r');
xlabel('时间 s'); 
ylabel('推力 N'); 
title('第一主动支链推杆推力TestforClass')
hold on;

figure(2);%%第二、三主动支链推杆推力
plot(time,qudingli2/10^6,'g',time,qudingli3/10^6,'b');
xlabel('时间 s'); 
ylabel('推力 N'); 
title('第二、三主动支链推杆推力TestforClass')
hold on;

figure(3);%%第4关节驱动力矩
plot(time,qudingli4/10^6,'r');
xlabel('时间 s'); 
ylabel('力矩 N.mm'); 
title('第4关节驱动力矩TestforClass')
hold on;

figure(4);%%第5关节驱动力矩
plot(time,qudingli5/10^6,'r');
xlabel('时间 s'); 
ylabel('力矩 N.mm'); 
title('第5关节驱动力矩TestforClass')
hold on;

figure(5);%%第6关节驱动力矩
plot(time,qudingli6/10^6,'r');
xlabel('时间 s'); 
ylabel('力矩 N.mm'); 
title('第6关节驱动力矩TestforClass')
hold on;
