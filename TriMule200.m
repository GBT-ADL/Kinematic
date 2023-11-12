classdef TriMule200 < handle 
    % *This class describes the TriMule200, a kind of hybrid robot
    % *All necessary information about this robot is covered in this class

    properties
        %%%%%%%%%%%%%%%% Fundamental geometric information %%%%%%%%%%%%%%%%
        % Constants in geometric structure
        dw % distance between point [C] and point [Q] [scalar,mm] 
        dv % distance between point [P] and point [Q] [scalar,mm]
        e  % distance between point [A5] and point [P] [scalar,mm]
           % Note that the parameter [e] is for actual robot 283.5 mm, which needs further confirm
        y = [-pi/2,0,pi]; % 伽玛值，这是由于三条主动支链第四构件摆放方式不同，但是建立的坐标系完全一样造成的恒定的转角偏差 [from TJU]
 
        %%%%%%%%%%%%%%%%%%%% Defined pose information %%%%%%%%%%%%%%%%%%%%%
        % Pose of robotic end-effector projected in the inertial coordinate
        % system, defined in kinematics
        rc % position of point C w.r.t. the inertial coordinate system [3d col]
        Rc % orientation matrix of TCP w.r.t. the inertial coordinate system [3*3 matrix]
           % each matrix [Rc] = [Xc,Yc,Zc], where the [Xc]~[Zc] are
           % orientations of axis [X]~[Z] of TCP coordinate system
           % projected in the inertial coordinate system

        %%%%%%%%%%%%%%%% Calculated kinematical information %%%%%%%%%%%%%%%
        % Kinematical information calculated in [inverse kinematics]. This
        % data are useful in subsequent program.
        rp  % P点的位置矢量 [from TJU]
        q3i % 储存三条主动支链第三关节变量 [from TJU]
        q34 % 被动支链第三关节长度，整机基坐标系原点到动平台中点A的距离 [from TJU]
        s31 % [from TJU]
        s32 % [from TJU]
        s33 % [from TJU]
        s34 % 被动支链第三关节的方向矢量，与点P的位置矢量方向相同 [from TJU]
        R14 % [from TJU]
        R31 % 第1支链(active chain)中第三构件的姿态矩阵 [from TJU]
        R32 % 第2支链(active chain)中第三构件的姿态矩阵 [from TJU]
        R33 % 第3支链(active chain)中第三构件的姿态矩阵 [from TJU]
        R34 % 第4支链(passive chain)中第三构件的姿态矩阵 [from TJU]
        R3ifen % 将R3i分成3块，每一块为3×3的矩阵，代表对应支链第3构件的姿态矩阵 [from TJU]
        R44 % [from TJU]
        R54 % [from TJU]
        R64 % 末端执行器相对于基坐标系的姿态矩阵 [from TJU]
        sita14 % 是由R34矩阵得出的，被动支链第一关节转角，注意测量时有一个45°附加值，此量单位为弧度 [from TJU]
        sita12i % 储存三条主动支链1，2关节变量 [from TJU]
        sita456i % 储存三条支链4，5，6关节变量 [from TJU]
        zhuantou % 保存最终的解，其余均为0 [from TJU]

        %%%%%%%%%%% Calculated information in velocity modeling %%%%%%%%%%%
        kecitai1 % [from TJU]
        kecitai2 % [from TJU]
        kecitai3 % [from TJU]
        zhudongzhiliansudu % 三条主动支链中所有关节的速度 [from TJU]
        zhudongguanjiesudu % 主动关节的速度 [from TJU]
        beidongzhiliansudu % 被动支链中所有关节的速度 [from TJU]
        TiFen % [from TJU]
        Tap % 雅可比的第一块 [from TJU]
        kecit1 % [from TJU]
        kecit2 % [from TJU]
        kecit3 % [from TJU]
        kecit4 % [from TJU]
        T1weifen % [from TJU]
        T2weifen % [from TJU]
        T3weifen % [from TJU]
        T4weifen % [from TJU]
        Wa4 % [from TJU]
        rcp % 点C到点P的矢量 [from TJU]
        sudu4 % [from TJU]

        %%%%%%%%% Calculated information in acceleration modeling %%%%%%%%%
        kecit21chacheng % [from TJU]
        kecit31chacheng % [from TJU]
        kecit22chacheng % [from TJU]
        kecit32chacheng % [from TJU]
        kecit23chacheng % [from TJU]
        kecit33chacheng % [from TJU]
        kecit14chacheng % [from TJU]
        kecit34chacheng % [from TJU]
        kecit44chacheng % [from TJU]
        kecit54chacheng % [from TJU]
        kecit64chacheng % [from TJU]
        kecita14 % [from TJU]
        kecita24 % [from TJU]
        kecita34 % [from TJU]
        kecita44 % [from TJU]
        kecita54 % [from TJU]
        kecita64 % [from TJU]
        
        %%%%%%%%%%%%%%%%%%%%%%% Joint variables %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Joint variables calculated in [inverse kinematics], where the 6*4
        % = 24 parameters distributed on 4 chains and 6 joints on each
        % of them.correspond to generalized coordinate of the [TriMule200 robot] 
        JointVariables % joint variables including 24 parameters [6*4 matrix, rad and mm]

        %%%%%%%%%%%%%%%%%%%%%%%% Joint velocity %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Joint variables calculated in [inverse velocity], where the 6*4
        % = 24 parameters distributed on 4 chains and 6 joints on each
        % of them.correspond to generalized velocity of the [TriMule200 robot] 
        JointVelocities % joint velocity including 24 parameters [6*4 matrix, rad/s and mm/s]

        %%%%%%%%%%%%%%%%%%%%% Joint acceleration %%%%%%%%%%%%%%%%%%%%%%%%%%
        % Joint variables calculated in [inverse acceleration], where the 6*4
        % = 24 parameters distributed on 4 chains and 6 joints on each
        % of them.correspond to generalized acceleration of the [TriMule200 robot] 
        JointAccelerations % joint accelerations including 24 parameters [6*4 matrix, rad/s^2 and mm/s^2]

        %%%%%%%%%%%%%%%%%%%%%%%%%% Running flag %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % To ensure the following execution order, a [Running flag] is loaded
        % (1) inverse kinematics can be calculated at any time
        % (2) inverse velocity issue is solved based on inverse kinematics
        % (3) inverse acceleration issue is solved based on inverse
        %     kinematics and inverse velocity issue
        % (4) dynamic modeling is available based on all information above
        % (5) continous solution of inverse kinematics, inverse velocity
        %     and acceleration issues are feasible
        RunningFlag = 1
        % *the bit [RunningFlag] has four situations [1/2/3/4]
        % [RunningFlag = 1]: No function have been executed
        % [RunningFlag = 2]: inverse kinematics has been executed
        % [RunningFlag = 3]: velocity issue has been solved inversely
        % [RunningFlag = 4]: acceleration issue has been solved inversely
        % *When [RunningFlag = 1/2/3/4], the inverse kinematics is
        % executable, thereafter the [RunningFlag] is set to 2
        % *When [RunningFlag = 2/3/4], the velocity issue can be solved
        % inversely, thereafter the [RunningFlag] is set to 3
        % *When [RunningFlag = 3/4], the acceleration issue can be solved
        % inversely, thereafter the [RunningFlag] is set to 4
        % *When only [RunningFlag = 4], the dynamic model can be built,
        % thereafter the [RunningFlag] is set to 1.
    end

    methods
        function obj = TriMule200(dw,dv,e)
            % The constructor is used to preliminary define the TriMule200 robot.
            % input: [dw] distance between point [C] and point [Q] [scalar,mm] 
            %        [dc] distance between point [P] and point [Q] [scalar,mm]
            %        [e] distance between point [A5] and point [P] [scalar,mm]
            % output: [obj] the defined [TriMule200] object [object]

            obj.dw = dw; % distance between point [C] and point [Q] [scalar,mm] 
            obj.dv = dv; % distance between point [P] and point [Q] [scalar,mm]
            obj.e = e;   % distance between point [A5] and point [P] [scalar,mm]
        end

        function JointVariables = T200InverseKinematics(obj,rc,Rc)
            % *this function can execute inverse kinematics for the robot [Trimule 200]
            % *Main body of this program comes from [TJU]

            % input:[rc] position of point C w.r.t. the inertial coordinate system 
            %            [3d col/3*n matrix,mm]
            %       [Rc] orientation matrix of TCP w.r.t. the inertial coordinate system
            %            [3*3 matrix/n*3*3 high-order matrix]

            % output: [JointVariables] matrix including all the joint parameters,
            %                          constructed by [theta1i],[theta2i],[q3i],
            %                          [theta4i],[theta5i] and [theta6i], where
            %                          i = 1~4 denotes the index of chain.
            %                          [6*4 matrix,rad and mm]
            % !Note that the unit of angles in joint varialbes is [degree]
            % is program written by [TJU]. This unit is modified as [rad]
            % in our program for consistency.

            % *form of [JointVariables]:[theta11 theta12 theta13 theta14 [rad];
            %                            theta21 theta22 theta23 theta24 [rad];
            %                            q31     q32     q33     q34     [mm];
            %                            theta41 theta42 theta43 theta44 [rad];
            %                            theta51 theta52 theta53 theta54 [rad];
            %                            theta61 theta62 theta63 theta64 [rad]];
            % where [theta ji] denotes the rotation angle of joint [j] on chain
            % [i], and [q ji] corresponds to the length of point [A0i] and point [A5i]
            % on chain [i]. More information is in [1.Robotic geometric model and
            % orientation], and [2. Train of thought of robotic forward and inverse
            % kinematics].

            %%%%%%%%%%%%%%%%%% Execution order check %%%%%%%%%%%%%%%%%%%%%%
            % [RunningFlag] is checked here to ensure the execution order
            if obj.RunningFlag == 1 || obj.RunningFlag == 2 ||...
               obj.RunningFlag == 3 || obj.RunningFlag == 4
            obj.RunningFlag = 2;
            else
                msg = "Error: The execution order is wrong!";
                err(msg)
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%% Form check %%%%%%%%%%%%%%%%%%%%%%%%%
            % forms of received data are checked and modified here to meet the
            % requirement of our program, faciliating the subsequent computation.

            % *form check for [rc], for wich the form of [3d column vector] is desired
            if isrow(rc);rc = rc';end
            % *form check for [Rc], for which the form of [3*3 matrix] is desired

            %%%%%%%%%%%%%%%% Pose of robotic end-effector %%%%%%%%%%%%%%%%%
            % provide the pose of robotic end-effector, including position
            % vector and orientation matrix of TCP coordinate system. All
            % the physical parameters are observed in the inertial
            % coordinate system
               
            obj.rc = rc; % position of TCP w.r.t. the inertial coordinate system [3d col,mm]
            obj.Rc = Rc; % orientation TCP coordinate system projected in the inertial coordinate system [3*3 matrix]
            obj.R64 = Rc;
            Xc = Rc(:,1); % orientation of axis [Xc] in the inertial coordinate system [3d col]
            % Yc = Rc(:,2); % orientation of axis [Yc] in the inertial coordinate system [3d col]
            Zc = Rc(:,3); % orientation of axis [Zc] in the inertial coordinate system [3d col]

            %%%%%%%%%%%%%%% Inverse kinematics calculation %%%%%%%%%%%%%%%%
            % implement inverse kinematics for [Trimule 200] robot. the 
            % following programs are written referring to the [TriMule-200机器人运动学及动力学建模.pdf]
            
            obj.rp = rc-obj.dv*Xc+obj.dw*Zc; % P点的位置矢量
            obj.q34 = norm(obj.rp)-obj.e; % 被动支链第三关节长度，整机基坐标系原点到动平台中点A的距离
            obj.s34 = obj.rp/(obj.q34+obj.e); % 被动支链第三关节的方向矢量，与点P的位置矢量方向相同
            obj.sita14 = atan(-obj.s34(2,1)/obj.s34(3,1)); % 是由R34矩阵得出的，被动支链第一关节转角，注意测量时有一个45°附加值，此量单位为弧度
            sita24 = asin(obj.s34(1,1)); % 被动支链第二关节转角，此量单位为弧度
            % degsita14 = rad2deg(atan(-obj.s34(2,1)/obj.s34(3,1))); % 被动支链第一关节转角，单位为度
            % degsita24 = rad2deg(asin(obj.s34(1,1))); % 被动支链第二关节转角，单位为度
            obj.R34 = [cos(sita24),0,sin(sita24);
                sin(obj.sita14)*sin(sita24),cos(obj.sita14),-sin(obj.sita14)*cos(sita24);
                -cos(obj.sita14)*sin(sita24),sin(obj.sita14),cos(obj.sita14)*cos(sita24)]; % 被动支链第3构件的姿态矩阵
            R6X3 = obj.R34'*Rc; % 被动支链的第6构件相对与第3构件的姿态矩阵，为了求解三自由度转头的三个转角
            if R6X3(1,1)*R6X3(2,1)>0
                sita44 = [atan(-R6X3(1,1)/R6X3(2,1));atan(-R6X3(1,1)/R6X3(2,1))+pi];
            else
                sita44 = [atan(-R6X3(1,1)/R6X3(2,1));atan(-R6X3(1,1)/R6X3(2,1))-pi];
            end
            if R6X3(3,2)*R6X3(3,3)>0
                sita64 = [atan(R6X3(3,2)/R6X3(3,3));atan(-R6X3(3,2)/R6X3(3,3))-pi];
            else
                sita64 = [atan(R6X3(3,2)/R6X3(3,3));atan(-R6X3(3,2)/R6X3(3,3))+pi];
            end
            % 由于在-180-180范围内，求反三角函数应该有两个解，对于malab中atan范围为-90-90，
            % 所以需要判断sin和cos是否同号，以加减PI获得两个解
            sita54 = [acos(R6X3(3,1));-acos(R6X3(3,1))];
            % degsita44 = [rad2deg(atan(-R6X3(1,1)/R6X3(2,1)));
            %              rad2deg(atan(-R6X3(1,1)/R6X3(2,1))+pi)];
            % degsita54 = [rad2deg(acos(R6X3(3,1)));
            %              rad2deg(-acos(R6X3(3,1)))];
            % degsita64 = [rad2deg(atan(-R6X3(3,2)/R6X3(3,3)));
            %              rad2deg(atan(-R6X3(3,2)/R6X3(3,3))+pi)];
            % 至此已经求得被动支链上的全部关节变量，但注意由于是用R6X3的第1列和第3行求解
            % 的三个转角，得到了8个解，这些解中满足R6X3中其余等式的解才是我们需要的解，
            % 所以下面进行判断
            delt12 = zeros(1,8);
            delt13 = zeros(1,8);
            delt22 = zeros(1,8);
            delt23 = zeros(1,8); % R6X3剩余四项和由上式三变量计算值得到的结果之间的误差，如果一组解四个误差均极小，则该解是正解
            obj.zhuantou = zeros(3,8); % 保存最终的解，其余均为0
            l = 0;
            for i = 1:2
                sita4 = sita44(i,1);
                for j = 1:2
                    sita5 = sita54(j,1);
                    for k = 1:2
                        l = l+1;
                        sita6 = sita64(k,1);
                        r12 = cos(sita4)*cos(sita6)-sin(sita4)*cos(sita5)*sin(sita6);
                        r13 = -cos(sita4)*sin(sita6)-sin(sita4)*cos(sita5)*cos(sita6);
                        r22 = sin(sita4)*cos(sita6)+cos(sita4)*cos(sita5)*sin(sita6);
                        r23 = -sin(sita4)*sin(sita6)+cos(sita4)*cos(sita5)*cos(sita6);
                        delt12(1,l) = abs(r12-R6X3(1,2));
                        delt13(1,l) = abs(r13-R6X3(1,3));
                        delt22(1,l) = abs(r22-R6X3(2,2));
                        delt23(1,l) = abs(r23-R6X3(2,3));
                        if delt12(1,l)<0.01&&delt13(1,l)<0.01&&delt22(1,l)<0.01&&delt23(1,l)<0.01
                            obj.zhuantou(1,l) = sita4;
                            obj.zhuantou(2,l) = sita5;
                            obj.zhuantou(3,l) = sita6;
                        end
                    end
                end
            end
            b = [294.16,167,167]; %by、bx
            obj.q3i=[0;0;0]; %储存三条主动支链第三关节变量
            for m = 1:3
                obj.q3i(m,1) = norm(obj.rp+obj.R34*75*[cos(obj.y(1,m));sin(obj.y(1,m));0]-...
                               obj.e*obj.s34-b(1,m)*[cos(obj.y(1,m));sin(obj.y(1,m));0]);
            end
            obj.s31 = (obj.rp+obj.R34*75*[cos(obj.y(1,1));sin(obj.y(1,1));0]-...
                       obj.e*obj.s34-b(1,1)*[cos(obj.y(1,1));sin(obj.y(1,1));0])/obj.q3i(1,1);
            obj.s32 = (obj.rp+obj.R34*75*[cos(obj.y(1,2));sin(obj.y(1,2));0]-...
                       obj.e*obj.s34-b(1,2)*[cos(obj.y(1,2));sin(obj.y(1,2));0])/obj.q3i(2,1);
            obj.s33 = (obj.rp+obj.R34*75*[cos(obj.y(1,3));sin(obj.y(1,3));0]-...
                       obj.e*obj.s34-b(1,3)*[cos(obj.y(1,3));sin(obj.y(1,3));0])/obj.q3i(3,1);
            s3i=  [obj.s31,obj.s32,obj.s33]; %三条主动支链的第三关节的方向矢量
            obj.sita12i = zeros(2,3); %储存三条主动支链1，2关节变量也是由R3i构造的
            for o=1:3
                obj.sita12i(1,o)=atan(-s3i(2,o)/s3i(3,o));
                obj.sita12i(2,o)=asin(s3i(1,o));
            end
            R3i = zeros(3,9); %储存三条主动支链第三构件的姿态矩阵
            obj.R3ifen = mat2cell(R3i,[3,0],[3,3,3]); %将R3i分成3块，每一块为3×3的矩阵，代表对应支链第3构件的姿态矩阵
            for p = 1:3
                obj.R3ifen{1,p} = [cos(obj.sita12i(2,p)),0,sin(obj.sita12i(2,p));
                                   sin(obj.sita12i(1,p))*sin(obj.sita12i(2,p)),cos(obj.sita12i(1,p)),-sin(obj.sita12i(1,p))*cos(obj.sita12i(2,p));
                                  -cos(obj.sita12i(1,p))*sin(obj.sita12i(2,p)),sin(obj.sita12i(1,p)),cos(obj.sita12i(1,p))*cos(obj.sita12i(2,p))];
            end
            obj.R31 = obj.R3ifen{1,1}; %第1支链中第三构件的姿态矩阵
            obj.R32 = obj.R3ifen{1,2}; %第2支链中第三构件的姿态矩阵
            obj.R33 = obj.R3ifen{1,3}; %第3支链中第三构件的姿态矩阵
            R6X3i = zeros(3,9); %为计算主动支链4，5，6关节变变量构建的中间矩阵
            R6X3ifen = mat2cell(R6X3i,[3,0],[3,3,3]); %同样的方式分块
            for q = 1:3
                R6X3ifen{1,q} = obj.R3ifen{1,q}'*obj.R34*[0,0,1;0,1,0;-1,0,0]*...
                                [1,0,0;0,cos(-obj.y(1,q)-pi/2),-sin(-obj.y(1,q)-pi/2);
                                 0,sin(-obj.y(1,q)-pi/2),cos(-obj.y(1,q)-pi/2)];
            end
            obj.sita456i = zeros(3,3); %储存三条支链4，5，6关节变量
            for r = 1:3
                R = R6X3ifen{1,r};
                obj.sita456i(:,r) = [atan(-R(1,3)/R(2,3));
                    asin(R(3,3));
                    atan(-R(3,2)/R(3,1))];
            end
            obj.sita456i(1,1) = obj.sita456i(1,1)+pi*3/2; %这个地方还有点问题，但这个值确实是对的，与测量值差180度
            obj.sita456i(2,1) = pi-obj.sita456i(2,1); %第一支链的这两个角度非常奇怪，是由于1支链和2、3支链的第四关节差90°造成的
            zzjg = zeros(6,4); %对最终结果进行处理
            zzjg([1 2],[1 2 3]) = obj.sita12i;
            zzjg(3,[1 2 3]) = obj.q3i';
            zzjg([4 5 6],[1 2 3]) = obj.sita456i;
            zzjg(1,4) = obj.sita14;
            zzjg(2,4) = sita24;
            zzjg(3,4) = obj.q34;
            zzjg([4 5 6],4) = obj.zhuantou(:,3);
            disanguanjie = zzjg(3,:);
            zuizhongjieguo = zzjg*180/pi;
            zuizhongjieguo(3,:) = disanguanjie;
            zuizhongjieguo([4 5],[2 3]) = zeros(2,2); % joint variables [6*4 matrix, mm and degree]
            % the conclusion that "joint variables of joint 4,5 in chains 1,2
            % are 0" is correct and desired.
            zuizhongjieguo([1 2 4 5 6],:) = zuizhongjieguo([1 2 4 5 6],:)/180*pi;
            % unit of angles in joint variables is changed from [degree] to [rad]
            JointVariables = zuizhongjieguo;
            obj.JointVariables = zuizhongjieguo;
            % matrix in the previously provided form, storing all the 24 joint
            % variables [6*4 matrix, rad and mm]
        end

        function JointVelocities = T200InverseVelocity(obj,vc,w)
            % *this function can solve the velocity issue inversely for the robot [Trimule 200]
            % *Main body of this program comes from [TJU]

            % input:[vc] linear velocity of TCP projected in body attached
            %            frame [Kc'][3d col,mm/s]
            %       [w] angular velocity of TCP coordinate system projected
            %           in body attached frame [Kc'][3d col,rad/s]
            % Known that the vector [vc] and [w] are all free vectors, and
            % the orientation of coordinate systems [K] and [Kc'] are the
            % same, so the observed [vc] and [w] in both coordinate systems
            % are the same. Therefore, the following statements are also
            % correct:
            %       [vc] linear velocity of TCP projected in inertial frame 
            %            [K][3d col,mm/s]
            %       [w] angular velocity of TCP coordinate system projected
            %           in inertial frame [K][3d col,rad/s]

            % output: [JointVelocities] matrix including all the joint velocities,
            %                           constructed by [w1i],[w2i],[v3i],[w4i],
            %                           [w5i] and [w6i], where i = 1~4 denotes the 
            %                           index of chain. 
            %                           [6*4 matrix,rad/s and mm/s]

            % *form of [JointVariables]:[w11 w12 w13 w14 [rad/s];
            %                            w21 w22 w23 w24 [rad/s];
            %                            v31 v32 v33 v34 [mm/s];
            %                            w41 w42 w43 w44 [rad/s];
            %                            w51 w52 w53 w54 [rad/s];
            %                            w61 w62 w63 w64 [rad/s]];
            % where [w ji] denotes the rotation velocity of joint [j] on 
            % chain [i], and [v ji] corresponds to the linear velocity 
            % of joint [j] on chain [i].

            %%%%%%%%%%%%%%%%%% Execution order check %%%%%%%%%%%%%%%%%%%%%%
            % [RunningFlag] is checked here to ensure the execution order
            if obj.RunningFlag == 2 || obj.RunningFlag == 3 || obj.RunningFlag == 4
            obj.RunningFlag = 3;
            else
                msg = "Error: The execution order is wrong!";
                err(msg)
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%% Form check %%%%%%%%%%%%%%%%%%%%%%%%%
            % forms of received data are checked and modified here to meet the
            % requirement of our program, faciliating the subsequent computation.

            % *form check for [vc], for wich the form of [3d column vector] is desired
            if isrow(vc); vc = vc';end
            % linear velocity of TCP projected in inertial coordinate system [3d col,mm/s]
            % *form check for [w], for which the form of [3d column vector] is desired
            if isrow(w); w = w';end
            % angular velcotiy of TCP frame projected in inertial frame [3d col,rad/s]

            %%%%%%%%%%%%%%%%%%%%%%%%%% Form check %%%%%%%%%%%%%%%%%%%%%%%%%
            % forms of received data are checked and modified here to meet the
            % requirement of our program, faciliating the subsequent computation.
            kecitc=[vc;w]; 
            % Velocity screw of end-effector projected in the body attached frame [Kc'] [6-d col,mm/s and rad/s]
            if ~iscolumn(kecitc)
                msg = 'Error: There may be something wrong!';err(msg);
                % confirm that the screw is 6d column vector or not
            end

            %%%%%%%%%%%%%% inverse solution of velocity issue %%%%%%%%%%%%%
            % solve the velocity issue inversely for [Trimule 200] robot. the 
            % following programs are written referring to the [TriMule-200机器人运动学及动力学建模.pdf]
            % 以下计算机器人的速度，包括6个主动关节速度和4条支链所有关节的速度
            
            % *Stage 1: velocities of active joints calculation, including
            % 3 linear velocities on chain 1~3 and 3 angular velocities on
            % chain 4.
            Wap=zeros(6,3); % Wp矩阵的第一块
            ci=zeros(3,3); % 点P到三个球铰中心的矢量
            s3i = [obj.s31,obj.s32,obj.s33];%三条主动支链的第三关节的方向矢量
            for i=1:3
                ci(:,i)=obj.R34*75*[cos(obj.y(1,i));sin(obj.y(1,i));0]-obj.e*obj.s34;%%上面用到过
                Wap(:,i)=[s3i(:,i);cross(ci(:,i),s3i(:,i))];%%参见论文
            end
            qe=obj.q34+obj.e;
            s14=[1;0;0]; % 第四支链第一转轴
            Ryz=[0 0 1;1 0 0;0 1 0];
            obj.R14=Ryz*[cos(obj.sita14) -sin(obj.sita14) 0;sin(obj.sita14) cos(obj.sita14) 0;0 0 1]*Ryz;
            s24=obj.R14(:,3); % 第四支链第二转轴方向向量（都在点P的坐标系下描述，但是其姿态与基坐标系一样）
            keciwc1=[cross(s24,obj.s34);-qe*s24]; % 组成Wp矩阵的第二块
            keciwc2=[s24;qe*cross(s24,obj.s34)]; % 组成Wp矩阵的第二块
            keciwc3=[0;0;0;cross(s14,s24)]; % 组成Wp矩阵的第二块
            Wcp=[keciwc1 keciwc2 keciwc3];
            Wp=[Wap Wcp];
            Tp=inv(Wp');
            Tpfen=mat2cell(Tp,[6,0],[3,3]); % 将其分块
            obj.Tap=Tpfen{1,1}; % 得到Tap是雅可比的第一块
            obj.kecita44=[0;0;0;obj.s34]; % 第四轴许动位移旋量
            obj.R44=obj.R34*[cos(obj.zhuantou(1,3)) -sin(obj.zhuantou(1,3)) 0;...
                         sin(obj.zhuantou(1,3)) cos(obj.zhuantou(1,3)) 0;0 0 1]*Ryz;
            s54=obj.R44(:,3);
            obj.kecita54=[0;0;0;s54]; % 第五轴许动位移旋量
            obj.R54=obj.R64*[0 0 1;1 0 0;0 1 0]*[cos(obj.zhuantou(3,3)),sin(obj.zhuantou(3,3)),0;
                        -sin(obj.zhuantou(3,3)),cos(obj.zhuantou(3,3)),0;0,0,1];
            s64=obj.R54(:,3);
            obj.kecita64=[0;0;0;s64]; % 第六轴许动位移旋量
            yakebiP=[obj.Tap,obj.kecita44,obj.kecita54,obj.kecita64];%%机器人雅可比矩阵
            obj.rcp=obj.rp-obj.rc; % 点C到点P的矢量
            X=[eye(3),[0 -obj.rcp(3,1) obj.rcp(2,1);obj.rcp(3,1) 0 -obj.rcp(1,1);
              -obj.rcp(2,1) obj.rcp(1,1) 0];zeros(3,3),eye(3)];%%伴随变换矩阵
            T=X*yakebiP; % 相对于C点的雅可比矩阵
            kecitc = kecitc; % screw of end-effector projected in the body attached frame [Kc'][6d col,mm/s and rad/s]
            obj.zhudongguanjiesudu=T\kecitc; % 6个主动关节的速度
            % velocities of active joints is in the form of the following column 
            %
            %      zhudongguanjiesudu = [v31,v32,v33,w44,w54,w64]'
            %
            % where [v31,v32,v33]' are linear velocities of 电推杆 on the 
            % chain 1~3, and [w44,w54,w64]' are angular velocities of joints
            % in the 摆角头 [6d col,mm/s and rad/mm]

            % *Stage2: velocities of all joints calculation, 
            % sitaaidian=zeros(6,3); % 三条主动支链各个关节的速度
            obj.kecitai1=zeros(6,6); % 支链1的6个单位许动位移旋量
            obj.kecitai2=zeros(6,6); % 支链2的6个单位许动位移旋量
            obj.kecitai3=zeros(6,6); % 支链3的6个单位许动位移旋量
            Ti=[obj.kecitai1,obj.kecitai2,obj.kecitai3]; % 储存三条主动支链的18个微小许动位移旋量
            obj.TiFen=mat2cell(Ti,[6,0],[6,6,6]); % 将其分块
            s1i=[1 1 1;0 0 0;0 0 0]; % 三条主动支链的第一关节找轴线方向
            R11=Ryz*[cos(obj.sita12i(1,1)) -sin(obj.sita12i(1,1)) 0;
                     sin(obj.sita12i(1,1)) cos(obj.sita12i(1,1)) 0;0 0 1]*Ryz; % 为了得到第一支链第二关节轴线方向
            s2i=[R11(:,3),s24,s24]; % 三条支链第二关节轴线方向
            R4i=zeros(3,9); % 用于储存三条支链第四构件姿态矩阵，获取第5关节轴线方向
            R4iFen=mat2cell(R4i,[3,0],[3,3,3]); % 分成三块，三条支链
            for i=1:3
                R4iFen{1,i}=obj.R3ifen{1,i}*[cos(obj.sita456i(1,i)+obj.y(1,i)),...
                                            -sin(obj.sita456i(1,i)+obj.y(1,i)),...
                                             0;
                                             sin(obj.sita456i(1,i)+obj.y(1,i)),...
                                             cos(obj.sita456i(1,i)+obj.y(1,i)),...
                                             0;...
                                             0,0,1]*Ryz;
            end
            R41=R4iFen{1,1};
            R42=R4iFen{1,2};
            R43=R4iFen{1,3}; % 三条支链第四构件姿态矩阵
            R4i=[R41,R42,R43];
            R5i=zeros(3,9); % 用于储存三条支链第五构件姿态矩阵，获取第6关节轴线方向
            R5iFen=mat2cell(R5i,[3,0],[3,3,3]);
            for i=1:3
                R5iFen{1,i}=R4iFen{1,i}*[cos(obj.sita456i(2,i)),-sin(obj.sita456i(2,i)),0;
                                         sin(obj.sita456i(2,i)),cos(obj.sita456i(2,i)),0;0,0,1]*Ryz;
            end % 依然是支链1的有点奇怪
            R51=R5iFen{1,1};
            R52=R5iFen{1,2};
            R53=R5iFen{1,3}; % 三条支链第5构件姿态矩阵
            R5i=[R51,R52,R53];
            for i=1:3
                obj.TiFen{1,i}(:,1)=[cross(ci(:,i)-obj.q3i(i,1)*s3i(:,i),s1i(:,i));s1i(:,i)];
                obj.TiFen{1,i}(:,2)=[cross(ci(:,i)-obj.q3i(i,1)*s3i(:,i),s2i(:,i));s2i(:,i)];
                obj.TiFen{1,i}(:,3)=[s3i(:,i);0;0;0];
                obj.TiFen{1,i}(:,4)=[cross(ci(:,i),s3i(:,i));s3i(:,i)];
                obj.TiFen{1,i}(:,5)=[cross(ci(:,i),R4i(:,3*i));R4i(:,3*i)];
                obj.TiFen{1,i}(:,6)=[cross(ci(:,i),R5i(:,3*i));R5i(:,3*i)];
            end % 构造Ti
            obj.kecitai1=obj.TiFen{1,1};
            obj.kecitai2=obj.TiFen{1,2};
            obj.kecitai3=obj.TiFen{1,3};
            Ti=[obj.kecitai1,obj.kecitai2,obj.kecitai3];
            obj.zhudongzhiliansudu=zeros(6,3);
            for i=1:3
                obj.zhudongzhiliansudu(:,i) = obj.TiFen{1,i}\(obj.Tap*obj.zhudongguanjiesudu([1 2 3],1));
            end % 计算三条主动支链中所有关节的速度

            obj.kecita14=[-qe*cross(obj.s34,s14);s14]; % 计算被动支链前3关节的速度
            obj.kecita24=[qe*cross(s24,obj.s34);s24];
            obj.kecita34=[obj.s34;0;0;0];
            kecitc14=[cross(s24,obj.s34);0;0;0];
            kecitc24=[s24;0;0;0];
            n34=cross(s14,s24);
            kecitc34=[-qe*cross(obj.s34,n34);n34];
            Ta4=[obj.kecita14,obj.kecita24,obj.kecita34];
            Tc4=[kecitc14,kecitc24,kecitc34];
            T4=[Ta4,Tc4];
            W=inv(T4');
            obj.Wa4=W(:,[1 2 3]);
            obj.beidongzhiliansudu = obj.Wa4'*obj.Tap*obj.zhudongguanjiesudu([1 2 3],1); % 计算被动支链前3关节的速度
            JointVelocities = [obj.zhudongzhiliansudu,[obj.beidongzhiliansudu;obj.zhudongguanjiesudu((end/2)+1:end)]];
            % joint velocities include [zhudongzhliiansudu],
            % [beidongzhiliansudu] of the first 3 joints, 
            % and the last 3 velocities in [zhudongguanjiesudu], exactly
            % the [w44,w54,w64]
            if (size(JointVelocities,1) ~= 6)||(size(JointVelocities,2) ~= 4)
                msg = 'Error: There may be something wrong!'; err(msg)
                % confirm whether the size of [JointVariables] is [6*4] or not
            end
            obj.JointVelocities = JointVelocities; % velocities of all joints [6*4 matrix rad/s mm/s]
        end

        function [JointAccelerations,DrivingAcceleration] = T200InverseAcceleration(obj,ac,alpha)
            % *this function can solve the acceleration issue inversely for the robot [Trimule 200]
            % *Main body of this program comes from [TJU]

            % input:[ac] linear acceleration of point C in body attached 
            %            frame [Kc'][3d col,mm/s^2]
            %            Note that this acceleration is [spatial acceleration]
            %            but not [material acceleration]
            %       [alpha] angular acceleration of TCP coordinate system
            %               projected in body attached frame [Kc']
            %               [3d col,rad/s]
            % 
            % *The same as [vc] and [w],the following statements are also 
            % correct for [ac] and [alpha]
            %       [ac] linear acceleration of TCP projected in inertial 
            %            frame [K][3d col,mm/s]
            %            Note that this acceleration is [spatial acceleration]
            %            but not [material acceleration]
            %       [alpha] angular acceleration of TCP coordinate system 
            %               projected in inertial frame [K][3d col,rad/s]
            % 
            % *[material acceleration] and [spatial acceleration]
            %  the [material acceleration] is exactly the actual linear
            % acceleration of TCP projected in frame [Kc'] or [K], 
            % comprising [spatial acceleration] and [centripetal
            % acceleration]. The [sptial acceleration] is the linear
            % acceleration caused by rotation acceleration and can be
            % calculated as [alpha*radius]. Moreover, the [centripetal
            % acceleration] describes the acceleration of centripetal
            % force. In the derivation of acceleration screw, the
            % [centripetal acceleration] is excluded from the outcomes, 
            % and only the [spatial acceleration] takes part in
            % constructing the acceleration screw. More information is in
            % [Note 5] and [global appendix 9]
            %
            % *In summary, the [material acceleration] is the combination
            % of the [centripetal acceleration] and [spatial
            % acceleration], where the latter is the linear acceleration
            % caused by rotation acceleration.

            % output: [JointAccelerations] matrix including all the joint accelerations,
            %                              constructed by [alpha1i],[alpha2i],[a3i],[alpha4i],
            %                              [alpha5i] and [alpha6i], where i = 1~4 denotes the
            %                              index of chain.
            %                              [6*4 matrix,rad/s^2 and mm/s^2]
            %         [DrivingAccelerations] accelerations of driving
            %                                joints, which can used in 
            %                                dynamic modeling
            %                                [6d col,rad/s^2 and mm/s^2]

            % *form of [JointVariables]:[alpha11 alpha12 alpha13 alpha14 [rad/s^2];
            %                            alpha21 alpha22 alpha23 alpha24 [rad/s^2];
            %                            a31     a32     a33     a34     [mm/s^2];
            %                            alpha41 alpha42 alpha43 alpha44 [rad/s^2];
            %                            alpha51 alpha52 alpha53 alpha54 [rad/s^2];
            %                            alpha61 alpha62 alpha63 alpha64 [rad/s^2]];
            % where [alpha ji] denotes the rotation acceleration of joint 
            % [j] on chain [i], and [a ji] corresponds to linear 
            % acceleration of joint [j] on chain [i].

            %%%%%%%%%%%%%%%%%% Execution order check %%%%%%%%%%%%%%%%%%%%%%
            % [RunningFlag] is checked here to ensure the execution order
            if obj.RunningFlag == 3 || obj.RunningFlag == 4
            obj.RunningFlag = 4;
            else
                msg = "Error: The execution order is wrong!";
                err(msg)
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%% Form check %%%%%%%%%%%%%%%%%%%%%%%%%
            % forms of received data are checked and modified here to meet the
            % requirement of our program, faciliating the subsequent computation.

            % *form check for [ac], for wich the form of [3d column vector] is desired
            if isrow(ac); ac = ac';end
            % spatial acceleration of TCP projected in inertial coordinate system [3d col,mm/s]
            % *form check for [alpha], for which the form of [3d column vector] is desired
            if isrow(alpha); alpha = alpha';end
            % angular acceleration of TCP frame projected in inertial frame [3d col,rad/s]

            %%%%%%%%%%%%%%%%%%%%%%%%%% Form check %%%%%%%%%%%%%%%%%%%%%%%%%
            % forms of received data are checked and modified here to meet the
            % requirement of our program, faciliating the subsequent computation.
            keciac = [ac;alpha]; 
            % Acceleration screw of end-effector projected in the body attached frame [Kc'] [6-d col,mm/s and rad/s]
            if ~iscolumn(keciac)
                msg = 'Error: There may be something wrong!';err(msg);
                % confirm that the screw is 6d column vector or not
            end
 
            %%%%%%%%%%%%%% inverse solution of velocity issue %%%%%%%%%%%%%
            % solve the acceleration issue inversely for [Trimule 200] 
            % robot. the following programs are written referring to 
            % the [TriMule-200机器人运动学及动力学建模.pdf]

            obj.kecit1=zeros(6,6); % 支链1的速度旋量
            obj.kecit1(:,1)=obj.zhudongzhiliansudu(1,1)*obj.kecitai1(:,1);
            for i=2:6
                obj.kecit1(:,i)=obj.kecit1(:,i-1)+obj.zhudongzhiliansudu(i,1)*obj.kecitai1(:,i);
            end
            obj.T1weifen=zeros(6,6); % 支链1的雅可比矩阵的微分
            obj.kecit21chacheng=zeros(6,6);
            obj.kecit31chacheng=zeros(6,6);
            for k=1:6
                omg=[0,-obj.kecit1(6,k),obj.kecit1(5,k);obj.kecit1(6,k),0,-obj.kecit1(4,k);-obj.kecit1(5,k),obj.kecit1(4,k),0];
                vi=[0,-obj.kecit1(3,k),obj.kecit1(2,k);obj.kecit1(3,k),0,-obj.kecit1(1,k);-obj.kecit1(2,k),obj.kecit1(1,k),0];
                chacheng=[omg,vi;zeros(3,3),omg];
                if k==2
                    obj.kecit21chacheng=chacheng;
                elseif k==3
                    obj.kecit31chacheng=chacheng;
                end
                obj.T1weifen(:,k)=chacheng*obj.kecitai1(:,k);
            end

            obj.kecit2=zeros(6,6); % 支链2的速度旋量
            obj.kecit2(:,1)=obj.zhudongzhiliansudu(1,2)*obj.kecitai2(:,1);
            for i=2:6
                obj.kecit2(:,i)=obj.kecit2(:,i-1)+obj.zhudongzhiliansudu(i,2)*obj.kecitai2(:,i);
            end
            obj.T2weifen=zeros(6,6); % 支链2的雅可比矩阵的微分
            obj.kecit22chacheng=zeros(6,6);
            obj.kecit32chacheng=zeros(6,6);
            for k=1:6
                omg=[0,-obj.kecit2(6,k),obj.kecit2(5,k);obj.kecit2(6,k),0,-obj.kecit2(4,k);-obj.kecit2(5,k),obj.kecit2(4,k),0];
                vi=[0,-obj.kecit2(3,k),obj.kecit2(2,k);obj.kecit2(3,k),0,-obj.kecit2(1,k);-obj.kecit2(2,k),obj.kecit2(1,k),0];
                chacheng=[omg,vi;zeros(3,3),omg];
                if k==2
                    obj.kecit22chacheng=chacheng;
                elseif k==3
                    obj.kecit32chacheng=chacheng;
                end
                obj.T2weifen(:,k)=chacheng*obj.kecitai2(:,k);
            end

            obj.kecit3=zeros(6,6); % 支链3的速度旋量
            obj.kecit3(:,1)=obj.zhudongzhiliansudu(1,3)*obj.kecitai3(:,1);
            for i=2:6
                obj.kecit3(:,i)=obj.kecit3(:,i-1)+obj.zhudongzhiliansudu(i,3)*obj.kecitai3(:,i);
            end
            obj.T3weifen=zeros(6,6); % 支链3的雅可比矩阵的微分
            obj.kecit23chacheng=zeros(6,6);
            obj.kecit33chacheng=zeros(6,6);
            for k=1:6
                omg=[0,-obj.kecit3(6,k),obj.kecit3(5,k);obj.kecit3(6,k),0,-obj.kecit3(4,k);-obj.kecit3(5,k),obj.kecit3(4,k),0];
                vi=[0,-obj.kecit3(3,k),obj.kecit3(2,k);obj.kecit3(3,k),0,-obj.kecit3(1,k);-obj.kecit3(2,k),obj.kecit3(1,k),0];
                chacheng=[omg,vi;zeros(3,3),omg];
                if k==2
                    obj.kecit23chacheng=chacheng;
                elseif k==3
                    obj.kecit33chacheng=chacheng;
                end
                obj.T3weifen(:,k)=chacheng*obj.kecitai3(:,k);
            end

            obj.kecit4=zeros(6,6); % 支链4的速度旋量
            obj.sudu4=[obj.beidongzhiliansudu;obj.zhudongguanjiesudu([4 5 6],1)];
            kecita4=[obj.kecita14,obj.kecita24,obj.kecita34,obj.kecita44,obj.kecita54,obj.kecita64];
            obj.kecit4(:,1)=obj.sudu4(1,1)*kecita4(:,1);
            for i=2:6
                obj.kecit4(:,i)=obj.kecit4(:,i-1)+obj.sudu4(i,1)*kecita4(:,i);
            end
            obj.T4weifen=zeros(6,6); % 支链4的雅可比矩阵的微分
            obj.kecit14chacheng=zeros(6,6);
            obj.kecit34chacheng=zeros(6,6);
            obj.kecit44chacheng=zeros(6,6);
            obj.kecit54chacheng=zeros(6,6);
            obj.kecit64chacheng=zeros(6,6);
            for k=1:6
                omg=[0,-obj.kecit4(6,k),obj.kecit4(5,k);obj.kecit4(6,k),0,-obj.kecit4(4,k);-obj.kecit4(5,k),obj.kecit4(4,k),0];
                vi=[0,-obj.kecit4(3,k),obj.kecit4(2,k);obj.kecit4(3,k),0,-obj.kecit4(1,k);-obj.kecit4(2,k),obj.kecit4(1,k),0];
                chacheng=[omg,vi;zeros(3,3),omg];
                if k==1
                    obj.kecit14chacheng=chacheng;
                elseif k==3
                    obj.kecit34chacheng=chacheng;
                elseif k==4
                    obj.kecit44chacheng=chacheng;
                elseif k==5
                    obj.kecit54chacheng=chacheng;
                elseif k==6
                    obj.kecit64chacheng=chacheng;
                end
                obj.T4weifen(:,k)=chacheng*kecita4(:,k);
            end
            keciac = keciac; % 点C的加速度旋量, have been defined in the beginning
            jiasudu4 = kecita4\(keciac-obj.T4weifen*obj.sudu4); % 支链4的6个关节的角加速度

            % 计算动平台关于点P的加速度旋量
            keciap=obj.T4weifen(:,[1 2 3])*obj.beidongzhiliansudu+kecita4(:,[1 2 3])*jiasudu4([1 2 3]);
            % 计算1 2 3三条主动支链各关节加速度
            jiasudu1 = obj.kecitai1\(keciap-obj.T1weifen*obj.zhudongzhiliansudu(:,1));
            % jiasudu1([1 2 4 5 6])=jiasudu1([1 2 4 5 6])*180/pi;%%支链1关节5、6加速度1.5s后，与测量值差一个正负号，程序最后有解释，这里暂时进行修正
            % unit of joint in [jiasudu1] is kept as [rad/s^2]
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % ! Note that there may be something wrong in the sign, this 
            % prompt is left here as a reference.
            % ------------------------------------------------------------
            % if time(1,s+1)>=1.5
            %     jiasudu1([5 6],s+1)=-jiasudu1([5 6],s+1);%%解决这一问题的根本方法需要修改128、129行
            % end 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            jiasudu2 = obj.kecitai2\(keciap-obj.T2weifen*obj.zhudongzhiliansudu(:,2));
            % jiasudu2([1 2 4 5 6])=jiasudu2([1 2 4 5 6])*180/pi;
            % unit of joint in [jiasudu2] is kept as [rad/s^2]
            jiasudu3 = obj.kecitai3\(keciap-obj.T3weifen*obj.zhudongzhiliansudu(:,3));
            % jiasudu3([1 2 4 5 6])=jiasudu3([1 2 4 5 6])*180/pi;
            % unit of joint in [jiasudu3] is kept as [rad/s^2]
            JointAccelerations = [jiasudu1,jiasudu2,jiasudu3,jiasudu4];
            % matrix including accelerations of all joints on 4 chains [6*4
            % matrix, rad/s^2, mm/s^2]
            DrivingAcceleration = [jiasudu1(3);jiasudu2(3);jiasudu3(3);jiasudu4([4,5,6])];
            % accelerations of driving joints [6d col, rad/s^2 and mm/s^2]
        end

        function [Dq,Hq,Gq,DrivingForces] = T200Dynamics(obj,DrivingAccelerations)
            % *this function establishes dynamic model for the robot [Trimule 200]
            % *Main body of this program comes from [TJU]

            % input:[DrivingAccelerations] Accelerations of driving joints
            %                              [6d col,rad/s^2,mm/s^2]
            % !Note that the acceleration data accepted by this dynamic 
            % model is [deg/s^2,mm/s^2]. So a unit conversion is necessary.
            %       [kinematic, velocity and acceleration information] 
            % !Note that the kinematic, velocity, and acceleration are also
            % essential parts for constructing the dynamic model. They can
            % be generated by function above. Therefore, this function 
            % [T200Dynamics] should by run after functions above.   
            % 
            % output:[D] matrix D including inertial properties of robot [6*6 matrix]
            %            its matching angular acceleration is [deg/s^2]
            %        [H] vector H reflecting Coriolis forces and centrifugal forces [6d col]
            %        [G] vector G presenting influences caused by gravity [6d col]
            %        [DrivingForces] forces of driving joints [6d col,N,N*mm]
            %
            % *the six driving joints mentioned here is three prismatic
            %  joints on chain1~3 and three rotation joints on 摆角头 on
            %  chain 4. Units of their accelerations are [mm/s^2] and
            %  [rad/s^2], which are modified into [deg/s^2] and [mm/s^2] to
            %  match the established dynamic model. Units of their driving
            %  forces are [N] and [N*mm].    

            %%%%%%%%%%%%%%%%%% Execution order check %%%%%%%%%%%%%%%%%%%%%%
            % [RunningFlag] is checked here to ensure the execution order
            if obj.RunningFlag == 4
            obj.RunningFlag = 1;
            else
                msg = "Error: The execution order is wrong!";
                err(msg)
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%% Form check %%%%%%%%%%%%%%%%%%%%%%%%%
            % forms of received data are checked and modified here to meet the
            % requirement of our program, faciliating the subsequent computation.

            % *form check for [DrivingAccelerations], for wich the form of [6d column vector] is desired
            if isrow(DrivingAccelerations); DrivingAccelerations = DrivingAccelerations';end

            %%%%%%%%%%%%%%%%%%%%%% Dynamic modeling %%%%%%%%%%%%%%%%%%%%%%%
            % Matrices and vectors involved in robotic dynamic model is 
            % generated in the rest. the following programs are written 
            % referring to the [TriMule-200机器人运动学及动力学建模.pdf]
            % All necessary inertial properties are defined during
            % programing.
            %%以下计算刚体动力学
            %%首先根据牛顿-欧拉方程、虚功原理列写方程
            I1ijubu=[136369297 0 0;
                     0 136344677 0;
                     0 0 2094913];
            %%三条主动支链第一构件(套筒)在局部坐标系下的惯性张量%%%%%%%%%%%%%%%%【】
            I14jubu=[102854706 0 0;
                     0 16869091 207422;
                     0 207422 91255649];
            %%被动支链第一构件(转动架)惯性张量%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
            m1i=4833;%%套筒质量%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
            m14=5324;%%转动架质量%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】

            M1ijubu=[m1i*eye(3),zeros(3,3);zeros(3,3),I1ijubu];%%构建广义惯性矩阵
            M14jubu=[m14*eye(3),zeros(3,3);zeros(3,3),I14jubu];
            g11=(obj.R31*[0;0;-18]+[0;-294.16;0]-obj.rp);%%支链1构件1的重心，在参考坐标系下的坐标
            g12=(obj.R32*[0;0;-18]+[167;0;0]-obj.rp);
            g13=(obj.R33*[0;0;-18]+[-167;0;0]-obj.rp);
            g14=(obj.R14*[0;0;-5.6]-obj.rp);
            g11chacheng=[0,-g11(3,1),g11(2,1);g11(3,1),0,-g11(1,1);-g11(2,1),g11(1,1),0];
            g12chacheng=[0,-g12(3,1),g12(2,1);g12(3,1),0,-g12(1,1);-g12(2,1),g12(1,1),0];
            g13chacheng=[0,-g13(3,1),g13(2,1);g13(3,1),0,-g13(1,1);-g13(2,1),g13(1,1),0];
            g14chacheng=[0,-g14(3,1),g14(2,1);g14(3,1),0,-g14(1,1);-g14(2,1),g14(1,1),0];
            Adg11=[obj.R31,g11chacheng*obj.R31;zeros(3,3),obj.R31];%%从局部坐标系到参考坐标系的伴随变换矩阵
            Adg12=[obj.R32,g12chacheng*obj.R32;zeros(3,3),obj.R32];
            Adg13=[obj.R33,g13chacheng*obj.R33;zeros(3,3),obj.R33];
            Adg14=[obj.R14,g14chacheng*obj.R14;zeros(3,3),obj.R14];
            M11=inv(Adg11')*M1ijubu*inv(Adg11);%%在参考坐标系下的广义惯性矩阵
            M12=inv(Adg12')*M1ijubu*inv(Adg12);
            M13=inv(Adg13')*M1ijubu*inv(Adg13);
            M14=inv(Adg14')*M14jubu*inv(Adg14);

            I2ijubu=[17984609,0,0;
                     0,17999476,0;
                     0,0,195625];
            %%主动支链第二构件(推杆)局部坐标系下的惯性张量%%%%%%%%%%%%%%%%%%%%%【】
            I24jubu=[689668.8,9.917,-2588.8;
                     9.917,690657.3,6302;
                     -2588.8,6302,28627.96];
            %%被动支链第二构件（支链体）在局部坐标系下的惯性张量%%%%%%%%%%%%%%%%【】
            m2i=1263;%%推杆质量%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
            m24=15379;%%支链体质量%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】

            M2ijubu=[m2i*eye(3),zeros(3,3);zeros(3,3),I2ijubu];%%构建局部坐标系下的广义惯性矩阵
            M24jubu=[m24*eye(3),zeros(3,3);zeros(3,3),I24jubu];
            g21=(obj.R31*[0;0;obj.q3i(1,1)-209.5]+[0;-294.16;0]-obj.rp);%%在参考坐标系下的重心坐标
            g22=(obj.R32*[0;0;obj.q3i(2,1)-209.5]+[167;0;0]-obj.rp);
            g23=(obj.R33*[0;0;obj.q3i(3,1)-209.5]+[-167;0;0]-obj.rp);
            g24=(obj.R34(:,3)*(obj.q34-208)-obj.rp);
            g21chacheng=[0,-g21(3,1),g21(2,1);g21(3,1),0,-g21(1,1);-g21(2,1),g21(1,1),0];
            g22chacheng=[0,-g22(3,1),g22(2,1);g22(3,1),0,-g22(1,1);-g22(2,1),g22(1,1),0];
            g23chacheng=[0,-g23(3,1),g23(2,1);g23(3,1),0,-g23(1,1);-g23(2,1),g23(1,1),0];
            g24chacheng=[0,-g24(3,1),g24(2,1);g24(3,1),0,-g24(1,1);-g24(2,1),g24(1,1),0];
            Adg21=[obj.R31,g21chacheng*obj.R31;zeros(3,3),obj.R31];%%从各连杆系到参考系的伴随变换矩阵
            Adg22=[obj.R32,g22chacheng*obj.R32;zeros(3,3),obj.R32];
            Adg23=[obj.R33,g23chacheng*obj.R33;zeros(3,3),obj.R33];
            Adg24=[obj.R34,g24chacheng*obj.R34;zeros(3,3),obj.R34];
            M21=inv(Adg21')*M2ijubu*inv(Adg21);%%在参考坐标系下的广义惯性矩阵
            M22=inv(Adg22')*M2ijubu*inv(Adg22);
            M23=inv(Adg23')*M2ijubu*inv(Adg23);
            M24=inv(Adg24')*M24jubu*inv(Adg24);

            %%为了计算Ta,j,i
            Ta11pie=obj.kecitai1(:,[1 2]);%%注意动力学考虑的第一各构件其实是运动学中的第二构件(套筒)
            Ta21pie=obj.kecitai1(:,[1 2 3]);
            Ta12pie=obj.kecitai2(:,[1 2]);
            Ta22pie=obj.kecitai2(:,[1 2 3]);
            Ta13pie=obj.kecitai3(:,[1 2]);
            Ta23pie=obj.kecitai3(:,[1 2 3]);
            Ta14pie=obj.kecita14;
            Ta24pie=[obj.kecita14,obj.kecita24,obj.kecita34];
            E11=[eye(2),zeros(2,4)];
            E21=[eye(3),zeros(3,3)];
            E12=[eye(2),zeros(2,4)];
            E22=[eye(3),zeros(3,3)];
            E13=[eye(2),zeros(2,4)];
            E23=[eye(3),zeros(3,3)];
            E14=[1 0 0];
            E24=eye(3);
            Wa1=inv(obj.TiFen{1,1})';
            Wa2=inv(obj.TiFen{1,2})';
            Wa3=inv(obj.TiFen{1,3})';
            Ta11=Ta11pie*E11*Wa1'*obj.Tap;% 至此可以求Ta,j,i了
            Ta21=Ta21pie*E21*Wa1'*obj.Tap;
            Ta12=Ta12pie*E12*Wa2'*obj.Tap;
            Ta22=Ta22pie*E22*Wa2'*obj.Tap;
            Ta13=Ta13pie*E13*Wa3'*obj.Tap;
            Ta23=Ta23pie*E23*Wa3'*obj.Tap;
            Ta14=Ta14pie*E14*obj.Wa4'*obj.Tap;
            Ta24=Ta24pie*E24*obj.Wa4'*obj.Tap;

            Is=11309;% 这是丝杠和联轴器绕自身轴线的转动惯量%%%%%%%%%%%%%%%%%%%【】
                     % 注意在SW中没有丝杠旋转，肯能导致误差
            daocheng=4;% 丝杠的导程%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】

            D3=Ta11'*M11*Ta11+Ta21'*M21*Ta21+Ta12'*M12*Ta12+Ta22'*M22*Ta22+Ta13'*M13*Ta13+Ta23'*M23*Ta23+...
                Ta14'*M14*Ta14+Ta24'*M24*Ta24+eye(3)*Is*(2*pi/daocheng)^2;

            sigemaa11=obj.T1weifen(:,[1 2])*obj.zhudongzhiliansudu([1 2],1); % 这就是计算加速时的第二项
            sigemaa21=obj.T1weifen(:,[1 2 3])*obj.zhudongzhiliansudu([1 2 3],1);
            sigemaa12=obj.T2weifen(:,[1 2])*obj.zhudongzhiliansudu([1 2],2);
            sigemaa22=obj.T2weifen(:,[1 2 3])*obj.zhudongzhiliansudu([1 2 3],2);
            sigemaa13=obj.T3weifen(:,[1 2])*obj.zhudongzhiliansudu([1 2],3);
            sigemaa23=obj.T3weifen(:,[1 2 3])*obj.zhudongzhiliansudu([1 2 3],3);
            sigemaa14=obj.T4weifen(:,1)*obj.beidongzhiliansudu(1);
            sigemaa24=obj.T4weifen(:,[1 2 3])*obj.beidongzhiliansudu([1 2 3]);
            H3=Ta11'*(M11*sigemaa11-obj.kecit21chacheng'*M11*obj.kecit1(:,2))+... % 构建H3
                Ta21'*(M21*sigemaa21-obj.kecit31chacheng'*M21*obj.kecit1(:,3))+...
                Ta12'*(M12*sigemaa12-obj.kecit22chacheng'*M12*obj.kecit2(:,2))+...
                Ta22'*(M22*sigemaa22-obj.kecit32chacheng'*M22*obj.kecit2(:,3))+...
                Ta13'*(M13*sigemaa13-obj.kecit23chacheng'*M13*obj.kecit3(:,2))+...
                Ta23'*(M23*sigemaa23-obj.kecit33chacheng'*M23*obj.kecit3(:,3))+...
                Ta14'*(M14*sigemaa14-obj.kecit14chacheng'*M14*obj.kecit4(:,1))+...
                Ta24'*(M24*sigemaa24-obj.kecit34chacheng'*M24*obj.kecit4(:,3));

            zhongli=[0;-sqrt(2)/2;sqrt(2)/2]; % 重力的方向向量
            G3=-(Ta11'*4833*9.8*10^(3)*[zhongli;cross(g11,zhongli)]+Ta21'*1263*9.8*10^(3)*[zhongli;cross(g21,zhongli)]+... % 构建G3
                Ta12'*4833*9.8*10^(3)*[zhongli;cross(g12,zhongli)]+Ta22'*1263*9.8*10^(3)*[zhongli;cross(g22,zhongli)]+...
                Ta13'*4833*9.8*10^(3)*[zhongli;cross(g13,zhongli)]+Ta23'*1263*9.8*10^(3)*[zhongli;cross(g23,zhongli)]+...
                Ta14'*5324*9.8*10^(3)*[zhongli;cross(g14,zhongli)]+Ta24'*15379*9.8*10^(3)*[zhongli;cross(g24,zhongli)]);

            I44jubu=[78253267 -4919741 1033606;
                    -4919741 45881663 2580650;
                     1033606 2580650 56855084]; 
            % 转头第一构件在局部坐标系下的惯性张量%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
            g44jubu=[-11.11;-83.47;2.7]; % 局部坐标系下重心坐标%%%%%%%%%%%%%【】
            m44=7540; % 构件质量%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
            I54jubu=[15011772 0 0;
                     0 14884080 0;
                     0 0 6173188];%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%【】
            g54jubu=[0;0;9.34];
            m54=4258;
            I64jubu=[2912854 0 154973;0 3111333 0;154973 0 1675256];
            g64jubu=[-4.1;0;111.73];
            m64=1819;
            M44jubu=[m44*eye(3),zeros(3,3);zeros(3,3),I44jubu]; % 构建局部坐标系下的广义惯性矩阵
            M54jubu=[m54*eye(3),zeros(3,3);zeros(3,3),I54jubu];
            M64jubu=[m64*eye(3),zeros(3,3);zeros(3,3),I64jubu];
            g44=obj.R44*g44jubu; % 在参考坐标系下的重心坐标
            g54=obj.R54*g54jubu;
            g64=obj.R64*g64jubu-obj.rcp;
            g44chacheng=[0,-g44(3,1),g44(2,1);g44(3,1),0,-g44(1,1);-g44(2,1),g44(1,1),0];
            g54chacheng=[0,-g54(3,1),g54(2,1);g54(3,1),0,-g54(1,1);-g54(2,1),g54(1,1),0];
            g64chacheng=[0,-g64(3,1),g64(2,1);g64(3,1),0,-g64(1,1);-g64(2,1),g64(1,1),0];
            Adg44=[obj.R44,g44chacheng*obj.R44;zeros(3,3),obj.R44]; % 连杆系相对于参考系的伴随变换矩阵
            Adg54=[obj.R54,g54chacheng*obj.R54;zeros(3,3),obj.R54];
            Adg64=[obj.R64,g64chacheng*obj.R64;zeros(3,3),obj.R64];
            M44=inv(Adg44')*M44jubu*inv(Adg44); % 参考系下的广义惯性矩阵
            M54=inv(Adg54')*M54jubu*inv(Adg54);
            M64=inv(Adg64')*M64jubu*inv(Adg64);
            Ta44=[obj.Tap,obj.kecita44];
            Ta54=[Ta44,obj.kecita54];
            Ta64=[Ta54,obj.kecita64];
            D4=Ta44'*M44*Ta44; % 暂时不考虑传动部件的转动惯量，可能造成误差
            D5=Ta54'*M54*Ta54;
            D6=Ta64'*M64*Ta64;

            sigemaa44=obj.T4weifen(:,[1 2 3 4])*obj.sudu4([1 2 3 4],1); % 等同于求加速度式中的第二项
            sigemaa54=obj.T4weifen(:,[1 2 3 4 5])*obj.sudu4([1 2 3 4 5],1);
            sigemaa64=obj.T4weifen(:,[1 2 3 4 5 6])*obj.sudu4;
            H4=Ta44'*(M44*sigemaa44-obj.kecit44chacheng'*M44*obj.kecit4(:,4));
            H5=Ta54'*(M54*sigemaa54-obj.kecit54chacheng'*M54*obj.kecit4(:,5));
            H6=Ta64'*(M64*sigemaa64-obj.kecit64chacheng'*M44*obj.kecit4(:,6));
            G4=-Ta44'*m44*9.8*10^(3)*[zhongli;cross(g44,zhongli)];
            G5=-Ta54'*m54*9.8*10^(3)*[zhongli;cross(g54,zhongli)];
            G6=-Ta64'*m64*9.8*10^(3)*[zhongli;cross(g64,zhongli)];
            % D矩阵的分块
            D113=D3;
            D4fen=mat2cell(D4,[3,1],[3,1]);
            D114=D4fen{1,1};
            D124=D4fen{1,2};
            D224=D4fen{2,2};
            D5fen=mat2cell(D5,[3,1,1],[3,1,1]);
            D115=D5fen{1,1};
            D125=D5fen{1,2};
            D135=D5fen{1,3};
            D225=D5fen{2,2};
            D235=D5fen{2,3};
            D335=D5fen{3,3};
            D6fen=mat2cell(D6,[3,1,1,1],[3,1,1,1]);
            D116=D6fen{1,1};
            D126=D6fen{1,2};
            D136=D6fen{1,3};
            D146=D6fen{1,4};
            D226=D6fen{2,2};
            D236=D6fen{2,3};
            D246=D6fen{2,4};
            D336=D6fen{3,3};
            D346=D6fen{3,4};
            D446=D6fen{4,4};
            % H矩阵的分块
            H13=H3;
            H14=H4([1 2 3],1);
            H24=H4(4,1);
            H15=H5([1 2 3],1);
            H25=H5(4,1);
            H35=H5(5,1);
            H16=H6([1 2 3],1);
            H26=H6(4,1);
            H36=H6(5,1);
            H46=H6(6,1);
            % G矩阵的分块
            G13=G3;
            G14=G4([1 2 3],1);
            G24=G4(4,1);
            G15=G5([1 2 3],1);
            G25=G5(4,1);
            G35=G5(5,1);
            G16=G6([1 2 3],1);
            G26=G6(4,1);
            G36=G6(5,1);
            G46=G6(6,1);

            % Matrix and vectors generation
            Dq = [D113+D114+D115+D116,D124+D125+D126,D135+D136,D146;...
                 (D124+D125+D126)',D224+D225+D226,D235+D236,D246;...
                 (D135+D136)',(D235+D236)',D335+D336,D346;...
                  D146',D246',D346',D446];
            Hq = [H13+H14+H15+H16;H24+H25+H26;H35+H36;H46];
            Gq = [G13+G14+G15+G16;G24+G25+G26;G35+G36;G46];

            % Driving acceleration loading
            zhudongjiasudu = DrivingAccelerations.*[pi/180,pi/180,pi/180,1,1,1]';
            % Accelerations of six driving joints [6d col,deg/s^2,mm/s^2]
            % the units of the accepted accelerations are [rad/s^2,mm/s^2],
            % which are modified into [deg/s^2,mm/s^2] to match the dynamic
            % model

            % Driving forces calculating
            DrivingForces = Dq*zhudongjiasudu+Hq+Gq;
            % Driving forces of six driving joints [6d col,N*mm,N]
        end
    end
end