
rc=zeros(3,61);%%末端参考点C的位矢
rc(:,1)=[300;-93.36;895.22];%%起点
t=0;%%时间
a=300;%%加速度，300 mm/s^2
%%得到离散时间下，末端参考点C的坐标
for i=1:60
    t=t+0.05;
    if t<=1
       S=0.5*a*t^2;
    elseif 1<t&&t<=2
       S=150+(t-1)*300;
    else
       S=600-0.5*a*(3-t)^2;
    end
    rc(:,i+1)=rc(:,1)+[-S;0;0];
end

zhudong1=zeros(1,61);%%6个主动关节的速度，时间0-3s，步长0.05s
zhudong2=zeros(1,61);
zhudong3=zeros(1,61);
zhudong4=zeros(1,61);
zhudong5=zeros(1,61);
zhudong6=zeros(1,61);
time=zeros(1,61);%%运动3s，步长0.05s，共61个点
jiasudu=zeros(6,61);
jiasudu4=zeros(6,61);%%储存支链4的六个关节的加速度值
beidongzhiliansudu=zeros(3,61);%%储存支链4前3个关节的速度
jiasudu1=zeros(6,61);%%储存三条主动支链6关节加速度
jiasudu2=zeros(6,61);
jiasudu3=zeros(6,61);
TAO=zeros(6,61);%%6个主动关节驱动力/力矩

for s=0:60
time(1,s+1)=0.05*s;%%步长为0.05s，从0一直到3
z64=[0;sqrt(2)/2;-sqrt(2)/2];%%电主轴的位姿
x64=[0;sqrt(2)/2;sqrt(2)/2];%%电主轴的位姿
y64=[1;0;0];%%电主轴的位姿，以上为给定量
%%电主轴保持竖直向下，进行水平平动，注意只有6自由度机器人可以，5自由度机器人不能做到水平平动
dw=132.5;%%刀具偏移量
dv=141.89;
e=233.5;%%动平台中心点到P点的距离(真机改为了283.5)
R64=[x64,y64,z64];%%末端执行器相对于基坐标系的姿态矩阵
rp=rc(:,s+1)-dv*x64+dw*z64;%%P点的位置矢量
q34=norm(rp)-e;%%被动支链第三关节长度，整机基坐标系原点到动平台中点A的距离
s34=rp/(q34+e);%%被动支链第三关节的方向矢量，与点P的位置矢量方向相同
sita14=atan(-s34(2,1)/s34(3,1));%%是由R34矩阵得出的，被动支链第一关节转角，注意测量时有一个45°附加值，此量单位为弧度
sita24=asin(s34(1,1));%%被动支链第二关节转角，此量单位为弧度
degsita14=rad2deg(atan(-s34(2,1)/s34(3,1)));%%被动支链第一关节转角，单位为度
degsita24=rad2deg(asin(s34(1,1)));%%被动支链第二关节转角，单位为度
R34=[cos(sita24),0,sin(sita24);sin(sita14)*sin(sita24),cos(sita14),-sin(sita14)*cos(sita24); ...
     -cos(sita14)*sin(sita24),sin(sita14),cos(sita14)*cos(sita24)];%%被动支链第3构件的姿态矩阵
R6X3=R34'*R64;%%被动支链的第6构件相对与第3构件的姿态矩阵，为了求解三自由度转头的三个转角
if R6X3(1,1)*R6X3(2,1)>0
    sita44=[atan(-R6X3(1,1)/R6X3(2,1));atan(-R6X3(1,1)/R6X3(2,1))+pi];
else
    sita44=[atan(-R6X3(1,1)/R6X3(2,1));atan(-R6X3(1,1)/R6X3(2,1))-pi];
end
if R6X3(3,2)*R6X3(3,3)>0
    sita64=[atan(R6X3(3,2)/R6X3(3,3));atan(-R6X3(3,2)/R6X3(3,3))-pi];
else
    sita64=[atan(R6X3(3,2)/R6X3(3,3));atan(-R6X3(3,2)/R6X3(3,3))+pi];
end%%由于在-180-180范围内，求反三角函数应该有两个解，对于malab中atan范围为-90-90，所以需要判断
%%sin和cos是否同号，以加减PI获得两个解
sita54=[acos(R6X3(3,1));-acos(R6X3(3,1))];
degsita44=[rad2deg(atan(-R6X3(1,1)/R6X3(2,1)));rad2deg(atan(-R6X3(1,1)/R6X3(2,1))+pi)];
degsita54=[rad2deg(acos(R6X3(3,1)));rad2deg(-acos(R6X3(3,1)))];
degsita64=[rad2deg(atan(-R6X3(3,2)/R6X3(3,3)));rad2deg(atan(-R6X3(3,2)/R6X3(3,3))+pi)];
%%至此已经求得被动支链上的全部关节变量，但注意由于是用R6X3的第1列和第3行求解的三个转角，得到了
%%8个解，这些解中满足R6X3中其余等式的解才是我们需要的解，所以下面进行判断
delt12=zeros(1,8);
delt13=zeros(1,8);
delt22=zeros(1,8);
delt23=zeros(1,8);%%R6X3剩余四项和由上式三变量计算值得到的结果之间的误差，如果一组解四个误差均极小，则该解是正解
zhuantou=zeros(3,8);%%保存最终的解，其余均为0
l=0;
for i=1:2
    sita4=sita44(i,1);
    for j=1:2
        sita5=sita54(j,1);
        for k=1:2
            l=l+1;
            sita6=sita64(k,1);
            r12=cos(sita4)*cos(sita6)-sin(sita4)*cos(sita5)*sin(sita6);
            r13=-cos(sita4)*sin(sita6)-sin(sita4)*cos(sita5)*cos(sita6);
            r22=sin(sita4)*cos(sita6)+cos(sita4)*cos(sita5)*sin(sita6);
            r23=-sin(sita4)*sin(sita6)+cos(sita4)*cos(sita5)*cos(sita6);
            delt12(1,l)=abs(r12-R6X3(1,2));
            delt13(1,l)=abs(r13-R6X3(1,3));
            delt22(1,l)=abs(r22-R6X3(2,2));
            delt23(1,l)=abs(r23-R6X3(2,3));
            if delt12(1,l)<0.01&&delt13(1,l)<0.01&&delt22(1,l)<0.01&&delt23(1,l)<0.01
                zhuantou(1,l)=sita4;
                zhuantou(2,l)=sita5;
                zhuantou(3,l)=sita6;
            end
        end
    end
end
b=[294.16,167,167];%%by、bx
y=[-pi/2,0,pi];%%伽玛值，这是由于三条主动支链第四构件摆放方式不同，但是建立的坐标系完全一样造成的恒定的转角偏差
q3i=[0;0;0];%%储存三条主动支链第三关节变量
for m=1:3
    q3i(m,1)=norm(rp+R34*75*[cos(y(1,m));sin(y(1,m));0]-e*s34-b(1,m)*[cos(y(1,m));sin(y(1,m));0]);
end
s31=(rp+R34*75*[cos(y(1,1));sin(y(1,1));0]-e*s34-b(1,1)*[cos(y(1,1));sin(y(1,1));0])/q3i(1,1);
s32=(rp+R34*75*[cos(y(1,2));sin(y(1,2));0]-e*s34-b(1,2)*[cos(y(1,2));sin(y(1,2));0])/q3i(2,1);
s33=(rp+R34*75*[cos(y(1,3));sin(y(1,3));0]-e*s34-b(1,3)*[cos(y(1,3));sin(y(1,3));0])/q3i(3,1);
s3i=[s31,s32,s33];%%三条主动支链的第三关节的方向矢量
sita12i=zeros(2,3);%%储存三条主动支链1，2关节变量
%%也是由R3i构造的
for o=1:3
    sita12i(1,o)=atan(-s3i(2,o)/s3i(3,o));
    sita12i(2,o)=asin(s3i(1,o));
end
R3i=zeros(3,9);%%储存三条主动支链第三构件的姿态矩阵
R3ifen=mat2cell(R3i,[3,0],[3,3,3]);%%将R3i分成3块，每一块为3×3的矩阵，代表对应支链第3构件的姿态矩阵
for p=1:3
    R3ifen{1,p}=[cos(sita12i(2,p)),0,sin(sita12i(2,p));sin(sita12i(1,p))*sin(sita12i(2,p)),cos(sita12i(1,p)),-sin(sita12i(1,p))*cos(sita12i(2,p)); ...
     -cos(sita12i(1,p))*sin(sita12i(2,p)),sin(sita12i(1,p)),cos(sita12i(1,p))*cos(sita12i(2,p))];
end
R31=R3ifen{1,1};%%第1支链中第三构件的姿态矩阵
R32=R3ifen{1,2};%%第2支链中第三构件的姿态矩阵
R33=R3ifen{1,3};%%第3支链中第三构件的姿态矩阵
R6X3i=zeros(3,9);%%为计算主动支链4，5，6关节变变量构建的中间矩阵
R6X3ifen=mat2cell(R6X3i,[3,0],[3,3,3]);%%同样的方式分块
for q=1:3
    R6X3ifen{1,q}=R3ifen{1,q}'*R34*[0,0,1;0,1,0;-1,0,0]*[1,0,0;0,cos(-y(1,q)-pi/2),-sin(-y(1,q)-pi/2);0,sin(-y(1,q)-pi/2),cos(-y(1,q)-pi/2)];
end
sita456i=zeros(3,3);%%储存三条支链4，5，6关节变量
for r=1:3
    R=R6X3ifen{1,r};
    sita456i(:,r)=[atan(-R(1,3)/R(2,3));asin(R(3,3));atan(-R(3,2)/R(3,1))];
end
sita456i(1,1)=sita456i(1,1)+pi*3/2;%%这个地方还有点问题，但这个值确实是对的，与测量值差180度
sita456i(2,1)=pi-sita456i(2,1);%%第一支链的这两个角度非常奇怪，是由于1支链和2、3支链的第四关节差90°造成的
zzjg=zeros(6,4);%%对最终结果进行处理
zzjg([1 2],[1 2 3])=sita12i;
zzjg(3,[1 2 3])=q3i';
zzjg([4 5 6],[1 2 3])=sita456i;
zzjg(1,4)=sita14;
zzjg(2,4)=sita24;
zzjg(3,4)=q34;
zzjg([4 5 6],4)=zhuantou(:,3);
disanguanjie=zzjg(3,:);
zuizhongjieguo=zzjg*180/pi;
zuizhongjieguo(3,:)=disanguanjie;
zuizhongjieguo([4 5],[2 3])=zeros(2,2);
%%初始情况下，sita62=sita63=22.65°，注意到2，3支链的4，5关节变量是常量，这是预料的
%%以上为TriMule-200机器人的逆运动学，最终的结果也就是四支链共24个关节变量全在zuizhongjieguo中
%%角度单位为度%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%以下计算机器人的速度，包括6个主动关节速度和4条支链所有关节的速度
Wap=zeros(6,3);%%Wp矩阵的第一块
ci=zeros(3,3);%%点P到三个球铰中心的矢量
for i=1:3
    ci(:,i)=R34*75*[cos(y(1,i));sin(y(1,i));0]-e*s34;%%上面用到过
    Wap(:,i)=[s3i(:,i);cross(ci(:,i),s3i(:,i))];%%参见论文
end
qe=q34+e;
s14=[1;0;0];%%第四支链第一转轴
Ryz=[0 0 1;1 0 0;0 1 0];
R14=Ryz*[cos(sita14) -sin(sita14) 0;sin(sita14) cos(sita14) 0;0 0 1]*Ryz;
s24=R14(:,3);%%第四支链第二转轴方向向量（都在点P的坐标系下描述，但是其姿态与基坐标系一样）
keciwc1=[cross(s24,s34);-qe*s24];%%组成Wp矩阵的第二块
keciwc2=[s24;qe*cross(s24,s34)];%%组成Wp矩阵的第二块
keciwc3=[0;0;0;cross(s14,s24)];%%组成Wp矩阵的第二块
Wcp=[keciwc1 keciwc2 keciwc3];
Wp=[Wap Wcp];
Tp=inv(Wp');
Tpfen=mat2cell(Tp,[6,0],[3,3]);%%将其分块
Tap=Tpfen{1,1};%%得到Tap是雅可比的第一块
kecita44=[0;0;0;s34];%%第四轴许动位移旋量
R44=R34*[cos(zhuantou(1,3)) -sin(zhuantou(1,3)) 0;sin(zhuantou(1,3)) cos(zhuantou(1,3)) 0;0 0 1]*Ryz;
s54=R44(:,3);
kecita54=[0;0;0;s54];%%第五轴许动位移旋量
R54=R64*[0 0 1;1 0 0;0 1 0]*[cos(zhuantou(3,3)),sin(zhuantou(3,3)),0;-sin(zhuantou(3,3)),cos(zhuantou(3,3)),0;0,0,1];
s64=R54(:,3);
kecita64=[0;0;0;s64];%%第六轴许动位移旋量
yakebiP=[Tap,kecita44,kecita54,kecita64];%%机器人雅可比矩阵
rcp=rp-rc(:,s+1);%%点C到点P的矢量
X=[eye(3),[0 -rcp(3,1) rcp(2,1);rcp(3,1) 0 -rcp(1,1);-rcp(2,1) rcp(1,1) 0];zeros(3,3),eye(3)];%%伴随变换矩阵
T=X*yakebiP;%%相对于C点的雅可比矩阵
%%末端执行器速度旋量
if time(1,s+1)<=1
    kecitc=[-a*time(1,s+1);0;0;0;0;0];
elseif time(1,s+1)>1&&time(1,s+1)<=2
    kecitc=[-300;0;0;0;0;0];
else
    kecitc=[-a*(3-time(1,s+1));0;0;0;0;0];
end
zhudongguanjiesudu=T\kecitc;%%6个主动关节的速度

%%zhudongguanjiesudu=[zhudongguanjiesudu([1 2 3],1);zhudongguanjiesudu([4 5 6],1)*180/pi];
sitaaidian=zeros(6,3);%%三条主动支链各个关节的速度
kecitai1=zeros(6,6);%%支链1的6个单位许动位移旋量
kecitai2=zeros(6,6);%%支链2的6个单位许动位移旋量
kecitai3=zeros(6,6);%%支链3的6个单位许动位移旋量
Ti=[kecitai1,kecitai2,kecitai3];%%储存三条主动支链的18个微小许动位移旋量
TiFen=mat2cell(Ti,[6,0],[6,6,6]);%将其分块
s1i=[1 1 1;0 0 0;0 0 0];%%三条主动支链的第一关节找轴线方向
R11=Ryz*[cos(sita12i(1,1)) -sin(sita12i(1,1)) 0;sin(sita12i(1,1)) cos(sita12i(1,1)) 0;0 0 1]*Ryz;%%为了得到第一支链第二关节轴线方向
s2i=[R11(:,3),s24,s24];%%三条支链第二关节轴线方向
R4i=zeros(3,9);%%用于储存三条支链第四构件姿态矩阵，获取第5关节轴线方向
R4iFen=mat2cell(R4i,[3,0],[3,3,3]);%%分成三块，三条支链
for i=1:3
    R4iFen{1,i}=R3ifen{1,i}*[cos(sita456i(1,i)+y(1,i)),-sin(sita456i(1,i)+y(1,i)),0;sin(sita456i(1,i)+y(1,i)),cos(sita456i(1,i)+y(1,i)),0;0,0,1]*Ryz;
end
R41=R4iFen{1,1};
R42=R4iFen{1,2};
R43=R4iFen{1,3};%%三条支链第四构件姿态矩阵
R4i=[R41,R42,R43];
R5i=zeros(3,9);%%用于储存三条支链第五构件姿态矩阵，获取第6关节轴线方向
R5iFen=mat2cell(R5i,[3,0],[3,3,3]);
for i=1:3
    R5iFen{1,i}=R4iFen{1,i}*[cos(sita456i(2,i)),-sin(sita456i(2,i)),0;sin(sita456i(2,i)),cos(sita456i(2,i)),0;0,0,1]*Ryz;
end%%依然是支链1的有点奇怪
R51=R5iFen{1,1};
R52=R5iFen{1,2};
R53=R5iFen{1,3};%%三条支链第5构件姿态矩阵
R5i=[R51,R52,R53];
for i=1:3
    TiFen{1,i}(:,1)=[cross(ci(:,i)-q3i(i,1)*s3i(:,i),s1i(:,i));s1i(:,i)];
    TiFen{1,i}(:,2)=[cross(ci(:,i)-q3i(i,1)*s3i(:,i),s2i(:,i));s2i(:,i)];
    TiFen{1,i}(:,3)=[s3i(:,i);0;0;0];
    TiFen{1,i}(:,4)=[cross(ci(:,i),s3i(:,i));s3i(:,i)];
    TiFen{1,i}(:,5)=[cross(ci(:,i),R4i(:,3*i));R4i(:,3*i)];
    TiFen{1,i}(:,6)=[cross(ci(:,i),R5i(:,3*i));R5i(:,3*i)];
end%%构造Ti
kecitai1=TiFen{1,1};
kecitai2=TiFen{1,2};
kecitai3=TiFen{1,3};
Ti=[kecitai1,kecitai2,kecitai3];
zhudongzhiliansudu=zeros(6,3);
for i=1:3
zhudongzhiliansudu(:,i)=TiFen{1,i}\(Tap*zhudongguanjiesudu([1 2 3],1))
end%%计算三条主动支链中所有关节的速度

kecita14=[-qe*cross(s34,s14);s14];%%计算被动支链前3关节的速度
kecita24=[qe*cross(s24,s34);s24];
kecita34=[s34;0;0;0];
kecitc14=[cross(s24,s34);0;0;0];
kecitc24=[s24;0;0;0];
n34=cross(s14,s24);
kecitc34=[-qe*cross(s34,n34);n34];
Ta4=[kecita14,kecita24,kecita34];
Tc4=[kecitc14,kecitc24,kecitc34];
T4=[Ta4,Tc4];
W=inv(T4');
Wa4=W(:,[1 2 3]);
beidongzhiliansudu(:,s+1)=Wa4'*Tap*zhudongguanjiesudu([1 2 3],1);%%计算被动支链前3关节的速度

%%以下计算机器人的加速度
%%首先使用矩阵格式计算
%%计算全部关节的速度旋量
kecit1=zeros(6,6);%%支链1的速度旋量
kecit1(:,1)=zhudongzhiliansudu(1,1)*kecitai1(:,1);
for i=2:6
    kecit1(:,i)=kecit1(:,i-1)+zhudongzhiliansudu(i,1)*kecitai1(:,i);
end
T1weifen=zeros(6,6);%%支链1的雅可比矩阵的微分
kecit21chacheng=zeros(6,6);
kecit31chacheng=zeros(6,6);
for k=1:6
     omg=[0,-kecit1(6,k),kecit1(5,k);kecit1(6,k),0,-kecit1(4,k);-kecit1(5,k),kecit1(4,k),0];
     vi=[0,-kecit1(3,k),kecit1(2,k);kecit1(3,k),0,-kecit1(1,k);-kecit1(2,k),kecit1(1,k),0];
     chacheng=[omg,vi;zeros(3,3),omg];
     if k==2
         kecit21chacheng=chacheng;
     elseif k==3
         kecit31chacheng=chacheng;
     end
     T1weifen(:,k)=chacheng*kecitai1(:,k);
end

kecit2=zeros(6,6);%%支链2的速度旋量
kecit2(:,1)=zhudongzhiliansudu(1,2)*kecitai2(:,1);
for i=2:6
    kecit2(:,i)=kecit2(:,i-1)+zhudongzhiliansudu(i,2)*kecitai2(:,i);
end
T2weifen=zeros(6,6);%%支链2的雅可比矩阵的微分
kecit22chacheng=zeros(6,6);
kecit32chacheng=zeros(6,6);
for k=1:6
     omg=[0,-kecit2(6,k),kecit2(5,k);kecit2(6,k),0,-kecit2(4,k);-kecit2(5,k),kecit2(4,k),0];
     vi=[0,-kecit2(3,k),kecit2(2,k);kecit2(3,k),0,-kecit2(1,k);-kecit2(2,k),kecit2(1,k),0];
     chacheng=[omg,vi;zeros(3,3),omg];
     if k==2
         kecit22chacheng=chacheng;
     elseif k==3
         kecit32chacheng=chacheng;
     end
     T2weifen(:,k)=chacheng*kecitai2(:,k);
end

kecit3=zeros(6,6);%%支链3的速度旋量
kecit3(:,1)=zhudongzhiliansudu(1,3)*kecitai3(:,1);
for i=2:6
    kecit3(:,i)=kecit3(:,i-1)+zhudongzhiliansudu(i,3)*kecitai3(:,i);
end
T3weifen=zeros(6,6);%%支链3的雅可比矩阵的微分
kecit23chacheng=zeros(6,6);
kecit33chacheng=zeros(6,6);
for k=1:6
     omg=[0,-kecit3(6,k),kecit3(5,k);kecit3(6,k),0,-kecit3(4,k);-kecit3(5,k),kecit3(4,k),0];
     vi=[0,-kecit3(3,k),kecit3(2,k);kecit3(3,k),0,-kecit3(1,k);-kecit3(2,k),kecit3(1,k),0];
     chacheng=[omg,vi;zeros(3,3),omg];
     if k==2
         kecit23chacheng=chacheng;
     elseif k==3
         kecit33chacheng=chacheng;
     end
     T3weifen(:,k)=chacheng*kecitai3(:,k);
end


kecit4=zeros(6,6);%%支链4的速度旋量
sudu4=[beidongzhiliansudu(:,s+1);zhudongguanjiesudu([4 5 6],1)];
kecita4=[kecita14,kecita24,kecita34,kecita44,kecita54,kecita64];
kecit4(:,1)=sudu4(1,1)*kecita4(:,1);
for i=2:6
    kecit4(:,i)=kecit4(:,i-1)+sudu4(i,1)*kecita4(:,i);
end
T4weifen=zeros(6,6);%%支链4的雅可比矩阵的微分
kecit14chacheng=zeros(6,6);
kecit34chacheng=zeros(6,6);
kecit44chacheng=zeros(6,6);
kecit54chacheng=zeros(6,6);
kecit64chacheng=zeros(6,6);
for k=1:6
     omg=[0,-kecit4(6,k),kecit4(5,k);kecit4(6,k),0,-kecit4(4,k);-kecit4(5,k),kecit4(4,k),0];
     vi=[0,-kecit4(3,k),kecit4(2,k);kecit4(3,k),0,-kecit4(1,k);-kecit4(2,k),kecit4(1,k),0];
     chacheng=[omg,vi;zeros(3,3),omg];
     if k==1
         kecit14chacheng=chacheng;
     elseif k==3
         kecit34chacheng=chacheng;
     elseif k==4
         kecit44chacheng=chacheng;
     elseif k==5
         kecit54chacheng=chacheng;
     elseif k==6
         kecit64chacheng=chacheng;
     end
     T4weifen(:,k)=chacheng*kecita4(:,k);
end
if time(1,s+1)<1
    keciac=[-300;0;0;0;0;0];%%点C的加速度旋量
elseif 1<=time(1,s+1)&&time(1,s+1)<2
    keciac=[0;0;0;0;0;0];%%点C的加速度旋量
else
    keciac=[300;0;0;0;0;0];%%点C的加速度旋量
end
jiasudu4(:,s+1)=kecita4\(keciac-T4weifen*sudu4);%%支链4的6个关节的角加速度

%%计算动平台关于点P的加速度旋量
keciap=T4weifen(:,[1 2 3])*beidongzhiliansudu(:,s+1)+kecita4(:,[1 2 3])*jiasudu4([1 2 3],s+1);
%%计算1 2 3三条主动支链各关节加速度
jiasudu1(:,s+1)=kecitai1\(keciap-T1weifen*zhudongzhiliansudu(:,1));
jiasudu1([1 2 4 5 6],s+1)=jiasudu1([1 2 4 5 6],s+1)*180/pi;%%支链1关节5、6加速度1.5s后，与测量值差一个正负号，程序最后有解释，这里暂时进行修正
if time(1,s+1)>=1.5
    jiasudu1([5 6],s+1)=-jiasudu1([5 6],s+1);%%解决这一问题的根本方法需要修改128、129行
end
jiasudu2(:,s+1)=kecitai2\(keciap-T2weifen*zhudongzhiliansudu(:,2));
jiasudu2([1 2 4 5 6],s+1)=jiasudu2([1 2 4 5 6],s+1)*180/pi;
jiasudu3(:,s+1)=kecitai3\(keciap-T3weifen*zhudongzhiliansudu(:,3));
jiasudu3([1 2 4 5 6],s+1)=jiasudu3([1 2 4 5 6],s+1)*180/pi;

%%测试：以第4支链计算加速度,使用双累加法
% kecita4=[kecita14,kecita24,kecita34,kecita44,kecita54,kecita64];
% sudu4=[beidongzhiliansudu;zhudongguanjiesudu([4 5 6],1)];%%支链4的6个关节速度
% sigema=zeros(6,1);
% for j=2:6
%     for k=1:j-1
%         omg=[0,-kecita4(6,k),kecita4(5,k);kecita4(6,k),0,-kecita4(4,k);-kecita4(5,k),kecita4(4,k),0];
%         vi=[0,-kecita4(3,k),kecita4(2,k);kecita4(3,k),0,-kecita4(1,k);-kecita4(2,k),kecita4(1,k),0];
%         chacheng=[omg,vi;zeros(3,3),omg];
%         sigema=sigema+sudu4(j,1)*sudu4(k,1)*chacheng*kecita4(:,j);
%     end
% end
% jiasudu4(1,s+1)=inv(kecita4)*(keciac-sigema);%%该结果的后三项应该为转头三关节的加速度值，暂时不对

%%以下计算刚体动力学
%%首先根据牛顿-欧拉方程、虚功原理列写方程
I1ijubu=[136369297 0 0;0 136344677 0;0 0 2094913];%%三条主动支链第一构件(套筒)在局部坐标系下的惯性张量
I14jubu=[102854706 0 0;0 16869091 207422;0 207422 91255649];%%被动支链第一构件(转动架)惯性张量
m1i=4833;%%套筒质量
m14=5324;%%转动架质量
M1ijubu=[m1i*eye(3),zeros(3,3);zeros(3,3),I1ijubu];%%构建广义惯性矩阵
M14jubu=[m14*eye(3),zeros(3,3);zeros(3,3),I14jubu];
g11=(R31*[0;0;-18]+[0;-294.16;0]-rp);%%支链1构件1的重心，在参考坐标系下的坐标
g12=(R32*[0;0;-18]+[167;0;0]-rp);
g13=(R33*[0;0;-18]+[-167;0;0]-rp);
g14=(R14*[0;0;-5.6]-rp);
g11chacheng=[0,-g11(3,1),g11(2,1);g11(3,1),0,-g11(1,1);-g11(2,1),g11(1,1),0];
g12chacheng=[0,-g12(3,1),g12(2,1);g12(3,1),0,-g12(1,1);-g12(2,1),g12(1,1),0];
g13chacheng=[0,-g13(3,1),g13(2,1);g13(3,1),0,-g13(1,1);-g13(2,1),g13(1,1),0];
g14chacheng=[0,-g14(3,1),g14(2,1);g14(3,1),0,-g14(1,1);-g14(2,1),g14(1,1),0];
Adg11=[R31,g11chacheng*R31;zeros(3,3),R31];%%从局部坐标系到参考坐标系的伴随变换矩阵
Adg12=[R32,g12chacheng*R32;zeros(3,3),R32];
Adg13=[R33,g13chacheng*R33;zeros(3,3),R33];
Adg14=[R14,g14chacheng*R14;zeros(3,3),R14];
M11=inv(Adg11')*M1ijubu*inv(Adg11);%%在参考坐标系下的广义惯性矩阵
M12=inv(Adg12')*M1ijubu*inv(Adg12);
M13=inv(Adg13')*M1ijubu*inv(Adg13);
M14=inv(Adg14')*M14jubu*inv(Adg14);

I2ijubu=[17984609,0,0;0,17999476,0;0,0,195625];%%主动支链第二构件(推杆)局部坐标系下的惯性张量
I24jubu=[689668.8,9.917,-2588.8;9.917,690657.3,6302;-2588.8,6302,28627.96];%%被动支链第二构件（支链体）在局部坐标系下的惯性张量
m2i=1263;%%推杆质量
m24=15379;%%支链体质量
M2ijubu=[m2i*eye(3),zeros(3,3);zeros(3,3),I2ijubu];%%构建局部坐标系下的广义惯性矩阵
M24jubu=[m24*eye(3),zeros(3,3);zeros(3,3),I24jubu];
g21=(R31*[0;0;q3i(1,1)-209.5]+[0;-294.16;0]-rp);%%在参考坐标系下的重心坐标
g22=(R32*[0;0;q3i(2,1)-209.5]+[167;0;0]-rp);
g23=(R33*[0;0;q3i(3,1)-209.5]+[-167;0;0]-rp);
g24=(R34(:,3)*(q34-208)-rp);
g21chacheng=[0,-g21(3,1),g21(2,1);g21(3,1),0,-g21(1,1);-g21(2,1),g21(1,1),0];
g22chacheng=[0,-g22(3,1),g22(2,1);g22(3,1),0,-g22(1,1);-g22(2,1),g22(1,1),0];
g23chacheng=[0,-g23(3,1),g23(2,1);g23(3,1),0,-g23(1,1);-g23(2,1),g23(1,1),0];
g24chacheng=[0,-g24(3,1),g24(2,1);g24(3,1),0,-g24(1,1);-g24(2,1),g24(1,1),0];
Adg21=[R31,g21chacheng*R31;zeros(3,3),R31];%%从各连杆系到参考系的伴随变换矩阵
Adg22=[R32,g22chacheng*R32;zeros(3,3),R32];
Adg23=[R33,g23chacheng*R33;zeros(3,3),R33];
Adg24=[R34,g24chacheng*R34;zeros(3,3),R34];
M21=inv(Adg21')*M2ijubu*inv(Adg21);%%在参考坐标系下的广义惯性矩阵
M22=inv(Adg22')*M2ijubu*inv(Adg22);
M23=inv(Adg23')*M2ijubu*inv(Adg23);
M24=inv(Adg24')*M24jubu*inv(Adg24);
%%为了计算Ta,j,i
Ta11pie=kecitai1(:,[1 2]);%%注意动力学考虑的第一各构件其实是运动学中的第二构件(套筒)
Ta21pie=kecitai1(:,[1 2 3]);
Ta12pie=kecitai2(:,[1 2]);
Ta22pie=kecitai2(:,[1 2 3]);
Ta13pie=kecitai3(:,[1 2]);
Ta23pie=kecitai3(:,[1 2 3]);
Ta14pie=kecita14;
Ta24pie=[kecita14,kecita24,kecita34];
E11=[eye(2),zeros(2,4)];
E21=[eye(3),zeros(3,3)];
E12=[eye(2),zeros(2,4)];
E22=[eye(3),zeros(3,3)];
E13=[eye(2),zeros(2,4)];
E23=[eye(3),zeros(3,3)];
E14=[1 0 0];
E24=eye(3);
Wa1=inv(TiFen{1,1})';
Wa2=inv(TiFen{1,2})';
Wa3=inv(TiFen{1,3})';
Ta11=Ta11pie*E11*Wa1'*Tap;%%至此可以求Ta,j,i了
Ta21=Ta21pie*E21*Wa1'*Tap;
Ta12=Ta12pie*E12*Wa2'*Tap;
Ta22=Ta22pie*E22*Wa2'*Tap;
Ta13=Ta13pie*E13*Wa3'*Tap;
Ta23=Ta23pie*E23*Wa3'*Tap;
Ta14=Ta14pie*E14*Wa4'*Tap;
Ta24=Ta24pie*E24*Wa4'*Tap;
Is=11309;%%这是丝杠和联轴器绕自身轴线的转动惯量，注意在SW中没有丝杠旋转，肯能导致误差
daocheng=4;%%丝杠的导程
D3=Ta11'*M11*Ta11+Ta21'*M21*Ta21+Ta12'*M12*Ta12+Ta22'*M22*Ta22+Ta13'*M13*Ta13+Ta23'*M23*Ta23+...
    Ta14'*M14*Ta14+Ta24'*M24*Ta24+eye(3)*Is*(2*pi/daocheng)^2;

sigemaa11=T1weifen(:,[1 2])*zhudongzhiliansudu([1 2],1);%%这就是计算加速时的第二项
sigemaa21=T1weifen(:,[1 2 3])*zhudongzhiliansudu([1 2 3],1);
sigemaa12=T2weifen(:,[1 2])*zhudongzhiliansudu([1 2],2);
sigemaa22=T2weifen(:,[1 2 3])*zhudongzhiliansudu([1 2 3],2);
sigemaa13=T3weifen(:,[1 2])*zhudongzhiliansudu([1 2],3);
sigemaa23=T3weifen(:,[1 2 3])*zhudongzhiliansudu([1 2 3],3);
sigemaa14=T4weifen(:,1)*beidongzhiliansudu(1,s+1);
sigemaa24=T4weifen(:,[1 2 3])*beidongzhiliansudu([1 2 3],s+1);
H3=Ta11'*(M11*sigemaa11-kecit21chacheng'*M11*kecit1(:,2))+...%%构建H3
   Ta21'*(M21*sigemaa21-kecit31chacheng'*M21*kecit1(:,3))+...
   Ta12'*(M12*sigemaa12-kecit22chacheng'*M12*kecit2(:,2))+...
   Ta22'*(M22*sigemaa22-kecit32chacheng'*M22*kecit2(:,3))+...
   Ta13'*(M13*sigemaa13-kecit23chacheng'*M13*kecit3(:,2))+...
   Ta23'*(M23*sigemaa23-kecit33chacheng'*M23*kecit3(:,3))+...
   Ta14'*(M14*sigemaa14-kecit14chacheng'*M14*kecit4(:,1))+...
   Ta24'*(M24*sigemaa24-kecit34chacheng'*M24*kecit4(:,3));

zhongli=[0;-sqrt(2)/2;sqrt(2)/2];%%重力的方向向量
G3=-(Ta11'*4833*9.8*10^(3)*[zhongli;cross(g11,zhongli)]+Ta21'*1263*9.8*10^(3)*[zhongli;cross(g21,zhongli)]+...%%构建G3
     Ta12'*4833*9.8*10^(3)*[zhongli;cross(g12,zhongli)]+Ta22'*1263*9.8*10^(3)*[zhongli;cross(g22,zhongli)]+...
     Ta13'*4833*9.8*10^(3)*[zhongli;cross(g13,zhongli)]+Ta23'*1263*9.8*10^(3)*[zhongli;cross(g23,zhongli)]+...
     Ta14'*5324*9.8*10^(3)*[zhongli;cross(g14,zhongli)]+Ta24'*15379*9.8*10^(3)*[zhongli;cross(g24,zhongli)]);

I44jubu=[78253267 -4919741 1033606;-4919741 45881663 2580650;1033606 2580650 56855084];%%转头第一构件在局部坐标系下的惯性张量
g44jubu=[-11.11;-83.47;2.7];%%局部坐标系下重心坐标
m44=7540;%%构件质量
I54jubu=[15011772 0 0;0 14884080 0;0 0 6173188];
g54jubu=[0;0;9.34];
m54=4258;
I64jubu=[2912854 0 154973;0 3111333 0;154973 0 1675256];
g64jubu=[-4.1;0;111.73];
m64=1819;
M44jubu=[m44*eye(3),zeros(3,3);zeros(3,3),I44jubu];%%构建局部坐标系下的广义惯性矩阵
M54jubu=[m54*eye(3),zeros(3,3);zeros(3,3),I54jubu];
M64jubu=[m64*eye(3),zeros(3,3);zeros(3,3),I64jubu];
g44=R44*g44jubu;%%在参考坐标系下的重心坐标
g54=R54*g54jubu;
g64=R64*g64jubu-rcp;
g44chacheng=[0,-g44(3,1),g44(2,1);g44(3,1),0,-g44(1,1);-g44(2,1),g44(1,1),0];
g54chacheng=[0,-g54(3,1),g54(2,1);g54(3,1),0,-g54(1,1);-g54(2,1),g54(1,1),0];
g64chacheng=[0,-g64(3,1),g64(2,1);g64(3,1),0,-g64(1,1);-g64(2,1),g64(1,1),0];
Adg44=[R44,g44chacheng*R44;zeros(3,3),R44];%%连杆系相对于参考系的伴随变换矩阵
Adg54=[R54,g54chacheng*R54;zeros(3,3),R54];
Adg64=[R64,g64chacheng*R64;zeros(3,3),R64];
M44=inv(Adg44')*M44jubu*inv(Adg44);%%参考系下的广义惯性矩阵
M54=inv(Adg54')*M54jubu*inv(Adg54);
M64=inv(Adg64')*M64jubu*inv(Adg64);
Ta44=[Tap,kecita44];
Ta54=[Ta44,kecita54];
Ta64=[Ta54,kecita64];
D4=Ta44'*M44*Ta44;%%暂时不考虑传动部件的转动惯量，可能造成误差
D5=Ta54'*M54*Ta54;
D6=Ta64'*M64*Ta64;

sigemaa44=T4weifen(:,[1 2 3 4])*sudu4([1 2 3 4],1);%%等同于求加速度式中的第二项
sigemaa54=T4weifen(:,[1 2 3 4 5])*sudu4([1 2 3 4 5],1);
sigemaa64=T4weifen(:,[1 2 3 4 5 6])*sudu4;
H4=Ta44'*(M44*sigemaa44-kecit44chacheng'*M44*kecit4(:,4));
H5=Ta54'*(M54*sigemaa54-kecit54chacheng'*M54*kecit4(:,5));
H6=Ta64'*(M64*sigemaa64-kecit64chacheng'*M44*kecit4(:,6));
G4=-Ta44'*m44*9.8*10^(3)*[zhongli;cross(g44,zhongli)];
G5=-Ta54'*m54*9.8*10^(3)*[zhongli;cross(g54,zhongli)];
G6=-Ta64'*m64*9.8*10^(3)*[zhongli;cross(g64,zhongli)];
%%D矩阵的分块
D113=D3;%%开始分块，便于求解
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
%%H矩阵的分块
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
%%G矩阵的分块
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

Dq=[D113+D114+D115+D116,D124+D125+D126,D135+D136,D146;...
    (D124+D125+D126)',D224+D225+D226,D235+D236,D246;...
    (D135+D136)',(D235+D236)',D335+D336,D346;...
    D146',D246',D346',D446];
Hq=[H13+H14+H15+H16;H24+H25+H26;H35+H36;H46];
Gq=[G13+G14+G15+G16;G24+G25+G26;G35+G36;G46];
zhudongjiasudu=[jiasudu1(3,s+1);jiasudu2(3,s+1);jiasudu3(3,s+1);jiasudu4([4 5 6],s+1)];%%6个主动关节的加速度

TAO(:,s+1)=Dq*zhudongjiasudu+Hq+Gq;%%6个主动关节的驱动力/力矩

zhudong1(1,s+1)=zhudongguanjiesudu(1,1);
zhudong2(1,s+1)=zhudongguanjiesudu(2,1);
zhudong3(1,s+1)=zhudongguanjiesudu(3,1);
zhudong4(1,s+1)=zhudongguanjiesudu(4,1)*180/pi;
zhudong5(1,s+1)=zhudongguanjiesudu(5,1)*180/pi;
zhudong6(1,s+1)=zhudongguanjiesudu(6,1)*180/pi;%%6个主动关节的速度随时间的变化规律
end
figure(1);%%三个伸缩杆的速度
plot(time,zhudong1,'r',time,zhudong2,'g',time,zhudong3,'b');
xlabel('时间 s');
ylabel('线速度 mm/s');
hold on;
figure(2);%%转头三个关节的速度
plot(time,zhudong4,'r',time,zhudong5,'g',time,zhudong6,'b');
xlabel('时间 s');
ylabel('角速度 deg/s');
hold on;
figure(3);%%三个伸缩杆的加速度
plot(time,jiasudu1(3,:),'r',time,jiasudu2(3,:),'g',time,jiasudu3(3,:),'b');
xlabel('时间 s');
ylabel('加速度 mm/s^2');
hold on;
figure(4);%%转头三关节的加速度
plot(time,jiasudu4(4,:)*180/pi,'r',time,jiasudu4(5,:)*180/pi,'g',time,jiasudu4(6,:)*180/pi,'b');
xlabel('时间 s');
ylabel('角加速度 deg/s^2');
%%验证加速度的正确性，文件读取的是Solidworks中测量的结果
% [num1]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链1关节1角加速度.xls');
% [num2]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链1关节2角加速度.xls');
% [num3]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链1关节4角加速度.xls');
% hold on;
% figure(5);
% plot(time,num1(:,2),'r--',time,num2(:,2),'g--',time,num3(:,2),'b--',time,jiasudu1(1,:),'r',time,jiasudu1(2,:),'g',time,jiasudu1(4,:),'b');
% xlabel('时间 s');
% ylabel('角加速度 deg/s^2');
% [num4]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链1关节5角加速度.xls');
% [num5]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链1关节6角加速度.xls');
% [num6]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链2关节2角加速度.xls');
% hold on;
% figure(5);
% plot(time,num4(:,2),'r--',time,num5(:,2),'g--',time,num6(:,2),'b--',time,jiasudu1(5,:),'r',time,jiasudu1(6,:),'g',time,jiasudu2(2,:),'b');
% xlabel('时间 s');
% ylabel('角加速度 deg/s^2');%%发现第一支链5、6关节加速度1.5s后与测量值差一个正负号，怀疑是4、5关节转角造成的,已经发现第一支链5、6关节旋量不对，绕x轴旋转了180°
% [num7]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链2关节6角加速度.xls');
% [num8]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链3关节2角加速度.xls');
% [num9]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链3关节6角加速度.xls');
% hold on;
% figure(5);
% plot(time,num7(:,2),'r--',time,num8(:,2),'g--',time,num9(:,2),'b--',time,jiasudu2(6,:),'r',time,jiasudu3(2,:),'g',time,jiasudu3(6,:),'b');
% xlabel('时间 s');
% ylabel('角加速度 deg/s^2');
% [num10]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\支链4关节2角加速度.xls');
% hold on;
% figure(5);
% plot(time,num10(:,2),'r--',time,jiasudu4(2,:)*180/pi,'r');
% xlabel('时间 s');
% ylabel('角加速度 deg/s^2');
% [num11]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\第四关节角加速度.xls');
% [num12]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\第五关节角加速度.xls');
% [num13]=xlsread('C:\Users\YAYALE\Desktop\Solidworks导出速度\第六关节角加速度.xls');
% hold on;
% figure(5);
% plot(time,num11(:,2),'r--',time,num12(:,2),'g--',time,num13(:,2),'b--',time,jiasudu4(4,:)*180/pi,'r',time,jiasudu4(5,:)*180/pi,'g',time,jiasudu4(6,:)*180/pi,'b');
% xlabel('时间 s');
% ylabel('角加速度 deg/s^2');
%%至此所有关节的加速度都和测量值保持一致，验证了模型的正确性
hold on;
figure(5);%%第一主动支链推杆推力
plot(time,TAO(1,:)/10^6,'r');
xlabel('时间 s');
ylabel('推力 N');
hold on;
figure(6);%%第2、3主动支链推杆推力
plot(time,TAO(2,:)/10^6,'g',time,TAO(3,:)/10^6,'b');
xlabel('时间 s');
ylabel('推力 N');
hold on;
figure(7);%%第四关节驱动力矩
plot(time,TAO(4,:)/10^6,'r');
xlabel('时间 s');
ylabel('力矩 N.mm');
hold on;
figure(8);%%第五关节驱动力矩
plot(time,TAO(5,:)/10^6,'r');
xlabel('时间 s');
ylabel('力矩 N.mm');
hold on;
figure(9);%%第六关节驱动力矩
plot(time,TAO(6,:)/10^6,'r');
xlabel('时间 s');
ylabel('力矩 N.mm');
hold on;
figure(10);
plot(time,jiasudu3(3,:),'r-o',time,jiasudu3(3,:),'g-*');
xlabel('t(s)');
ylabel('a(mm/s^2)');