
rc=zeros(3,61);%%ĩ�˲ο���C��λʸ
rc(:,1)=[300;-93.36;895.22];%%���
t=0;%%ʱ��
a=300;%%���ٶȣ�300 mm/s^2
%%�õ���ɢʱ���£�ĩ�˲ο���C������
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

zhudong1=zeros(1,61);%%6�������ؽڵ��ٶȣ�ʱ��0-3s������0.05s
zhudong2=zeros(1,61);
zhudong3=zeros(1,61);
zhudong4=zeros(1,61);
zhudong5=zeros(1,61);
zhudong6=zeros(1,61);
time=zeros(1,61);%%�˶�3s������0.05s����61����
jiasudu=zeros(6,61);
jiasudu4=zeros(6,61);%%����֧��4�������ؽڵļ��ٶ�ֵ
beidongzhiliansudu=zeros(3,61);%%����֧��4ǰ3���ؽڵ��ٶ�
jiasudu1=zeros(6,61);%%������������֧��6�ؽڼ��ٶ�
jiasudu2=zeros(6,61);
jiasudu3=zeros(6,61);
TAO=zeros(6,61);%%6�������ؽ�������/����

for s=0:60
time(1,s+1)=0.05*s;%%����Ϊ0.05s����0һֱ��3
z64=[0;sqrt(2)/2;-sqrt(2)/2];%%�������λ��
x64=[0;sqrt(2)/2;sqrt(2)/2];%%�������λ��
y64=[1;0;0];%%�������λ�ˣ�����Ϊ������
%%�����ᱣ����ֱ���£�����ˮƽƽ����ע��ֻ��6���ɶȻ����˿��ԣ�5���ɶȻ����˲�������ˮƽƽ��
dw=132.5;%%����ƫ����
dv=141.89;
e=233.5;%%��ƽ̨���ĵ㵽P��ľ���(�����Ϊ��283.5)
R64=[x64,y64,z64];%%ĩ��ִ��������ڻ�����ϵ����̬����
rp=rc(:,s+1)-dv*x64+dw*z64;%%P���λ��ʸ��
q34=norm(rp)-e;%%����֧�������ؽڳ��ȣ�����������ϵԭ�㵽��ƽ̨�е�A�ľ���
s34=rp/(q34+e);%%����֧�������ؽڵķ���ʸ�������P��λ��ʸ��������ͬ
sita14=atan(-s34(2,1)/s34(3,1));%%����R34����ó��ģ�����֧����һ�ؽ�ת�ǣ�ע�����ʱ��һ��45�㸽��ֵ��������λΪ����
sita24=asin(s34(1,1));%%����֧���ڶ��ؽ�ת�ǣ�������λΪ����
degsita14=rad2deg(atan(-s34(2,1)/s34(3,1)));%%����֧����һ�ؽ�ת�ǣ���λΪ��
degsita24=rad2deg(asin(s34(1,1)));%%����֧���ڶ��ؽ�ת�ǣ���λΪ��
R34=[cos(sita24),0,sin(sita24);sin(sita14)*sin(sita24),cos(sita14),-sin(sita14)*cos(sita24); ...
     -cos(sita14)*sin(sita24),sin(sita14),cos(sita14)*cos(sita24)];%%����֧����3��������̬����
R6X3=R34'*R64;%%����֧���ĵ�6����������3��������̬����Ϊ����������ɶ�תͷ������ת��
if R6X3(1,1)*R6X3(2,1)>0
    sita44=[atan(-R6X3(1,1)/R6X3(2,1));atan(-R6X3(1,1)/R6X3(2,1))+pi];
else
    sita44=[atan(-R6X3(1,1)/R6X3(2,1));atan(-R6X3(1,1)/R6X3(2,1))-pi];
end
if R6X3(3,2)*R6X3(3,3)>0
    sita64=[atan(R6X3(3,2)/R6X3(3,3));atan(-R6X3(3,2)/R6X3(3,3))-pi];
else
    sita64=[atan(R6X3(3,2)/R6X3(3,3));atan(-R6X3(3,2)/R6X3(3,3))+pi];
end%%������-180-180��Χ�ڣ������Ǻ���Ӧ���������⣬����malab��atan��ΧΪ-90-90��������Ҫ�ж�
%%sin��cos�Ƿ�ͬ�ţ��ԼӼ�PI���������
sita54=[acos(R6X3(3,1));-acos(R6X3(3,1))];
degsita44=[rad2deg(atan(-R6X3(1,1)/R6X3(2,1)));rad2deg(atan(-R6X3(1,1)/R6X3(2,1))+pi)];
degsita54=[rad2deg(acos(R6X3(3,1)));rad2deg(-acos(R6X3(3,1)))];
degsita64=[rad2deg(atan(-R6X3(3,2)/R6X3(3,3)));rad2deg(atan(-R6X3(3,2)/R6X3(3,3))+pi)];
%%�����Ѿ���ñ���֧���ϵ�ȫ���ؽڱ�������ע����������R6X3�ĵ�1�к͵�3����������ת�ǣ��õ���
%%8���⣬��Щ��������R6X3�������ʽ�Ľ����������Ҫ�Ľ⣬������������ж�
delt12=zeros(1,8);
delt13=zeros(1,8);
delt22=zeros(1,8);
delt23=zeros(1,8);%%R6X3ʣ�����������ʽ����������ֵ�õ��Ľ��֮��������һ����ĸ�������С����ý�������
zhuantou=zeros(3,8);%%�������յĽ⣬�����Ϊ0
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
b=[294.16,167,167];%%by��bx
y=[-pi/2,0,pi];%%٤��ֵ������������������֧�����Ĺ����ڷŷ�ʽ��ͬ�����ǽ���������ϵ��ȫһ����ɵĺ㶨��ת��ƫ��
q3i=[0;0;0];%%������������֧�������ؽڱ���
for m=1:3
    q3i(m,1)=norm(rp+R34*75*[cos(y(1,m));sin(y(1,m));0]-e*s34-b(1,m)*[cos(y(1,m));sin(y(1,m));0]);
end
s31=(rp+R34*75*[cos(y(1,1));sin(y(1,1));0]-e*s34-b(1,1)*[cos(y(1,1));sin(y(1,1));0])/q3i(1,1);
s32=(rp+R34*75*[cos(y(1,2));sin(y(1,2));0]-e*s34-b(1,2)*[cos(y(1,2));sin(y(1,2));0])/q3i(2,1);
s33=(rp+R34*75*[cos(y(1,3));sin(y(1,3));0]-e*s34-b(1,3)*[cos(y(1,3));sin(y(1,3));0])/q3i(3,1);
s3i=[s31,s32,s33];%%��������֧���ĵ����ؽڵķ���ʸ��
sita12i=zeros(2,3);%%������������֧��1��2�ؽڱ���
%%Ҳ����R3i�����
for o=1:3
    sita12i(1,o)=atan(-s3i(2,o)/s3i(3,o));
    sita12i(2,o)=asin(s3i(1,o));
end
R3i=zeros(3,9);%%������������֧��������������̬����
R3ifen=mat2cell(R3i,[3,0],[3,3,3]);%%��R3i�ֳ�3�飬ÿһ��Ϊ3��3�ľ��󣬴����Ӧ֧����3��������̬����
for p=1:3
    R3ifen{1,p}=[cos(sita12i(2,p)),0,sin(sita12i(2,p));sin(sita12i(1,p))*sin(sita12i(2,p)),cos(sita12i(1,p)),-sin(sita12i(1,p))*cos(sita12i(2,p)); ...
     -cos(sita12i(1,p))*sin(sita12i(2,p)),sin(sita12i(1,p)),cos(sita12i(1,p))*cos(sita12i(2,p))];
end
R31=R3ifen{1,1};%%��1֧���е�����������̬����
R32=R3ifen{1,2};%%��2֧���е�����������̬����
R33=R3ifen{1,3};%%��3֧���е�����������̬����
R6X3i=zeros(3,9);%%Ϊ��������֧��4��5��6�ؽڱ�����������м����
R6X3ifen=mat2cell(R6X3i,[3,0],[3,3,3]);%%ͬ���ķ�ʽ�ֿ�
for q=1:3
    R6X3ifen{1,q}=R3ifen{1,q}'*R34*[0,0,1;0,1,0;-1,0,0]*[1,0,0;0,cos(-y(1,q)-pi/2),-sin(-y(1,q)-pi/2);0,sin(-y(1,q)-pi/2),cos(-y(1,q)-pi/2)];
end
sita456i=zeros(3,3);%%��������֧��4��5��6�ؽڱ���
for r=1:3
    R=R6X3ifen{1,r};
    sita456i(:,r)=[atan(-R(1,3)/R(2,3));asin(R(3,3));atan(-R(3,2)/R(3,1))];
end
sita456i(1,1)=sita456i(1,1)+pi*3/2;%%����ط����е����⣬�����ֵȷʵ�ǶԵģ������ֵ��180��
sita456i(2,1)=pi-sita456i(2,1);%%��һ֧�����������Ƕȷǳ���֣�������1֧����2��3֧���ĵ��Ĺؽڲ�90����ɵ�
zzjg=zeros(6,4);%%�����ս�����д���
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
%%��ʼ����£�sita62=sita63=22.65�㣬ע�⵽2��3֧����4��5�ؽڱ����ǳ���������Ԥ�ϵ�
%%����ΪTriMule-200�����˵����˶�ѧ�����յĽ��Ҳ������֧����24���ؽڱ���ȫ��zuizhongjieguo��
%%�Ƕȵ�λΪ��%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%���¼�������˵��ٶȣ�����6�������ؽ��ٶȺ�4��֧�����йؽڵ��ٶ�
Wap=zeros(6,3);%%Wp����ĵ�һ��
ci=zeros(3,3);%%��P������������ĵ�ʸ��
for i=1:3
    ci(:,i)=R34*75*[cos(y(1,i));sin(y(1,i));0]-e*s34;%%�����õ���
    Wap(:,i)=[s3i(:,i);cross(ci(:,i),s3i(:,i))];%%�μ�����
end
qe=q34+e;
s14=[1;0;0];%%����֧����һת��
Ryz=[0 0 1;1 0 0;0 1 0];
R14=Ryz*[cos(sita14) -sin(sita14) 0;sin(sita14) cos(sita14) 0;0 0 1]*Ryz;
s24=R14(:,3);%%����֧���ڶ�ת�᷽�����������ڵ�P������ϵ����������������̬�������ϵһ����
keciwc1=[cross(s24,s34);-qe*s24];%%���Wp����ĵڶ���
keciwc2=[s24;qe*cross(s24,s34)];%%���Wp����ĵڶ���
keciwc3=[0;0;0;cross(s14,s24)];%%���Wp����ĵڶ���
Wcp=[keciwc1 keciwc2 keciwc3];
Wp=[Wap Wcp];
Tp=inv(Wp');
Tpfen=mat2cell(Tp,[6,0],[3,3]);%%����ֿ�
Tap=Tpfen{1,1};%%�õ�Tap���ſɱȵĵ�һ��
kecita44=[0;0;0;s34];%%��������λ������
R44=R34*[cos(zhuantou(1,3)) -sin(zhuantou(1,3)) 0;sin(zhuantou(1,3)) cos(zhuantou(1,3)) 0;0 0 1]*Ryz;
s54=R44(:,3);
kecita54=[0;0;0;s54];%%��������λ������
R54=R64*[0 0 1;1 0 0;0 1 0]*[cos(zhuantou(3,3)),sin(zhuantou(3,3)),0;-sin(zhuantou(3,3)),cos(zhuantou(3,3)),0;0,0,1];
s64=R54(:,3);
kecita64=[0;0;0;s64];%%��������λ������
yakebiP=[Tap,kecita44,kecita54,kecita64];%%�������ſɱȾ���
rcp=rp-rc(:,s+1);%%��C����P��ʸ��
X=[eye(3),[0 -rcp(3,1) rcp(2,1);rcp(3,1) 0 -rcp(1,1);-rcp(2,1) rcp(1,1) 0];zeros(3,3),eye(3)];%%����任����
T=X*yakebiP;%%�����C����ſɱȾ���
%%ĩ��ִ�����ٶ�����
if time(1,s+1)<=1
    kecitc=[-a*time(1,s+1);0;0;0;0;0];
elseif time(1,s+1)>1&&time(1,s+1)<=2
    kecitc=[-300;0;0;0;0;0];
else
    kecitc=[-a*(3-time(1,s+1));0;0;0;0;0];
end
zhudongguanjiesudu=T\kecitc;%%6�������ؽڵ��ٶ�

%%zhudongguanjiesudu=[zhudongguanjiesudu([1 2 3],1);zhudongguanjiesudu([4 5 6],1)*180/pi];
sitaaidian=zeros(6,3);%%��������֧�������ؽڵ��ٶ�
kecitai1=zeros(6,6);%%֧��1��6����λ��λ������
kecitai2=zeros(6,6);%%֧��2��6����λ��λ������
kecitai3=zeros(6,6);%%֧��3��6����λ��λ������
Ti=[kecitai1,kecitai2,kecitai3];%%������������֧����18��΢С��λ������
TiFen=mat2cell(Ti,[6,0],[6,6,6]);%����ֿ�
s1i=[1 1 1;0 0 0;0 0 0];%%��������֧���ĵ�һ�ؽ������߷���
R11=Ryz*[cos(sita12i(1,1)) -sin(sita12i(1,1)) 0;sin(sita12i(1,1)) cos(sita12i(1,1)) 0;0 0 1]*Ryz;%%Ϊ�˵õ���һ֧���ڶ��ؽ����߷���
s2i=[R11(:,3),s24,s24];%%����֧���ڶ��ؽ����߷���
R4i=zeros(3,9);%%���ڴ�������֧�����Ĺ�����̬���󣬻�ȡ��5�ؽ����߷���
R4iFen=mat2cell(R4i,[3,0],[3,3,3]);%%�ֳ����飬����֧��
for i=1:3
    R4iFen{1,i}=R3ifen{1,i}*[cos(sita456i(1,i)+y(1,i)),-sin(sita456i(1,i)+y(1,i)),0;sin(sita456i(1,i)+y(1,i)),cos(sita456i(1,i)+y(1,i)),0;0,0,1]*Ryz;
end
R41=R4iFen{1,1};
R42=R4iFen{1,2};
R43=R4iFen{1,3};%%����֧�����Ĺ�����̬����
R4i=[R41,R42,R43];
R5i=zeros(3,9);%%���ڴ�������֧�����幹����̬���󣬻�ȡ��6�ؽ����߷���
R5iFen=mat2cell(R5i,[3,0],[3,3,3]);
for i=1:3
    R5iFen{1,i}=R4iFen{1,i}*[cos(sita456i(2,i)),-sin(sita456i(2,i)),0;sin(sita456i(2,i)),cos(sita456i(2,i)),0;0,0,1]*Ryz;
end%%��Ȼ��֧��1���е����
R51=R5iFen{1,1};
R52=R5iFen{1,2};
R53=R5iFen{1,3};%%����֧����5������̬����
R5i=[R51,R52,R53];
for i=1:3
    TiFen{1,i}(:,1)=[cross(ci(:,i)-q3i(i,1)*s3i(:,i),s1i(:,i));s1i(:,i)];
    TiFen{1,i}(:,2)=[cross(ci(:,i)-q3i(i,1)*s3i(:,i),s2i(:,i));s2i(:,i)];
    TiFen{1,i}(:,3)=[s3i(:,i);0;0;0];
    TiFen{1,i}(:,4)=[cross(ci(:,i),s3i(:,i));s3i(:,i)];
    TiFen{1,i}(:,5)=[cross(ci(:,i),R4i(:,3*i));R4i(:,3*i)];
    TiFen{1,i}(:,6)=[cross(ci(:,i),R5i(:,3*i));R5i(:,3*i)];
end%%����Ti
kecitai1=TiFen{1,1};
kecitai2=TiFen{1,2};
kecitai3=TiFen{1,3};
Ti=[kecitai1,kecitai2,kecitai3];
zhudongzhiliansudu=zeros(6,3);
for i=1:3
zhudongzhiliansudu(:,i)=TiFen{1,i}\(Tap*zhudongguanjiesudu([1 2 3],1))
end%%������������֧�������йؽڵ��ٶ�

kecita14=[-qe*cross(s34,s14);s14];%%���㱻��֧��ǰ3�ؽڵ��ٶ�
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
beidongzhiliansudu(:,s+1)=Wa4'*Tap*zhudongguanjiesudu([1 2 3],1);%%���㱻��֧��ǰ3�ؽڵ��ٶ�

%%���¼�������˵ļ��ٶ�
%%����ʹ�þ����ʽ����
%%����ȫ���ؽڵ��ٶ�����
kecit1=zeros(6,6);%%֧��1���ٶ�����
kecit1(:,1)=zhudongzhiliansudu(1,1)*kecitai1(:,1);
for i=2:6
    kecit1(:,i)=kecit1(:,i-1)+zhudongzhiliansudu(i,1)*kecitai1(:,i);
end
T1weifen=zeros(6,6);%%֧��1���ſɱȾ����΢��
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

kecit2=zeros(6,6);%%֧��2���ٶ�����
kecit2(:,1)=zhudongzhiliansudu(1,2)*kecitai2(:,1);
for i=2:6
    kecit2(:,i)=kecit2(:,i-1)+zhudongzhiliansudu(i,2)*kecitai2(:,i);
end
T2weifen=zeros(6,6);%%֧��2���ſɱȾ����΢��
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

kecit3=zeros(6,6);%%֧��3���ٶ�����
kecit3(:,1)=zhudongzhiliansudu(1,3)*kecitai3(:,1);
for i=2:6
    kecit3(:,i)=kecit3(:,i-1)+zhudongzhiliansudu(i,3)*kecitai3(:,i);
end
T3weifen=zeros(6,6);%%֧��3���ſɱȾ����΢��
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


kecit4=zeros(6,6);%%֧��4���ٶ�����
sudu4=[beidongzhiliansudu(:,s+1);zhudongguanjiesudu([4 5 6],1)];
kecita4=[kecita14,kecita24,kecita34,kecita44,kecita54,kecita64];
kecit4(:,1)=sudu4(1,1)*kecita4(:,1);
for i=2:6
    kecit4(:,i)=kecit4(:,i-1)+sudu4(i,1)*kecita4(:,i);
end
T4weifen=zeros(6,6);%%֧��4���ſɱȾ����΢��
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
    keciac=[-300;0;0;0;0;0];%%��C�ļ��ٶ�����
elseif 1<=time(1,s+1)&&time(1,s+1)<2
    keciac=[0;0;0;0;0;0];%%��C�ļ��ٶ�����
else
    keciac=[300;0;0;0;0;0];%%��C�ļ��ٶ�����
end
jiasudu4(:,s+1)=kecita4\(keciac-T4weifen*sudu4);%%֧��4��6���ؽڵĽǼ��ٶ�

%%���㶯ƽ̨���ڵ�P�ļ��ٶ�����
keciap=T4weifen(:,[1 2 3])*beidongzhiliansudu(:,s+1)+kecita4(:,[1 2 3])*jiasudu4([1 2 3],s+1);
%%����1 2 3��������֧�����ؽڼ��ٶ�
jiasudu1(:,s+1)=kecitai1\(keciap-T1weifen*zhudongzhiliansudu(:,1));
jiasudu1([1 2 4 5 6],s+1)=jiasudu1([1 2 4 5 6],s+1)*180/pi;%%֧��1�ؽ�5��6���ٶ�1.5s�������ֵ��һ�������ţ���������н��ͣ�������ʱ��������
if time(1,s+1)>=1.5
    jiasudu1([5 6],s+1)=-jiasudu1([5 6],s+1);%%�����һ����ĸ���������Ҫ�޸�128��129��
end
jiasudu2(:,s+1)=kecitai2\(keciap-T2weifen*zhudongzhiliansudu(:,2));
jiasudu2([1 2 4 5 6],s+1)=jiasudu2([1 2 4 5 6],s+1)*180/pi;
jiasudu3(:,s+1)=kecitai3\(keciap-T3weifen*zhudongzhiliansudu(:,3));
jiasudu3([1 2 4 5 6],s+1)=jiasudu3([1 2 4 5 6],s+1)*180/pi;

%%���ԣ��Ե�4֧��������ٶ�,ʹ��˫�ۼӷ�
% kecita4=[kecita14,kecita24,kecita34,kecita44,kecita54,kecita64];
% sudu4=[beidongzhiliansudu;zhudongguanjiesudu([4 5 6],1)];%%֧��4��6���ؽ��ٶ�
% sigema=zeros(6,1);
% for j=2:6
%     for k=1:j-1
%         omg=[0,-kecita4(6,k),kecita4(5,k);kecita4(6,k),0,-kecita4(4,k);-kecita4(5,k),kecita4(4,k),0];
%         vi=[0,-kecita4(3,k),kecita4(2,k);kecita4(3,k),0,-kecita4(1,k);-kecita4(2,k),kecita4(1,k),0];
%         chacheng=[omg,vi;zeros(3,3),omg];
%         sigema=sigema+sudu4(j,1)*sudu4(k,1)*chacheng*kecita4(:,j);
%     end
% end
% jiasudu4(1,s+1)=inv(kecita4)*(keciac-sigema);%%�ý���ĺ�����Ӧ��Ϊתͷ���ؽڵļ��ٶ�ֵ����ʱ����

%%���¼�����嶯��ѧ
%%���ȸ���ţ��-ŷ�����̡��鹦ԭ����д����
I1ijubu=[136369297 0 0;0 136344677 0;0 0 2094913];%%��������֧����һ����(��Ͳ)�ھֲ�����ϵ�µĹ�������
I14jubu=[102854706 0 0;0 16869091 207422;0 207422 91255649];%%����֧����һ����(ת����)��������
m1i=4833;%%��Ͳ����
m14=5324;%%ת��������
M1ijubu=[m1i*eye(3),zeros(3,3);zeros(3,3),I1ijubu];%%����������Ծ���
M14jubu=[m14*eye(3),zeros(3,3);zeros(3,3),I14jubu];
g11=(R31*[0;0;-18]+[0;-294.16;0]-rp);%%֧��1����1�����ģ��ڲο�����ϵ�µ�����
g12=(R32*[0;0;-18]+[167;0;0]-rp);
g13=(R33*[0;0;-18]+[-167;0;0]-rp);
g14=(R14*[0;0;-5.6]-rp);
g11chacheng=[0,-g11(3,1),g11(2,1);g11(3,1),0,-g11(1,1);-g11(2,1),g11(1,1),0];
g12chacheng=[0,-g12(3,1),g12(2,1);g12(3,1),0,-g12(1,1);-g12(2,1),g12(1,1),0];
g13chacheng=[0,-g13(3,1),g13(2,1);g13(3,1),0,-g13(1,1);-g13(2,1),g13(1,1),0];
g14chacheng=[0,-g14(3,1),g14(2,1);g14(3,1),0,-g14(1,1);-g14(2,1),g14(1,1),0];
Adg11=[R31,g11chacheng*R31;zeros(3,3),R31];%%�Ӿֲ�����ϵ���ο�����ϵ�İ���任����
Adg12=[R32,g12chacheng*R32;zeros(3,3),R32];
Adg13=[R33,g13chacheng*R33;zeros(3,3),R33];
Adg14=[R14,g14chacheng*R14;zeros(3,3),R14];
M11=inv(Adg11')*M1ijubu*inv(Adg11);%%�ڲο�����ϵ�µĹ�����Ծ���
M12=inv(Adg12')*M1ijubu*inv(Adg12);
M13=inv(Adg13')*M1ijubu*inv(Adg13);
M14=inv(Adg14')*M14jubu*inv(Adg14);

I2ijubu=[17984609,0,0;0,17999476,0;0,0,195625];%%����֧���ڶ�����(�Ƹ�)�ֲ�����ϵ�µĹ�������
I24jubu=[689668.8,9.917,-2588.8;9.917,690657.3,6302;-2588.8,6302,28627.96];%%����֧���ڶ�������֧���壩�ھֲ�����ϵ�µĹ�������
m2i=1263;%%�Ƹ�����
m24=15379;%%֧��������
M2ijubu=[m2i*eye(3),zeros(3,3);zeros(3,3),I2ijubu];%%�����ֲ�����ϵ�µĹ�����Ծ���
M24jubu=[m24*eye(3),zeros(3,3);zeros(3,3),I24jubu];
g21=(R31*[0;0;q3i(1,1)-209.5]+[0;-294.16;0]-rp);%%�ڲο�����ϵ�µ���������
g22=(R32*[0;0;q3i(2,1)-209.5]+[167;0;0]-rp);
g23=(R33*[0;0;q3i(3,1)-209.5]+[-167;0;0]-rp);
g24=(R34(:,3)*(q34-208)-rp);
g21chacheng=[0,-g21(3,1),g21(2,1);g21(3,1),0,-g21(1,1);-g21(2,1),g21(1,1),0];
g22chacheng=[0,-g22(3,1),g22(2,1);g22(3,1),0,-g22(1,1);-g22(2,1),g22(1,1),0];
g23chacheng=[0,-g23(3,1),g23(2,1);g23(3,1),0,-g23(1,1);-g23(2,1),g23(1,1),0];
g24chacheng=[0,-g24(3,1),g24(2,1);g24(3,1),0,-g24(1,1);-g24(2,1),g24(1,1),0];
Adg21=[R31,g21chacheng*R31;zeros(3,3),R31];%%�Ӹ�����ϵ���ο�ϵ�İ���任����
Adg22=[R32,g22chacheng*R32;zeros(3,3),R32];
Adg23=[R33,g23chacheng*R33;zeros(3,3),R33];
Adg24=[R34,g24chacheng*R34;zeros(3,3),R34];
M21=inv(Adg21')*M2ijubu*inv(Adg21);%%�ڲο�����ϵ�µĹ�����Ծ���
M22=inv(Adg22')*M2ijubu*inv(Adg22);
M23=inv(Adg23')*M2ijubu*inv(Adg23);
M24=inv(Adg24')*M24jubu*inv(Adg24);
%%Ϊ�˼���Ta,j,i
Ta11pie=kecitai1(:,[1 2]);%%ע�⶯��ѧ���ǵĵ�һ��������ʵ���˶�ѧ�еĵڶ�����(��Ͳ)
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
Ta11=Ta11pie*E11*Wa1'*Tap;%%���˿�����Ta,j,i��
Ta21=Ta21pie*E21*Wa1'*Tap;
Ta12=Ta12pie*E12*Wa2'*Tap;
Ta22=Ta22pie*E22*Wa2'*Tap;
Ta13=Ta13pie*E13*Wa3'*Tap;
Ta23=Ta23pie*E23*Wa3'*Tap;
Ta14=Ta14pie*E14*Wa4'*Tap;
Ta24=Ta24pie*E24*Wa4'*Tap;
Is=11309;%%����˿�ܺ����������������ߵ�ת��������ע����SW��û��˿����ת�����ܵ������
daocheng=4;%%˿�ܵĵ���
D3=Ta11'*M11*Ta11+Ta21'*M21*Ta21+Ta12'*M12*Ta12+Ta22'*M22*Ta22+Ta13'*M13*Ta13+Ta23'*M23*Ta23+...
    Ta14'*M14*Ta14+Ta24'*M24*Ta24+eye(3)*Is*(2*pi/daocheng)^2;

sigemaa11=T1weifen(:,[1 2])*zhudongzhiliansudu([1 2],1);%%����Ǽ������ʱ�ĵڶ���
sigemaa21=T1weifen(:,[1 2 3])*zhudongzhiliansudu([1 2 3],1);
sigemaa12=T2weifen(:,[1 2])*zhudongzhiliansudu([1 2],2);
sigemaa22=T2weifen(:,[1 2 3])*zhudongzhiliansudu([1 2 3],2);
sigemaa13=T3weifen(:,[1 2])*zhudongzhiliansudu([1 2],3);
sigemaa23=T3weifen(:,[1 2 3])*zhudongzhiliansudu([1 2 3],3);
sigemaa14=T4weifen(:,1)*beidongzhiliansudu(1,s+1);
sigemaa24=T4weifen(:,[1 2 3])*beidongzhiliansudu([1 2 3],s+1);
H3=Ta11'*(M11*sigemaa11-kecit21chacheng'*M11*kecit1(:,2))+...%%����H3
   Ta21'*(M21*sigemaa21-kecit31chacheng'*M21*kecit1(:,3))+...
   Ta12'*(M12*sigemaa12-kecit22chacheng'*M12*kecit2(:,2))+...
   Ta22'*(M22*sigemaa22-kecit32chacheng'*M22*kecit2(:,3))+...
   Ta13'*(M13*sigemaa13-kecit23chacheng'*M13*kecit3(:,2))+...
   Ta23'*(M23*sigemaa23-kecit33chacheng'*M23*kecit3(:,3))+...
   Ta14'*(M14*sigemaa14-kecit14chacheng'*M14*kecit4(:,1))+...
   Ta24'*(M24*sigemaa24-kecit34chacheng'*M24*kecit4(:,3));

zhongli=[0;-sqrt(2)/2;sqrt(2)/2];%%�����ķ�������
G3=-(Ta11'*4833*9.8*10^(3)*[zhongli;cross(g11,zhongli)]+Ta21'*1263*9.8*10^(3)*[zhongli;cross(g21,zhongli)]+...%%����G3
     Ta12'*4833*9.8*10^(3)*[zhongli;cross(g12,zhongli)]+Ta22'*1263*9.8*10^(3)*[zhongli;cross(g22,zhongli)]+...
     Ta13'*4833*9.8*10^(3)*[zhongli;cross(g13,zhongli)]+Ta23'*1263*9.8*10^(3)*[zhongli;cross(g23,zhongli)]+...
     Ta14'*5324*9.8*10^(3)*[zhongli;cross(g14,zhongli)]+Ta24'*15379*9.8*10^(3)*[zhongli;cross(g24,zhongli)]);

I44jubu=[78253267 -4919741 1033606;-4919741 45881663 2580650;1033606 2580650 56855084];%%תͷ��һ�����ھֲ�����ϵ�µĹ�������
g44jubu=[-11.11;-83.47;2.7];%%�ֲ�����ϵ����������
m44=7540;%%��������
I54jubu=[15011772 0 0;0 14884080 0;0 0 6173188];
g54jubu=[0;0;9.34];
m54=4258;
I64jubu=[2912854 0 154973;0 3111333 0;154973 0 1675256];
g64jubu=[-4.1;0;111.73];
m64=1819;
M44jubu=[m44*eye(3),zeros(3,3);zeros(3,3),I44jubu];%%�����ֲ�����ϵ�µĹ�����Ծ���
M54jubu=[m54*eye(3),zeros(3,3);zeros(3,3),I54jubu];
M64jubu=[m64*eye(3),zeros(3,3);zeros(3,3),I64jubu];
g44=R44*g44jubu;%%�ڲο�����ϵ�µ���������
g54=R54*g54jubu;
g64=R64*g64jubu-rcp;
g44chacheng=[0,-g44(3,1),g44(2,1);g44(3,1),0,-g44(1,1);-g44(2,1),g44(1,1),0];
g54chacheng=[0,-g54(3,1),g54(2,1);g54(3,1),0,-g54(1,1);-g54(2,1),g54(1,1),0];
g64chacheng=[0,-g64(3,1),g64(2,1);g64(3,1),0,-g64(1,1);-g64(2,1),g64(1,1),0];
Adg44=[R44,g44chacheng*R44;zeros(3,3),R44];%%����ϵ����ڲο�ϵ�İ���任����
Adg54=[R54,g54chacheng*R54;zeros(3,3),R54];
Adg64=[R64,g64chacheng*R64;zeros(3,3),R64];
M44=inv(Adg44')*M44jubu*inv(Adg44);%%�ο�ϵ�µĹ�����Ծ���
M54=inv(Adg54')*M54jubu*inv(Adg54);
M64=inv(Adg64')*M64jubu*inv(Adg64);
Ta44=[Tap,kecita44];
Ta54=[Ta44,kecita54];
Ta64=[Ta54,kecita64];
D4=Ta44'*M44*Ta44;%%��ʱ�����Ǵ���������ת������������������
D5=Ta54'*M54*Ta54;
D6=Ta64'*M64*Ta64;

sigemaa44=T4weifen(:,[1 2 3 4])*sudu4([1 2 3 4],1);%%��ͬ������ٶ�ʽ�еĵڶ���
sigemaa54=T4weifen(:,[1 2 3 4 5])*sudu4([1 2 3 4 5],1);
sigemaa64=T4weifen(:,[1 2 3 4 5 6])*sudu4;
H4=Ta44'*(M44*sigemaa44-kecit44chacheng'*M44*kecit4(:,4));
H5=Ta54'*(M54*sigemaa54-kecit54chacheng'*M54*kecit4(:,5));
H6=Ta64'*(M64*sigemaa64-kecit64chacheng'*M44*kecit4(:,6));
G4=-Ta44'*m44*9.8*10^(3)*[zhongli;cross(g44,zhongli)];
G5=-Ta54'*m54*9.8*10^(3)*[zhongli;cross(g54,zhongli)];
G6=-Ta64'*m64*9.8*10^(3)*[zhongli;cross(g64,zhongli)];
%%D����ķֿ�
D113=D3;%%��ʼ�ֿ飬�������
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
%%H����ķֿ�
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
%%G����ķֿ�
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
zhudongjiasudu=[jiasudu1(3,s+1);jiasudu2(3,s+1);jiasudu3(3,s+1);jiasudu4([4 5 6],s+1)];%%6�������ؽڵļ��ٶ�

TAO(:,s+1)=Dq*zhudongjiasudu+Hq+Gq;%%6�������ؽڵ�������/����

zhudong1(1,s+1)=zhudongguanjiesudu(1,1);
zhudong2(1,s+1)=zhudongguanjiesudu(2,1);
zhudong3(1,s+1)=zhudongguanjiesudu(3,1);
zhudong4(1,s+1)=zhudongguanjiesudu(4,1)*180/pi;
zhudong5(1,s+1)=zhudongguanjiesudu(5,1)*180/pi;
zhudong6(1,s+1)=zhudongguanjiesudu(6,1)*180/pi;%%6�������ؽڵ��ٶ���ʱ��ı仯����
end
figure(1);%%���������˵��ٶ�
plot(time,zhudong1,'r',time,zhudong2,'g',time,zhudong3,'b');
xlabel('ʱ�� s');
ylabel('���ٶ� mm/s');
hold on;
figure(2);%%תͷ�����ؽڵ��ٶ�
plot(time,zhudong4,'r',time,zhudong5,'g',time,zhudong6,'b');
xlabel('ʱ�� s');
ylabel('���ٶ� deg/s');
hold on;
figure(3);%%���������˵ļ��ٶ�
plot(time,jiasudu1(3,:),'r',time,jiasudu2(3,:),'g',time,jiasudu3(3,:),'b');
xlabel('ʱ�� s');
ylabel('���ٶ� mm/s^2');
hold on;
figure(4);%%תͷ���ؽڵļ��ٶ�
plot(time,jiasudu4(4,:)*180/pi,'r',time,jiasudu4(5,:)*180/pi,'g',time,jiasudu4(6,:)*180/pi,'b');
xlabel('ʱ�� s');
ylabel('�Ǽ��ٶ� deg/s^2');
%%��֤���ٶȵ���ȷ�ԣ��ļ���ȡ����Solidworks�в����Ľ��
% [num1]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��1�ؽ�1�Ǽ��ٶ�.xls');
% [num2]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��1�ؽ�2�Ǽ��ٶ�.xls');
% [num3]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��1�ؽ�4�Ǽ��ٶ�.xls');
% hold on;
% figure(5);
% plot(time,num1(:,2),'r--',time,num2(:,2),'g--',time,num3(:,2),'b--',time,jiasudu1(1,:),'r',time,jiasudu1(2,:),'g',time,jiasudu1(4,:),'b');
% xlabel('ʱ�� s');
% ylabel('�Ǽ��ٶ� deg/s^2');
% [num4]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��1�ؽ�5�Ǽ��ٶ�.xls');
% [num5]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��1�ؽ�6�Ǽ��ٶ�.xls');
% [num6]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��2�ؽ�2�Ǽ��ٶ�.xls');
% hold on;
% figure(5);
% plot(time,num4(:,2),'r--',time,num5(:,2),'g--',time,num6(:,2),'b--',time,jiasudu1(5,:),'r',time,jiasudu1(6,:),'g',time,jiasudu2(2,:),'b');
% xlabel('ʱ�� s');
% ylabel('�Ǽ��ٶ� deg/s^2');%%���ֵ�һ֧��5��6�ؽڼ��ٶ�1.5s�������ֵ��һ�������ţ�������4��5�ؽ�ת����ɵ�,�Ѿ����ֵ�һ֧��5��6�ؽ��������ԣ���x����ת��180��
% [num7]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��2�ؽ�6�Ǽ��ٶ�.xls');
% [num8]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��3�ؽ�2�Ǽ��ٶ�.xls');
% [num9]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��3�ؽ�6�Ǽ��ٶ�.xls');
% hold on;
% figure(5);
% plot(time,num7(:,2),'r--',time,num8(:,2),'g--',time,num9(:,2),'b--',time,jiasudu2(6,:),'r',time,jiasudu3(2,:),'g',time,jiasudu3(6,:),'b');
% xlabel('ʱ�� s');
% ylabel('�Ǽ��ٶ� deg/s^2');
% [num10]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\֧��4�ؽ�2�Ǽ��ٶ�.xls');
% hold on;
% figure(5);
% plot(time,num10(:,2),'r--',time,jiasudu4(2,:)*180/pi,'r');
% xlabel('ʱ�� s');
% ylabel('�Ǽ��ٶ� deg/s^2');
% [num11]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\���ĹؽڽǼ��ٶ�.xls');
% [num12]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\����ؽڽǼ��ٶ�.xls');
% [num13]=xlsread('C:\Users\YAYALE\Desktop\Solidworks�����ٶ�\�����ؽڽǼ��ٶ�.xls');
% hold on;
% figure(5);
% plot(time,num11(:,2),'r--',time,num12(:,2),'g--',time,num13(:,2),'b--',time,jiasudu4(4,:)*180/pi,'r',time,jiasudu4(5,:)*180/pi,'g',time,jiasudu4(6,:)*180/pi,'b');
% xlabel('ʱ�� s');
% ylabel('�Ǽ��ٶ� deg/s^2');
%%�������йؽڵļ��ٶȶ��Ͳ���ֵ����һ�£���֤��ģ�͵���ȷ��
hold on;
figure(5);%%��һ����֧���Ƹ�����
plot(time,TAO(1,:)/10^6,'r');
xlabel('ʱ�� s');
ylabel('���� N');
hold on;
figure(6);%%��2��3����֧���Ƹ�����
plot(time,TAO(2,:)/10^6,'g',time,TAO(3,:)/10^6,'b');
xlabel('ʱ�� s');
ylabel('���� N');
hold on;
figure(7);%%���Ĺؽ���������
plot(time,TAO(4,:)/10^6,'r');
xlabel('ʱ�� s');
ylabel('���� N.mm');
hold on;
figure(8);%%����ؽ���������
plot(time,TAO(5,:)/10^6,'r');
xlabel('ʱ�� s');
ylabel('���� N.mm');
hold on;
figure(9);%%�����ؽ���������
plot(time,TAO(6,:)/10^6,'r');
xlabel('ʱ�� s');
ylabel('���� N.mm');
hold on;
figure(10);
plot(time,jiasudu3(3,:),'r-o',time,jiasudu3(3,:),'g-*');
xlabel('t(s)');
ylabel('a(mm/s^2)');