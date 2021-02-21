clear
close all
clc

%% 机器人工具系标定

% 机器人模型,单位m和rad
d1 = 0.1;  d2 = 0;  d3 = 0;  d4 = 0.13;  d5 = 0.13;  d6 = 0.12;
a0 = 0;  a1 = 0;  a2 = 0.42;  a3 = 0.41;  a4 = 0;  a5 = 0;
alp0 = 0;  alp1 = pi/2;  alp2 = 0;  alp3 = 0;  alp4 = pi/2;  alp5 = -pi/2; 
d=[d1,d2,d3,d4,d5,d6];
a=[a0,a1,a2,a3,a4,a5];
alp=[alp0,alp1,alp2,alp3,alp4,alp5];
% DH parameters  th     d    a    alpha  sigma
L1 = Link([0, d(1), a(1), alp(1), 0], 'modified');
L2 = Link([0, d(2), a(2), alp(2), 0], 'modified');
L3 = Link([0, d(3), a(3), alp(3), 0], 'modified');
L4 = Link([0, d(4), a(4), alp(4), 0], 'modified');
L5 = Link([0, d(5), a(5), alp(5), 0], 'modified');  
L6 = Link([0, d(6), a(6), alp(6), 0], 'modified');
robot = SerialLink([L1, L2, L3, L4, L5, L6],'offset',[0,pi/2,0,pi/2,0,0]); 
to=[rotz(10)*rotx(5),[0.12,0.16,0.17]';[0,0,0,1]];
robot.tool=to; % 在机器人末端添加工具系
W=[roty(30)*rotz(34)*rotx(56),[0.8,0.7,0.69]';[0,0,0,1]]; % 测试物体在机器人基坐标系中的位姿

% 第1个位姿
Tr_x1=double(robot.fkine(deg2rad([10,20,30,40,50,60])));
Tr1=Tr_x1*inv(to);
Tc1=inv(Tr_x1)*W;  % 测试物体在工具系下的表示
% 第2个位姿
Tr_x2=double(robot.fkine(deg2rad([10,30,40,50,55,66])));
Tr2=Tr_x2*inv(to);
Tc2=inv(Tr_x2)*W;  % 测试物体在工具系下的表示
% 第3个位姿
Tr_x3=double(robot.fkine(deg2rad([5,42,51,43,66,0])));
Tr3=Tr_x3*inv(to);
Tc3=inv(Tr_x3)*W;  % 测试物体在工具系下的表示


% 工具系标定
%  Step 1 : 求解  RA*X=X*RB  RC*X=X*RD
% 两个不同为位姿
TA=inv(Tr2)*Tr1;
A=TA(1:3,1:3);
TB=Tc2*inv(Tc1);
B=TB(1:3,1:3);
TC=inv(Tr3)*Tr2;
C=TC(1:3,1:3);
TD=Tc3*inv(Tc2);
D=TD(1:3,1:3);

% 转换为四元数,行向量
pA=dcm2quat(A);
pB =dcm2quat(B);
pC=dcm2quat(C);
pD =dcm2quat(D);
% 转换为欧拉参数
theta_A=2*acos(pA(1));
theta_B=2*acos(pB(1));% 理论上两个角度是同等的
theta_C=2*acos(pC(1));
theta_D=2*acos(pD(1));% 理论上两个角度是同等的

ua=pA(2:4)/sin(theta_A/2);
ub=pB(2:4)/sin(theta_B/2);
uc=pC(2:4)/sin(theta_C/2);
ud=pD(2:4)/sin(theta_D/2);

BB1=[0,-ua+ub;(ua-ub)',get_Skew_symmetric_mat(ua)+get_Skew_symmetric_mat(ub)];
BB2=[0,-uc+ud;(uc-ud)',get_Skew_symmetric_mat(uc)+get_Skew_symmetric_mat(ud)];
BB=[BB1;BB2];
% matlab求解线性齐次方程的函数为null()
qX=null(BB) ; % 返回标准正交基，即相互正交的通解
RX=quat2dcm(qX')'
to(1:3,1:3)

% Step 2 : 求  (RA1-I)tx=RX*tB1-tA1  ,  (RA2-I)tx=RX*tB2-tA2
E=[A-eye(3);C-eye(3)];  % 论文中的C
F=[RX*TB(1:3,4)-TA(1:3,4);RX*TD(1:3,4)-TC(1:3,4)];
tx=inv(E'*E)*E'*F
to(1:3,4)



