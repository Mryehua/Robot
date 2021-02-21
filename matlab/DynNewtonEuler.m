% 六自由度机械臂瞬态运动的牛顿-欧拉递归逆动力学求解：
% 参数：（运动指令）各关节运动角度， 关节速度， 关节加速度（6*1矩阵）
% 返回值：各关节力矩（6*1矩阵）
% 由于我不清楚机械臂实际质量参数，因此下面的惯性张量及质心位置均为假设杜撰
function tau = DynNewtonEuler(theta, theta_d, theta_dd,g,Fex)
%% 初始化
% 改进D-H参数

d1 = 0.103;  d2 = 0;  d3 = 0;  d4 = 0.1385;  d5 = 0.134;  d6 = 0.127;
a0 = 0;  a1 = 0;  a2 = 0.426;  a3 = 0.414;  a4 = 0;  a5 = 0;
alp0 = 0;  alp1 = pi/2;  alp2 = 0;  alp3 = 0;  alp4 = pi/2;  alp5 = -pi/2; 
d=[d1,d2,d3,d4,d5,d6];
a=[a0,a1,a2,a3,a4,a5];
alp=[alp0,alp1,alp2,alp3,alp4,alp5];

th(1) = theta(1);
th(2) = theta(2);   
th(3) = theta(3); 
th(4) = theta(4); 
th(5) = theta(5); 
th(6) = theta(6);


w00 = [0; 0; 0]; v00 = [0; 0; 0]; w00d = [0; 0; 0]; v00d = g;%base_link的各项初始值

z = [0; 0; 1];
m = [2.9258;  7.134;  3.5585;  1.6688;  1.4967;  0.2297];  %kg
m1 =m(1); m2 =m(2); m3 = m(3); m4 =m(4); m5 =m(5); m6 = m(6);
% 惯性张量
I1 =  [0.0111 0 0; 0 0.0108 0.0004; 0 0.0004 0.0072]; 
I2 = [0.1439 0 0.1988; 0 0.7297 0; 0.1988 0 0.6027];
I3 = [0.0093 0 0.0118; 0 0.3427 0.3405; 0.0118 0.3405 0.3405];
I4 = [0.0052 0 0; 0 0.0030 0.00017; 0 0.00017 0.0051];
I5 = [0.0042 0 0; 0 0.0029 -0.0001; 0 -0.0001 0.0041]; 
I6 = [0.00051 0 0; 0 0.00051 0; 0 0 0.00014];

% 各齐次矩阵function T = MDHTrans(alpha, a, d, theta)
T01 = MDHTrans(alp(1), a(1), d(1), th(1));
T12 = MDHTrans(alp(2), a(2), d(2), th(2));
T23 = MDHTrans(alp(3), a(3), d(3), th(3));
T34 = MDHTrans(alp(4), a(4), d(4), th(4));
T45 = MDHTrans(alp(5), a(5), d(5), th(5));
T56 = MDHTrans(alp(6), a(6), d(6), th(6));

% 各关节p及各link质心pc的距离(假设质心在几何中心)
p10 = T01(1: 3, 4); p21 = T12(1: 3, 4); p32 = T23(1: 3, 4);
p43 = T34(1: 3, 4); p54 = T45(1: 3, 4); p65 = T56(1: 3, 4); p76 = [0, 0, 0]';
pc11 =  [0;    -0.0049524;      -0.024551];
pc22 = [        0.213;                  0;          0.13086];
pc33 = [    0.25362;                  0;         0.013039];
pc44 =  [              0;     -0.021096;      -0.0047608];
pc55 = [              0;      0.015883;      -0.0053082];
pc66 = [              0;                 0;         -0.041194];

% 旋转矩阵
R01 = T01(1:3, 1:3); R12 = T12(1:3, 1:3); R23 = T23(1:3, 1:3);
R34 = T34(1:3, 1:3); R45 = T45(1:3, 1:3); R56 = T56(1:3, 1:3);
R10 = R01'; R21 = R12'; R32 = R23';
R43 = R34'; R54 = R45'; R65 = R56';
R67 = [1 0 0; 0 1 0; 0 0 1]; R76 = R67';

%% Outward iterations: i: 0->5
% 连杆1到连杆6向外迭代
% i = 0
w11 = R10*w00 + theta_d(1)*z;
w11d = R10*w00d + cross(R10*w00, z*theta_d(1)) + theta_dd(1)*z;
v11d = R10*(cross(w00d, p10) + cross(w00, cross(w00, p10)) + v00d);
vc11d = cross(w11d, pc11) + cross(w11, cross(w11, pc11)) + v11d;
F11 = m1*vc11d;
N11 = I1*w11d + cross(w11, I1*w11);
% i = 1
w22 = R21*w11 + theta_d(2)*z;
w22d = R21*w11d + cross(R21*w11, z*theta_d(2)) + theta_dd(2)*z;
v22d = R21*(cross(w11d, p21) + cross(w11, cross(w11, p21)) + v11d);
vc22d = cross(w22d, pc22) + cross(w22, cross(w22, pc22)) + v22d;
F22 = m2*vc22d;
N22 = I2*w22d + cross(w22, I2*w22);
% i = 2
w33 = R32*w22 + theta_d(3)*z;
w33d = R32*w22d + cross(R32*w22, z*theta_d(3)) + theta_dd(3)*z;
v33d = R32*(cross(w22d, p32) + cross(w22, cross(w22, p32)) + v22d);
vc33d = cross(w33d, pc33) + cross(w33, cross(w33, pc33)) + v33d;
F33 = m3*vc33d;
N33 = I3*w33d + cross(w33, I3*w33);
% i= 3
w44 = R43*w33 + theta_d(4)*z;
w44d = R43*w33d + cross(R43*w33, z*theta_d(4)) + theta_dd(4)*z;
v44d = R43*(cross(w33d, p43) + cross(w33, cross(w33, p43)) + v33d);
vc44d = cross(w44d, pc44) + cross(w44, cross(w44, pc44)) + v44d;
F44 = m4*vc44d;
N44 = I4*w44d + cross(w44, I4*w44);
% i = 4
w55 = R54*w44 + theta_d(5)*z;
w55d = R54*w44d + cross(R54*w44, z*theta_d(5)) + theta_dd(5)*z;
v55d = R54*(cross(w44d, p54) + cross(w44, cross(w44, p54)) + v44d);
vc55d = cross(w55d, pc55) + cross(w55, cross(w55, pc55)) + v55d;
F55 = m5*vc55d;
N55 = I5*w55d + cross(w55, I5*w55);
% i = 5
w66 = R65*w55 + theta_d(6)*z;
w66d = R65*w55d + cross(R65*w55, z*theta_d(6)) + theta_dd(6)*z;
v66d = R65*(cross(w55d, p65) + cross(w55, cross(w55, p65)) + v55d);
vc66d = cross(w66d, pc66) + cross(w66, cross(w66, pc66)) + v66d;
F66 = m6*vc66d;
N66 = I6*w66d + cross(w66, I6*w66);

%% Inward iterations: i: 6->1
% 连杆6到连杆1向内迭代
f77 = Fex(1:3); n77 = Fex(4:6);
% i = 6
f66 = R67*f77 + F66;
n66 = N66 + R67*n77 + cross(pc66, F66) + cross(p76, R67*f77);
tau6 = n66'*z;
% i = 5
f55 = R56*f66 + F55;
n55 = N55 + R56*n66 + cross(pc55, F55) + cross(p65, R56*f66);
tau5= n55'*z;
% i = 4
f44 = R45*f55 + F44;
n44 = N44 + R45*n55 + cross(pc44, F44) + cross(p54, R45*f55);
tau4 = n44'*z;
% i = 3
f33 = R34*f44 + F33;
n33 = N33 + R34*n44 + cross(pc33, F33) + cross(p43, R34*f44);
tau3 = n33'*z;
% i = 2
f22 = R23*f33 + F22;
n22 = N22 + R23*n33 + cross(pc22, F22) + cross(p32, R23*f33);
tau2 = n22'*z;
% i =1
f11 = R12*f22 + F11;
n11 = N11 + R12*n22 + cross(pc11, F11) + cross(p21, R12*f22);
tau1= n11'*z;
tau=[tau1;tau2;tau3;tau4;tau5;tau6];

end