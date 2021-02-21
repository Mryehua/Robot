function [sys,x0,str,ts] = ctrler(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 30;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];
end

function sys=mdlOutputs(t,x,u)
% 输出为关节力矩，数量6
% 输入为各关节状态量，转角，速度，加速度

q1_d=u(1);dq1_d=u(2);ddq1_d=u(3);
q2_d=u(4);dq2_d=u(5);ddq2_d=u(6);
q3_d=u(7);dq3_d=u(8);ddq3_d=u(9);
q4_d=u(10);dq4_d=u(11);ddq4_d=u(12);
q5_d=u(13);dq5_d=u(14);ddq5_d=u(15);
q6_d=u(16);dq6_d=u(17);ddq6_d=u(18);

q1=u(19);dq1=u(20);
q2=u(21);dq2=u(22);
q3=u(23);dq3=u(24);
q4=u(25);dq4=u(26);
q5=u(27);dq5=u(28);
q6=u(29);dq6=u(30);


dq_d=[dq1_d,dq2_d,dq3_d,dq4_d,dq5_d,dq6_d]';
ddq_d=[ddq1_d,ddq2_d,ddq3_d,ddq4_d,ddq5_d,ddq6_d]';

q_error=[q1-q1_d,q2-q2_d,q3-q3_d,q4-q4_d,q5-q5_d,q6-q6_d]';
dq_error=[dq1-dq1_d,dq2-dq2_d,dq3-dq3_d,dq4-dq4_d,dq5-dq5_d,dq6-dq6_d]';

Fai=10*eye(6);
dqr=dq_d-Fai*q_error;
ddqr=ddq_d-Fai*dq_error;
s=Fai*q_error+dq_error;
Kd=900*eye(6);

[M,C,G]=dynamic_model(q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6);
tol=M*ddqr+C*dqr+G-Kd*s;
    
sys(1)=tol(1);
sys(2)=tol(2);
sys(3)=tol(3);
sys(4)=tol(4);
sys(5)=tol(5);
sys(6)=tol(6);
end

function [M,C,G] = dynamic_model(q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6)

g = [0;  0;  -9.81;  1];    % g = [gx;  gy;  gz;  1];
m = [2.9258;  7.134;  3.5585;  1.6688;  1.4967;  0.2297];  %kg
m1 = m(1);  m2 = m(2);  m3 = m(3);  m4 = m(4);  m5 = m(5);  m6 = m(6);

%ri表示连杆i的质心在坐标系i中的位置  单位：m
r1 = [              0;    -0.0049524;      -0.024551;   1];
r2 = [        0.213;                  0;          0.13086;   1];
r3 = [    0.25362;                  0;         0.013039;   1];
r4 = [              0;     -0.021096;      -0.0047608;   1];
r5 = [              0;      0.015883;      -0.0053082;   1];
r6 = [              0;                 0;         -0.041194;   1];

% Ii单位：kg.m2   由输出坐标系决定
I1 = [0.0111 0 0; 0 0.0108 0.0004; 0 0.0004 0.0072];
I2 = [0.1439 0 0.1988; 0 0.7297 0; 0.1988 0 0.6027];
I3 = [0.0093 0 0.0118; 0 0.3427 0.3405; 0.0118 0 0.3405];
I4 = [0.0052 0 0; 0 0.0030 0.00017; 0 0.00017 0.0051];
I5 = [0.0042 0 0; 0 0.0029 -0.0001; 0 -0.0001 0.0041];
I6 = [0.00051 0 0; 0 0.00051 0; 0 0 0.00014];

J1 = [(-I1(1,1)+I1(2,2)+I1(3,3))/2,                                    I1(1,2),                                   I1(1,3),    m1*r1(1,1);
                                          I1(2,1),      (I1(1,1)-I1(2,2)+I1(3,3))/2,                                  I1(2,3),    m1*r1(2,1);
                                          I1(3,1),                                    I1(3,2),    (I1(1,1)+I1(2,2)-I1(3,3))/2,    m1*r1(3,1);
                                   m1*r1(1,1),                             m1*r1(2,1),                           m1*r1(3,1),               m1];

J2 = [(-I2(1,1)+I2(2,2)+I2(3,3))/2,                                    I2(1,2),                                   I2(1,3),    m2*r2(1,1);
                                          I2(2,1),      (I2(1,1)-I2(2,2)+I2(3,3))/2,                                  I2(2,3),    m2*r2(2,1);
                                          I2(3,1),                                    I2(3,2),    (I2(1,1)+I2(2,2)-I2(3,3))/2,    m2*r2(3,1);
                                   m2*r2(1,1),                             m2*r2(2,1),                           m2*r2(3,1),               m2];

J3 = [(-I3(1,1)+I3(2,2)+I3(3,3))/2,                                    I3(1,2),                                   I3(1,3),    m3*r3(1,1);
                                          I3(2,1),      (I3(1,1)-I3(2,2)+I3(3,3))/2,                                  I3(2,3),    m3*r3(2,1);
                                          I3(3,1),                                    I3(3,2),    (I3(1,1)+I3(2,2)-I3(3,3))/2,    m3*r3(3,1);
                                   m3*r3(1,1),                             m3*r3(2,1),                           m3*r3(3,1),               m3];

J4 = [(-I4(1,1)+I4(2,2)+I4(3,3))/2,                                    I4(1,2),                                   I4(1,3),    m4*r4(1,1);
                                          I4(2,1),      (I4(1,1)-I4(2,2)+I4(3,3))/2,                                  I4(2,3),    m4*r4(2,1);
                                          I4(3,1),                                    I4(3,2),    (I4(1,1)+I4(2,2)-I4(3,3))/2,    m4*r4(3,1);
                                   m4*r4(1,1),                             m4*r4(2,1),                           m4*r4(3,1),               m4];

J5 = [(-I5(1,1)+I5(2,2)+I5(3,3))/2,                                    I5(1,2),                                   I5(1,3),    m5*r5(1,1);
                                          I5(2,1),      (I5(1,1)-I5(2,2)+I5(3,3))/2,                                  I5(2,3),    m5*r5(2,1);
                                          I5(3,1),                                    I5(3,2),    (I5(1,1)+I5(2,2)-I5(3,3))/2,    m5*r5(3,1);
                                   m5*r5(1,1),                             m5*r5(2,1),                           m5*r5(3,1),               m5];

J6 = [(-I6(1,1)+I6(2,2)+I6(3,3))/2,                                    I6(1,2),                                   I6(1,3),    m6*r6(1,1);
                                          I6(2,1),      (I6(1,1)-I6(2,2)+I6(3,3))/2,                                  I6(2,3),    m6*r6(2,1);
                                          I6(3,1),                                    I6(3,2),    (I6(1,1)+I6(2,2)-I6(3,3))/2,    m6*r6(3,1);
                                   m6*r6(1,1),                             m6*r6(2,1),                           m6*r6(3,1),               m6];
                               
U11 = [ -sin(q1), -cos(q1), 0, 0; 
            cos(q1),   -sin(q1), 0, 0; 
            0, 0, 0, 0; 
            0, 0, 0, 0];  

U21 = [sin(q1)*sin(q2),      cos(q2)*sin(q1), cos(q1), 0;
           -cos(q1)*sin(q2), -cos(q1)*cos(q2), sin(q1), 0;
            0   0   0   0;
            0   0   0   0];
U22 = [-cos(q1)*cos(q2), cos(q1)*sin(q2), 0, 0;
           -cos(q2)*sin(q1), sin(q1)*sin(q2), 0, 0;
           -sin(q2),    -cos(q2), 0, 0;
            0   0   0   0];

U31 = [sin(q2 + q3)*sin(q1),      cos(q2 + q3)*sin(q1), cos(q1),  0.426*sin(q1)*sin(q2);
           -sin(q2 + q3)*cos(q1), -cos(q2 + q3)*cos(q1), sin(q1), -0.426*cos(q1)*sin(q2);
           0   0   0   0;
           0   0   0   0];
U32 = [-cos(q2 + q3)*cos(q1), sin(q2 + q3)*cos(q1), 0, -0.426*cos(q1)*cos(q2);
             -cos(q2 + q3)*sin(q1), sin(q2 + q3)*sin(q1), 0, -0.426*cos(q2)*sin(q1);
             -sin(q2 + q3),    -cos(q2 + q3), 0,         -0.426*sin(q2);
            0   0   0   0];
U33 = [-cos(q2 + q3)*cos(q1), sin(q2 + q3)*cos(q1), 0, 0;
            -cos(q2 + q3)*sin(q1), sin(q2 + q3)*sin(q1), 0, 0;
            -sin(q2 + q3),    -cos(q2 + q3), 0, 0;
            0   0   0   0];

U41 = [cos(q2 + q3 + q4)*sin(q1), -sin(q2 + q3 + q4)*sin(q1), cos(q1), 0.1385*cos(q1) + 0.426*sin(q1)*sin(q2) + 0.414*cos(q2)*sin(q1)*sin(q3) + 0.414*cos(q3)*sin(q1)*sin(q2);
            -cos(q2 + q3 + q4)*cos(q1),      sin(q2 + q3 + q4)*cos(q1), sin(q1), 0.1385*sin(q1) - 0.426*cos(q1)*sin(q2) - 0.414*cos(q1)*cos(q2)*sin(q3) - 0.414*cos(q1)*cos(q3)*sin(q2);
            0   0   0   0;
            0   0   0   0];
U42 = [sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1), 0, -(3*cos(q1)*(69.0*cos(q2 + q3) + 71.0*cos(q2)))/500;
            sin(q2 + q3 + q4)*sin(q1), cos(q2 + q3 + q4)*sin(q1), 0, -(3*sin(q1)*(69.0*cos(q2 + q3) + 71.0*cos(q2)))/500;
            -cos(q2 + q3 + q4),         sin(q2 + q3 + q4), 0,                - 0.414*sin(q2 + q3) - 0.426*sin(q2);
            0   0   0   0];
U43 = [sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1), 0, -0.414*cos(q2 + q3)*cos(q1);
            sin(q2 + q3 + q4)*sin(q1), cos(q2 + q3 + q4)*sin(q1), 0, -0.414*cos(q2 + q3)*sin(q1);
             -cos(q2 + q3 + q4),         sin(q2 + q3 + q4), 0,         -0.414*sin(q2 + q3);
            0   0   0   0];
U44 = [sin(q2 + q3 + q4)*cos(q1), cos(q2 + q3 + q4)*cos(q1), 0, 0;
            sin(q2 + q3 + q4)*sin(q1), cos(q2 + q3 + q4)*sin(q1), 0, 0;
            -cos(q2 + q3 + q4),         sin(q2 + q3 + q4), 0, 0;
            0   0   0   0];

U51 = [cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1), cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5),      sin(q2 + q3 + q4)*sin(q1), 0.1385*cos(q1) - 0.134*sin(q4)*(1.0*sin(q1)*sin(q2)*sin(q3) - 1.0*cos(q2)*cos(q3)*sin(q1)) + 0.426*sin(q1)*sin(q2) + 0.134*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + 0.414*cos(q2)*sin(q1)*sin(q3) + 0.414*cos(q3)*sin(q1)*sin(q2);
            sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5), cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*sin(q2 + q3 + q4)*cos(q1), 0.1385*sin(q1) - 0.426*cos(q1)*sin(q2) - 0.134*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) - 0.134*sin(q4)*(1.0*cos(q1)*cos(q2)*cos(q3) - 1.0*cos(q1)*sin(q2)*sin(q3)) - 0.414*cos(q1)*cos(q2)*sin(q3) - 0.414*cos(q1)*cos(q3)*sin(q2);
            0   0   0   0;
            0   0   0   0];
U52 = [sin(q2 + q3 + q4)*cos(q1)*cos(q5), -sin(q2 + q3 + q4)*cos(q1)*sin(q5), -cos(q2 + q3 + q4)*cos(q1), -0.002*cos(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3) + 213.0*cos(q2));
            sin(q2 + q3 + q4)*cos(q5)*sin(q1), -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -cos(q2 + q3 + q4)*sin(q1), -0.002*sin(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3) + 213.0*cos(q2));
            -cos(q2 + q3 + q4)*cos(q5),              cos(q2 + q3 + q4)*sin(q5),         -sin(q2 + q3 + q4),               - 0.134*sin(q2 + q3 + q4) - 0.414*sin(q2 + q3) - 0.426*sin(q2);
            0   0   0   0];
U53 = [sin(q2 + q3 + q4)*cos(q1)*cos(q5), -sin(q2 + q3 + q4)*cos(q1)*sin(q5), -cos(q2 + q3 + q4)*cos(q1), -0.002*cos(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3));
            sin(q2 + q3 + q4)*cos(q5)*sin(q1), -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -cos(q2 + q3 + q4)*sin(q1), -0.002*sin(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3));
            -cos(q2 + q3 + q4)*cos(q5),              cos(q2 + q3 + q4)*sin(q5),         -sin(q2 + q3 + q4),               - 0.134*sin(q2 + q3 + q4) - 0.414*sin(q2 + q3);
            0   0   0   0];
U54 = [sin(q2 + q3 + q4)*cos(q1)*cos(q5), -sin(q2 + q3 + q4)*cos(q1)*sin(q5), -cos(q2 + q3 + q4)*cos(q1), -0.134*cos(q2 + q3 + q4)*cos(q1);
            sin(q2 + q3 + q4)*cos(q5)*sin(q1), -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -cos(q2 + q3 + q4)*sin(q1), -0.134*cos(q2 + q3 + q4)*sin(q1);
            -cos(q2 + q3 + q4)*cos(q5),              cos(q2 + q3 + q4)*sin(q5),         -sin(q2 + q3 + q4),         -0.134*sin(q2 + q3 + q4);
            0   0   0   0];
U55 = [cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5), cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 1.0*sin(q1)*sin(q5), 0, 0;
            cos(q2 + q3 + q4)*sin(q1)*sin(q5) - cos(q1)*cos(q5),     cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            sin(q2 + q3 + q4)*sin(q5),                               sin(q2 + q3 + q4)*cos(q5), 0, 0;
            0   0   0   0];

U61 = [cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6), - sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1), cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.1385*cos(q1) + 0.127*cos(q1)*cos(q5) + 0.426*sin(q1)*sin(q2) - 0.127*cos(q2 + q3 + q4)*sin(q1)*sin(q5) + 0.134*cos(q2 + q3)*sin(q1)*sin(q4) + 0.134*sin(q2 + q3)*cos(q4)*sin(q1) + 0.414*cos(q2)*sin(q1)*sin(q3) + 0.414*cos(q3)*sin(q1)*sin(q2);
           cos(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) + sin(q2 + q3 + q4)*cos(q1)*sin(q6),   sin(q2 + q3 + q4)*cos(q1)*cos(q6) - sin(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)), cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.1385*sin(q1) - 0.426*cos(q1)*sin(q2) + 0.127*cos(q5)*sin(q1) + 0.127*cos(q2 + q3 + q4)*cos(q1)*sin(q5) - 0.134*cos(q2 + q3)*cos(q1)*sin(q4) - 0.134*sin(q2 + q3)*cos(q1)*cos(q4) - 0.414*cos(q1)*cos(q2)*sin(q3) - 0.414*cos(q1)*cos(q3)*sin(q2);
            0   0   0   0;
            0   0   0   0];
U62 = [cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5),                       -(cos(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 426.0*cos(q2) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*sin(q1)*sin(q5),                       -(sin(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 426.0*cos(q2) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6),               sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),              cos(q2 + q3 + q4)*sin(q5), sin(q5)*(0.127*cos(q2 + q3)*cos(q4) - 0.127*sin(q2 + q3)*sin(q4)) - 0.426*sin(q2) - 0.134*cos(q2 + q3)*sin(q4) - 0.134*sin(q2 + q3)*cos(q4) - 0.414*sin(q2 + q3);
            0   0   0   0];
U63 = [cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*sin(q1)*sin(q5), -(sin(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6),               sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),              cos(q2 + q3 + q4)*sin(q5),                                             0.127*cos(q2 + q3 + q4)*sin(q5) - 0.414*sin(q2 + q3) - 0.134*sin(q2 + q3 + q4);
            0   0   0   0];
U64 = [cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(127.0*sin(q2 + q3 + q4)*sin(q5) + 134.0*cos(q2 + q3)*cos(q4) - 134.0*sin(q2 + q3)*sin(q4)))/1000;
            sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*sin(q1)*sin(q5), -(sin(q1)*(127.0*sin(q2 + q3 + q4)*sin(q5) + 134.0*cos(q2 + q3)*cos(q4) - 134.0*sin(q2 + q3)*sin(q4)))/1000;
            sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6),               sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),              cos(q2 + q3 + q4)*sin(q5),                                                   0.127*cos(q2 + q3 + q4)*sin(q5) - 0.134*sin(q2 + q3 + q4);
            0   0   0   0];
U65 = [cos(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -1.0*sin(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 1.0*sin(q1)*sin(q5), 0.127*cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 0.127*sin(q1)*sin(q5);
           -cos(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)),      sin(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)),     cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1), 0.127*cos(q1)*sin(q5) + 0.127*cos(q2 + q3 + q4)*cos(q5)*sin(q1);
            sin(q2 + q3 + q4)*cos(q6)*sin(q5),                             -1.0*sin(q2 + q3 + q4)*sin(q5)*sin(q6),                               sin(q2 + q3 + q4)*cos(q5),                                 0.127*sin(q2 + q3 + q4)*cos(q5);
            0   0   0   0];
U66 = [sin(q2 + q3 + q4)*cos(q1)*cos(q6) - 1.0*sin(q6)*(sin(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)), - 1.0*cos(q6)*(sin(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - 1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q6), 0, 0;
            sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) + sin(q2 + q3 + q4)*cos(q6)*sin(q1),           cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - 1.0*sin(q2 + q3 + q4)*sin(q1)*sin(q6), 0, 0;
            sin(q2 + q3 + q4)*cos(q5)*sin(q6) - cos(q2 + q3 + q4)*cos(q6),                                                   cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6), 0, 0;
            0   0   0   0];


U111 = [-cos(q1),  sin(q1), 0, 0;
              -sin(q1), -cos(q1), 0, 0;
              0,   0,   0,    0;
              0,   0,   0,    0];

U211 = [cos(q1)*sin(q2), cos(q1)*cos(q2), -1.0*sin(q1), 0;
            sin(q1)*sin(q2), cos(q2)*sin(q1),      cos(q1), 0;
            0   0   0   0;
            0   0   0   0];
U212 = [cos(q2)*sin(q1), -1.0*sin(q1)*sin(q2), 0, 0;
            -cos(q1)*cos(q2),      cos(q1)*sin(q2), 0, 0;
            0   0   0   0;
            0   0   0   0];
U221 = [cos(q2)*sin(q1), -1.0*sin(q1)*sin(q2), 0, 0;
            -cos(q1)*cos(q2),      cos(q1)*sin(q2), 0, 0;
            0   0   0   0;
            0   0   0   0];
U222 = [cos(q1)*sin(q2), cos(q1)*cos(q2), 0, 0;
            sin(q1)*sin(q2), cos(q2)*sin(q1), 0, 0;
            -cos(q2),         sin(q2), 0, 0;
            0   0   0   0];

U311 = [sin(q2 + q3)*cos(q1), cos(q2 + q3)*cos(q1), -1.0*sin(q1), 0.426*cos(q1)*sin(q2);
            sin(q2 + q3)*sin(q1), cos(q2 + q3)*sin(q1),      cos(q1), 0.426*sin(q1)*sin(q2);
            0   0   0   0;
            0   0   0   0];
U312 = [cos(q2 + q3)*sin(q1), -1.0*sin(q2 + q3)*sin(q1), 0,  0.426*cos(q2)*sin(q1);
            -cos(q2 + q3)*cos(q1),      sin(q2 + q3)*cos(q1), 0, -0.426*cos(q1)*cos(q2);
            0   0   0   0;
            0   0   0   0];
U313 = [cos(q2 + q3)*sin(q1), -1.0*sin(q2 + q3)*sin(q1), 0, 0;
            -cos(q2 + q3)*cos(q1),      sin(q2 + q3)*cos(q1), 0, 0;
            0   0   0   0;
            0   0   0   0];
U321 = [cos(q2 + q3)*sin(q1), -1.0*sin(q2 + q3)*sin(q1), 0,  0.426*cos(q2)*sin(q1);
            -cos(q2 + q3)*cos(q1),      sin(q2 + q3)*cos(q1), 0, -0.426*cos(q1)*cos(q2);
            0   0   0   0;
            0   0   0   0];
U322 = [sin(q2 + q3)*cos(q1), cos(q2 + q3)*cos(q1), 0, 0.426*cos(q1)*sin(q2);
            sin(q2 + q3)*sin(q1), cos(q2 + q3)*sin(q1), 0, 0.426*sin(q1)*sin(q2);
            -cos(q2 + q3),         sin(q2 + q3), 0,        -0.426*cos(q2);
            0   0   0   0];
U323 = [sin(q2 + q3)*cos(q1), cos(q2 + q3)*cos(q1), 0, 0;
            sin(q2 + q3)*sin(q1), cos(q2 + q3)*sin(q1), 0, 0;
            -cos(q2 + q3),         sin(q2 + q3), 0, 0;
            0   0   0   0];
U331 = [cos(q2 + q3)*sin(q1), -1.0*sin(q2 + q3)*sin(q1), 0, 0;
            -cos(q2 + q3)*cos(q1),      sin(q2 + q3)*cos(q1), 0, 0;
            0   0   0   0;
            0   0   0   0];
U332 = [sin(q2 + q3)*cos(q1), cos(q2 + q3)*cos(q1), 0, 0;
            sin(q2 + q3)*sin(q1), cos(q2 + q3)*sin(q1), 0, 0;
            -cos(q2 + q3),         sin(q2 + q3), 0, 0;
            0   0   0   0];
U333 = [sin(q2 + q3)*cos(q1), cos(q2 + q3)*cos(q1), 0, 0;
            sin(q2 + q3)*sin(q1), cos(q2 + q3)*sin(q1), 0, 0;
            -cos(q2 + q3),         sin(q2 + q3), 0, 0;
            0   0   0   0];

U411 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), -1.0*sin(q1), 0.426*cos(q1)*sin(q2) - 0.1385*sin(q1) + 0.414*cos(q1)*cos(q2)*sin(q3) + 0.414*cos(q1)*cos(q3)*sin(q2);
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1),      cos(q1), 0.1385*cos(q1) + 0.426*sin(q1)*sin(q2) + 0.414*cos(q2)*sin(q1)*sin(q3) + 0.414*cos(q3)*sin(q1)*sin(q2);
            0   0   0   0;
            0   0   0   0];
U412 = [-sin(q2 + q3 + q4)*sin(q1), -cos(q2 + q3 + q4)*sin(q1), 0,  (3*sin(q1)*(69.0*cos(q2 + q3) + 71.0*cos(q2)))/500;
            sin(q2 + q3 + q4)*cos(q1),      cos(q2 + q3 + q4)*cos(q1), 0, -(3*cos(q1)*(69.0*cos(q2 + q3) + 71.0*cos(q2)))/500;
            0   0   0   0;
            0   0   0   0];
U413 = [-sin(q2 + q3 + q4)*sin(q1), -cos(q2 + q3 + q4)*sin(q1), 0,  0.414*cos(q2 + q3)*sin(q1);
            sin(q2 + q3 + q4)*cos(q1),      cos(q2 + q3 + q4)*cos(q1), 0, -0.414*cos(q2 + q3)*cos(q1);
            0   0   0   0;
            0   0   0   0];
U414 = [ -sin(q2 + q3 + q4)*sin(q1), -cos(q2 + q3 + q4)*sin(q1), 0, 0;
            sin(q2 + q3 + q4)*cos(q1),      cos(q2 + q3 + q4)*cos(q1), 0, 0;
                0   0   0   0;
                0   0   0   0];
U421 = [-sin(q2 + q3 + q4)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1), 0,  (3*sin(q1)*(69.0*cos(q2 + q3) + 71.0*cos(q2)))/500;
            sin(q2 + q3 + q4)*cos(q1),      cos(q2 + q3 + q4)*cos(q1), 0, -(3*cos(q1)*(69.0*cos(q2 + q3) + 71.0*cos(q2)))/500;
            0   0   0   0;
            0   0   0   0];
U422 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, (3*cos(q1)*(69.0*sin(q2 + q3) + 71.0*sin(q2)))/500;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, (3*sin(q1)*(69.0*sin(q2 + q3) + 71.0*sin(q2)))/500;
             sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0,               - 0.414*cos(q2 + q3) - 0.426*cos(q2);
             0   0   0   0];
U423 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, (207*sin(q2 + q3)*cos(q1))/500;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, (207*sin(q2 + q3)*sin(q1))/500;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0,            -0.414*cos(q2 + q3);
            0   0   0   0];
U424 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, 0;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0, 0;
            0   0   0   0];
U431 = [-sin(q2 + q3 + q4)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1), 0,  0.414*cos(q2 + q3)*sin(q1);
            sin(q2 + q3 + q4)*cos(q1),      cos(q2 + q3 + q4)*cos(q1), 0, -0.414*cos(q2 + q3)*cos(q1);
            0   0   0   0;
            0   0   0   0];
U432 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, (207*sin(q2 + q3)*cos(q1))/500;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, (207*sin(q2 + q3)*sin(q1))/500;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0,            -0.414*cos(q2 + q3);
            0   0   0   0];
U433 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, (207*sin(q2 + q3)*cos(q1))/500;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, (207*sin(q2 + q3)*sin(q1))/500;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0,            -0.414*cos(q2 + q3);
            0   0   0   0];
U434 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, 0;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0, 0;
            0   0   0   0];
U441 = [-sin(q2 + q3 + q4)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1), 0, 0;
            sin(q2 + q3 + q4)*cos(q1),      cos(q2 + q3 + q4)*cos(q1), 0, 0;
            0   0   0   0;
            0   0   0   0];
U442 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, 0;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0, 0;
            0   0   0   0];
U443 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, 0;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0, 0;
            0   0   0   0];
U444 = [cos(q2 + q3 + q4)*cos(q1), -1.0*sin(q2 + q3 + q4)*cos(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q1), -1.0*sin(q2 + q3 + q4)*sin(q1), 0, 0;
            sin(q2 + q3 + q4),              cos(q2 + q3 + q4), 0, 0;
            0   0   0   0];

U511 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 1.0*sin(q1)*sin(q5), - cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), 0.426*cos(q1)*sin(q2) - 0.1385*sin(q1) + 0.134*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 0.134*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - 1.0*cos(q1)*sin(q2)*sin(q3)) + 0.414*cos(q1)*cos(q2)*sin(q3) + 0.414*cos(q1)*cos(q3)*sin(q2);
            cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1),   cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), 0.1385*cos(q1) + 0.426*sin(q1)*sin(q2) + 0.134*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - 0.134*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - 1.0*cos(q2)*cos(q3)*sin(q1)) + 0.414*cos(q2)*sin(q1)*sin(q3) + 0.414*cos(q3)*sin(q1)*sin(q2);
            0   0   0   0;
            0   0   0   0];
U512 = [-sin(q2 + q3 + q4)*cos(q5)*sin(q1),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),      cos(q2 + q3 + q4)*sin(q1),  0.002*sin(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3) + 213.0*cos(q2));
            sin(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*cos(q2 + q3 + q4)*cos(q1), -0.002*cos(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3) + 213.0*cos(q2));
            0   0   0   0;
            0   0   0   0];
U513 = [-sin(q2 + q3 + q4)*cos(q5)*sin(q1),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),      cos(q2 + q3 + q4)*sin(q1),  0.002*sin(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3));
            sin(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*cos(q2 + q3 + q4)*cos(q1), -0.002*cos(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3));
            0   0   0   0;
            0   0   0   0];
U514 = [-sin(q2 + q3 + q4)*cos(q5)*sin(q1),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),      cos(q2 + q3 + q4)*sin(q1),  0.134*cos(q2 + q3 + q4)*sin(q1);
            sin(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*cos(q2 + q3 + q4)*cos(q1), -0.134*cos(q2 + q3 + q4)*cos(q1);
            0   0   0   0;
            0   0   0   0];
U515 = [cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5),   - cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5), cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 1.0*sin(q1)*sin(q5), 0, 0;
            0   0   0   0;
            0   0   0   0];
U521 = [-sin(q2 + q3 + q4)*cos(q5)*sin(q1),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),      cos(q2 + q3 + q4)*sin(q1),  0.002*sin(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3) + 213.0*cos(q2));
            sin(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*cos(q2 + q3 + q4)*cos(q1), -0.002*cos(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3) + 213.0*cos(q2));
            0   0   0   0;
            0   0   0   0];
U522 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), (cos(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3) + 213.0*sin(q2)))/500;
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), (sin(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3) + 213.0*sin(q2)))/500;
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),              - 0.134*cos(q2 + q3 + q4) - 0.414*cos(q2 + q3) - 0.426*cos(q2);
            0   0   0   0];
U523 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), (cos(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3)))/500;
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), (sin(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3)))/500;
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),              - 0.134*cos(q2 + q3 + q4) - 0.414*cos(q2 + q3);
            0   0   0   0];
U524 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), 0.134*sin(q2 + q3 + q4)*cos(q1);
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), 0.134*sin(q2 + q3 + q4)*sin(q1);
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),        -0.134*cos(q2 + q3 + q4);
            0   0   0   0];
U525 = [-sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), 0, 0;
            -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q5),              cos(q2 + q3 + q4)*cos(q5), 0, 0;
            0   0   0   0];
U531 = [-sin(q2 + q3 + q4)*cos(q5)*sin(q1),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),      cos(q2 + q3 + q4)*sin(q1),  0.002*sin(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3));
            sin(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*cos(q2 + q3 + q4)*cos(q1), -0.002*cos(q1)*(67.0*cos(q2 + q3 + q4) + 207.0*cos(q2 + q3));
            0   0   0   0;
            0   0   0   0];
U532 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), (cos(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3)))/500;
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), (sin(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3)))/500;
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),              - 0.134*cos(q2 + q3 + q4) - 0.414*cos(q2 + q3);
            0   0   0   0];
U533 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), (cos(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3)))/500;
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), (sin(q1)*(67.0*sin(q2 + q3 + q4) + 207.0*sin(q2 + q3)))/500;
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),              - 0.134*cos(q2 + q3 + q4) - 0.414*cos(q2 + q3);
            0   0   0   0];
U534 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), 0.134*sin(q2 + q3 + q4)*cos(q1);
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), 0.134*sin(q2 + q3 + q4)*sin(q1);
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),        -0.134*cos(q2 + q3 + q4);
            0   0   0   0];
U535 = [-sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), 0, 0;
            -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q5),              cos(q2 + q3 + q4)*cos(q5), 0, 0;
            0   0   0   0];
U541 = [-sin(q2 + q3 + q4)*cos(q5)*sin(q1),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),      cos(q2 + q3 + q4)*sin(q1),  0.134*cos(q2 + q3 + q4)*sin(q1);
            sin(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -1.0*cos(q2 + q3 + q4)*cos(q1), -0.134*cos(q2 + q3 + q4)*cos(q1);
            0   0   0   0;
            0   0   0   0];
U542 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), 0.134*sin(q2 + q3 + q4)*cos(q1);
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), 0.134*sin(q2 + q3 + q4)*sin(q1);
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),        -0.134*cos(q2 + q3 + q4);
            0   0   0   0];
U543 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), 0.134*sin(q2 + q3 + q4)*cos(q1);
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), 0.134*sin(q2 + q3 + q4)*sin(q1);
            sin(q2 + q3 + q4)*cos(q5),         -1.0*sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),        -0.134*cos(q2 + q3 + q4);
            0   0   0   0];
U544 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5), -cos(q2 + q3 + q4)*cos(q1)*sin(q5), sin(q2 + q3 + q4)*cos(q1), 0.134*sin(q2 + q3 + q4)*cos(q1);
            cos(q2 + q3 + q4)*cos(q5)*sin(q1), -cos(q2 + q3 + q4)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1), 0.134*sin(q2 + q3 + q4)*sin(q1);
            sin(q2 + q3 + q4)*cos(q5),         -sin(q2 + q3 + q4)*sin(q5),    -1.0*cos(q2 + q3 + q4),        -0.134*cos(q2 + q3 + q4);
            0   0   0   0];
U545 = [-sin(q2 + q3 + q4)*cos(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q1)*cos(q5), 0, 0;
            -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q5),              cos(q2 + q3 + q4)*cos(q5), 0, 0;
            0   0   0   0];
U551 = [cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5),   - cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5), cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 1.0*sin(q1)*sin(q5), 0, 0;
            0   0   0   0;
            0   0   0   0];
U552 = [-sin(q2 + q3 + q4)*cos(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q1)*cos(q5), 0, 0;
            -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q5),              cos(q2 + q3 + q4)*cos(q5), 0, 0;
            0   0   0   0];
U553 = [-sin(q2 + q3 + q4)*cos(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q1)*cos(q5), 0, 0;
            -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q5),              cos(q2 + q3 + q4)*cos(q5), 0, 0;
            0   0   0   0];
U554 = [-sin(q2 + q3 + q4)*cos(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q1)*cos(q5), 0, 0;
            -sin(q2 + q3 + q4)*sin(q1)*sin(q5), -sin(q2 + q3 + q4)*cos(q5)*sin(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q5),              cos(q2 + q3 + q4)*cos(q5), 0, 0;
            0   0   0   0];
U555 = [cos(q2 + q3 + q4)*cos(q1)*cos(q5) - sin(q1)*sin(q5), - cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0, 0;
            cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1),   cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0, 0;
            sin(q2 + q3 + q4)*cos(q5),                        -sin(q2 + q3 + q4)*sin(q5), 0, 0;
            0   0   0   0];
        
U611 = [-cos(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - sin(q2 + q3 + q4)*cos(q1)*sin(q6), sin(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - 1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q6), - cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.426*cos(q1)*sin(q2) - 0.1385*sin(q1) - 0.127*cos(q5)*sin(q1) - 0.127*cos(q2 + q3 + q4)*cos(q1)*sin(q5) + 0.134*cos(q2 + q3)*cos(q1)*sin(q4) + 0.134*sin(q2 + q3)*cos(q1)*cos(q4) + 0.414*cos(q1)*cos(q2)*sin(q3) + 0.414*cos(q1)*cos(q3)*sin(q2);
            cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*sin(q1)*sin(q6),   - sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - 1.0*sin(q2 + q3 + q4)*cos(q6)*sin(q1),   cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.1385*cos(q1) + 0.127*cos(q1)*cos(q5) + 0.426*sin(q1)*sin(q2) - 0.127*cos(q2 + q3 + q4)*sin(q1)*sin(q5) + 0.134*cos(q2 + q3)*sin(q1)*sin(q4) + 0.134*sin(q2 + q3)*cos(q4)*sin(q1) + 0.414*cos(q2)*sin(q1)*sin(q3) + 0.414*cos(q3)*sin(q1)*sin(q2);
            0   0   0   0;
            0   0   0   0];
U612 = [-sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),  (sin(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 426.0*cos(q2) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)),  cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 426.0*cos(q2) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            0   0   0   0;
            0   0   0   0];
U613 = [-sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),  (sin(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)),  cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            0   0   0   0;
            0   0   0   0];
U614 = [-sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),  (sin(q1)*(127.0*sin(q2 + q3 + q4)*sin(q5) + 134.0*cos(q2 + q3)*cos(q4) - 134.0*sin(q2 + q3)*sin(q4)))/1000;
            cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)),  cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(127.0*sin(q2 + q3 + q4)*sin(q5) + 134.0*cos(q2 + q3)*cos(q4) - 134.0*sin(q2 + q3)*sin(q4)))/1000;
            0   0   0   0;
            0   0   0   0];
U615 = [cos(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)), -sin(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)),   - cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1), - 0.127*cos(q1)*sin(q5) - 0.127*cos(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -sin(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 1.0*sin(q1)*sin(q5),   0.127*cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 0.127*sin(q1)*sin(q5);
            0   0   0   0;
            0   0   0   0];
U616 = [-sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - sin(q2 + q3 + q4)*cos(q6)*sin(q1),           sin(q2 + q3 + q4)*sin(q1)*sin(q6) - 1.0*cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)), 0, 0;
            sin(q2 + q3 + q4)*cos(q1)*cos(q6) - 1.0*sin(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)), - 1.0*cos(q6)*(sin(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - 1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q6), 0, 0;
            0   0   0   0;
            0   0   0   0];
U621 = [-sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),  (sin(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 426.0*cos(q2) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)),  cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 426.0*cos(q2) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            0   0   0   0;
            0   0   0   0];
U622 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5),                             0.001*cos(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 426.0*sin(q2) + 63.5*sin(q2 + q3 + q4 - q5));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5),                             0.001*sin(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 426.0*sin(q2) + 63.5*sin(q2 + q3 + q4 - q5));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5), 0.134*sin(q2 + q3)*sin(q4) - 0.426*cos(q2) - 0.134*cos(q2 + q3)*cos(q4) - 1.0*sin(q5)*(0.127*cos(q2 + q3)*sin(q4) + 0.127*sin(q2 + q3)*cos(q4)) - 0.414*cos(q2 + q3);
           0   0   0   0];
U623 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 63.5*sin(q2 + q3 + q4 - q5));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 63.5*sin(q2 + q3 + q4 - q5));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                         - 0.134*cos(q2 + q3 + q4) - 0.414*cos(q2 + q3) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U624 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                               - 0.134*cos(q2 + q3 + q4) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U625 = [-sin(q2 + q3 + q4)*cos(q1)*cos(q6)*sin(q5), sin(q2 + q3 + q4)*cos(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), -0.127*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
            -sin(q2 + q3 + q4)*cos(q6)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), -0.127*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q2 + q3 + q4)*cos(q6)*sin(q5),    -1.0*cos(q2 + q3 + q4)*sin(q5)*sin(q6),              cos(q2 + q3 + q4)*cos(q5),          0.127*cos(q2 + q3 + q4)*cos(q5);
            0   0   0   0];
U626 = [cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),        cos(q2 + q3 + q4)*cos(q5)*cos(q6) - 1.0*sin(q2 + q3 + q4)*sin(q6), 0, 0;
            0   0   0   0];
U631 = [-sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),  (sin(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)),  cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(134.0*cos(q2 + q3 + q4) - 63.5*cos(q2 + q3 + q4 + q5) + 414.0*cos(q2 + q3) + 63.5*cos(q2 + q3 + q4 - q5)))/1000;
            0   0   0   0;
            0   0   0   0];
U632 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 63.5*sin(q2 + q3 + q4 - q5));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 63.5*sin(q2 + q3 + q4 - q5));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                         - 0.134*cos(q2 + q3 + q4) - 0.414*cos(q2 + q3) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U633 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 63.5*sin(q2 + q3 + q4 - q5));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*sin(q2 + q3 + q4) - 63.5*sin(q2 + q3 + q4 + q5) + 414.0*sin(q2 + q3) + 63.5*sin(q2 + q3 + q4 - q5));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                         - 0.134*cos(q2 + q3 + q4) - 0.414*cos(q2 + q3) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U634 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                               - 0.134*cos(q2 + q3 + q4) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U635 = [-sin(q2 + q3 + q4)*cos(q1)*cos(q6)*sin(q5), sin(q2 + q3 + q4)*cos(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), -0.127*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
            -sin(q2 + q3 + q4)*cos(q6)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), -0.127*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q2 + q3 + q4)*cos(q6)*sin(q5),    -1.0*cos(q2 + q3 + q4)*sin(q5)*sin(q6),              cos(q2 + q3 + q4)*cos(q5),          0.127*cos(q2 + q3 + q4)*cos(q5);
            0   0   0   0];
U636 = [cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),        cos(q2 + q3 + q4)*cos(q5)*cos(q6) - 1.0*sin(q2 + q3 + q4)*sin(q6), 0, 0;
            0   0   0   0];
U641 = [-sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)),      sin(q2 + q3 + q4)*sin(q1)*sin(q5),  (sin(q1)*(127.0*sin(q2 + q3 + q4)*sin(q5) + 134.0*cos(q2 + q3)*cos(q4) - 134.0*sin(q2 + q3)*sin(q4)))/1000;
            cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)),  cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q5), -(cos(q1)*(127.0*sin(q2 + q3 + q4)*sin(q5) + 134.0*cos(q2 + q3)*cos(q4) - 134.0*sin(q2 + q3)*sin(q4)))/1000;
            0   0   0   0;
            0   0   0   0];
U642 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                               - 0.134*cos(q2 + q3 + q4) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U643 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                               - 0.134*cos(q2 + q3 + q4) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U644 = [-cos(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -cos(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*cos(q1)*sin(q5), 0.001*cos(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            -sin(q1)*(sin(q2 + q3 + q4)*sin(q6) - cos(q2 + q3 + q4)*cos(q5)*cos(q6)), -sin(q1)*(sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6)), -1.0*cos(q2 + q3 + q4)*sin(q1)*sin(q5), 0.001*sin(q1)*(134.0*cos(q2 + q3)*sin(q4) - 127.0*cos(q2 + q3 + q4)*sin(q5) + 134.0*sin(q2 + q3)*cos(q4));
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),        cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),         -1.0*sin(q2 + q3 + q4)*sin(q5),                                               - 0.134*cos(q2 + q3 + q4) - 0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U645 = [-sin(q2 + q3 + q4)*cos(q1)*cos(q6)*sin(q5), sin(q2 + q3 + q4)*cos(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), -0.127*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
            -sin(q2 + q3 + q4)*cos(q6)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), -0.127*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q2 + q3 + q4)*cos(q6)*sin(q5),    -1.0*cos(q2 + q3 + q4)*sin(q5)*sin(q6),              cos(q2 + q3 + q4)*cos(q5),          0.127*cos(q2 + q3 + q4)*cos(q5);
            0   0   0   0];
U646 = [cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),        cos(q2 + q3 + q4)*cos(q5)*cos(q6) - 1.0*sin(q2 + q3 + q4)*sin(q6), 0, 0;
            0   0   0   0];
U651 = [cos(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)), -1.0*sin(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)),   - cos(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q5)*sin(q1), - 0.127*cos(q1)*sin(q5) - 0.127*cos(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -1.0*sin(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 1.0*sin(q1)*sin(q5),   0.127*cos(q2 + q3 + q4)*cos(q1)*cos(q5) - 0.127*sin(q1)*sin(q5);
            0   0   0   0;
            0   0   0   0];
U652 = [-sin(q2 + q3 + q4)*cos(q1)*cos(q6)*sin(q5), sin(q2 + q3 + q4)*cos(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), -0.127*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
            -sin(q2 + q3 + q4)*cos(q6)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), -0.127*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q2 + q3 + q4)*cos(q6)*sin(q5),    -1.0*cos(q2 + q3 + q4)*sin(q5)*sin(q6),              cos(q2 + q3 + q4)*cos(q5),          0.127*cos(q2 + q3 + q4)*cos(q5);
            0   0   0   0];
U653 = [-sin(q2 + q3 + q4)*cos(q1)*cos(q6)*sin(q5), sin(q2 + q3 + q4)*cos(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), -0.127*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
            -sin(q2 + q3 + q4)*cos(q6)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), -0.127*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q2 + q3 + q4)*cos(q6)*sin(q5),    -1.0*cos(q2 + q3 + q4)*sin(q5)*sin(q6),              cos(q2 + q3 + q4)*cos(q5),          0.127*cos(q2 + q3 + q4)*cos(q5);
            0   0   0   0];
U654 = [-sin(q2 + q3 + q4)*cos(q1)*cos(q6)*sin(q5), sin(q2 + q3 + q4)*cos(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q5), -0.127*sin(q2 + q3 + q4)*cos(q1)*cos(q5);
            -sin(q2 + q3 + q4)*cos(q6)*sin(q1)*sin(q5), sin(q2 + q3 + q4)*sin(q1)*sin(q5)*sin(q6), -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q1), -0.127*sin(q2 + q3 + q4)*cos(q5)*sin(q1);
            cos(q2 + q3 + q4)*cos(q6)*sin(q5),    -1.0*cos(q2 + q3 + q4)*sin(q5)*sin(q6),              cos(q2 + q3 + q4)*cos(q5),          0.127*cos(q2 + q3 + q4)*cos(q5);
            0   0   0   0];
U655 = [-cos(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)),      sin(q6)*(sin(q1)*sin(q5) - cos(q2 + q3 + q4)*cos(q1)*cos(q5)), - cos(q5)*sin(q1) - cos(q2 + q3 + q4)*cos(q1)*sin(q5), - 0.127*cos(q5)*sin(q1) - 0.127*cos(q2 + q3 + q4)*cos(q1)*sin(q5);
            cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)), -1.0*sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)),   cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5),   0.127*cos(q1)*cos(q5) - 0.127*cos(q2 + q3 + q4)*sin(q1)*sin(q5);
            sin(q2 + q3 + q4)*cos(q5)*cos(q6),                             -1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6),                        -1.0*sin(q2 + q3 + q4)*sin(q5),                                  -0.127*sin(q2 + q3 + q4)*sin(q5);
            0   0   0   0];
U656 = [-sin(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -1.0*cos(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), 0, 0;
             sin(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)),      cos(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)), 0, 0;
             -sin(q2 + q3 + q4)*sin(q5)*sin(q6),                             -1.0*sin(q2 + q3 + q4)*cos(q6)*sin(q5), 0, 0;
             0   0   0   0];
U661 = [-sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - 1.0*sin(q2 + q3 + q4)*cos(q6)*sin(q1),           sin(q2 + q3 + q4)*sin(q1)*sin(q6) - 1.0*cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)), 0, 0;
            sin(q2 + q3 + q4)*cos(q1)*cos(q6) - 1.0*sin(q6)*(sin(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)), - 1.0*cos(q6)*(sin(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - 1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q6), 0, 0;
            0   0   0   0;
            0   0   0   0];
U662 = [cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),        cos(q2 + q3 + q4)*cos(q5)*cos(q6) - 1.0*sin(q2 + q3 + q4)*sin(q6), 0, 0;
            0   0   0   0];
U663 = [cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),        cos(q2 + q3 + q4)*cos(q5)*cos(q6) - 1.0*sin(q2 + q3 + q4)*sin(q6), 0, 0;
            0   0   0   0];
U664 = [cos(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -cos(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q1)*(cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6)), -sin(q1)*(cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6)), 0, 0;
            sin(q2 + q3 + q4)*cos(q6) + cos(q2 + q3 + q4)*cos(q5)*sin(q6),        cos(q2 + q3 + q4)*cos(q5)*cos(q6) - 1.0*sin(q2 + q3 + q4)*sin(q6), 0, 0;
            0   0   0   0];
U665 = [-sin(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), -1.0*cos(q6)*(cos(q5)*sin(q1) + cos(q2 + q3 + q4)*cos(q1)*sin(q5)), 0, 0;
            sin(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)),      cos(q6)*(cos(q1)*cos(q5) - cos(q2 + q3 + q4)*sin(q1)*sin(q5)), 0, 0;
            -sin(q2 + q3 + q4)*sin(q5)*sin(q6),                             -1.0*sin(q2 + q3 + q4)*cos(q6)*sin(q5), 0, 0;
            0   0   0   0];
U666 = [-cos(q6)*(sin(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - 1.0*sin(q2 + q3 + q4)*cos(q1)*sin(q6), 1.0*sin(q6)*(sin(q1)*sin(q5) - 1.0*cos(q2 + q3 + q4)*cos(q1)*cos(q5)) - 1.0*sin(q2 + q3 + q4)*cos(q1)*cos(q6), 0, 0;
            cos(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - 1.0*sin(q2 + q3 + q4)*sin(q1)*sin(q6),   - 1.0*sin(q6)*(cos(q1)*sin(q5) + cos(q2 + q3 + q4)*cos(q5)*sin(q1)) - 1.0*sin(q2 + q3 + q4)*cos(q6)*sin(q1), 0, 0;
            cos(q2 + q3 + q4)*sin(q6) + sin(q2 + q3 + q4)*cos(q5)*cos(q6),                                             cos(q2 + q3 + q4)*cos(q6) - 1.0*sin(q2 + q3 + q4)*cos(q5)*sin(q6), 0, 0;
            0   0   0   0];

M11 = trace(U11*J1*U11.')+trace(U21*J2*U21.')+trace(U31*J3*U31.')+trace(U41*J4*U41.')+trace(U51*J5*U51.')+trace(U61*J6*U61.');
M12 = trace(U21*J2*U22.')+trace(U31*J3*U32.')+trace(U41*J4*U42.')+trace(U51*J5*U52.')+trace(U61*J6*U62.');
M13 = trace(U31*J3*U33.')+trace(U41*J4*U43.')+trace(U51*J5*U53.')+trace(U61*J6*U63.');
M14 = trace(U41*J4*U44.')+trace(U51*J5*U54.')+trace(U61*J6*U64.');
M15 = trace(U51*J5*U55.')+trace(U61*J6*U65.');
M16 = trace(U61*J6*U66.');

M22 = trace(U22*J2*U22.')+trace(U32*J3*U32.')+trace(U42*J4*U42.')+trace(U52*J5*U52.')+trace(U62*J6*U62.');
M23 = trace(U32*J3*U33.')+trace(U42*J4*U43.')+trace(U52*J5*U53.')+trace(U62*J6*U63.');
M24 = trace(U42*J4*U44.')+trace(U52*J5*U54.')+trace(U62*J6*U64.');
M25 = trace(U52*J5*U55.')+trace(U62*J6*U65.');
M26 = trace(U62*J6*U66.');

M33 = trace(U33*J3*U33.')+trace(U43*J4*U43.')+trace(U53*J5*U53.')+trace(U63*J6*U63.');
M34 = trace(U43*J4*U44.')+trace(U53*J5*U54.')+trace(U63*J6*U64.');
M35 = trace(U53*J5*U55.')+trace(U63*J6*U65.');
M36 = trace(U63*J6*U66.');

M44 = trace(U44*J4*U44.')+trace(U54*J5*U54.')+trace(U64*J6*U64.');
M45 = trace(U54*J5*U55.')+trace(U64*J6*U65.');
M46 = trace(U64*J6*U66.');

M55 = trace(U55*J5*U55.')+trace(U65*J6*U65.');
M56 = trace(U65*J6*U66.');

M66 = trace(U66*J6*U66.');

%对称正定矩阵M
M21 = M12; M31 = M13; M41 = M14; M51 = M15; M61 = M16; 
M32 = M23; M42 = M24; M52 = M25; M62 = M26; 
M43 = M34; M53 = M35; M63 = M36; 
M54 = M45; M64 = M46; 
M65 = M56; 

%惯性矩阵
M = [M11, M12, M13, M14, M15, M16;
         M21, M22, M23, M24, M25, M26;
         M31, M32, M33, M34, M35, M36;
         M41, M42, M43, M44, M45, M46;
         M51, M52, M53, M54, M55, M56;
         M61, M62, M63, M64, M65, M66];

     
%理论法
C111 = trace(U111*J1*U11.')+trace(U211*J2*U21.')+trace(U311*J3*U31.')+trace(U411*J4*U41.')+trace(U511*J5*U51.')+trace(U611*J6*U61.');
C112 = trace(U212*J2*U21.')+trace(U312*J3*U31.')+trace(U412*J4*U41.')+trace(U512*J5*U51.')+trace(U612*J6*U61.');
C113 = trace(U313*J3*U31.')+trace(U413*J4*U41.')+trace(U513*J5*U51.')+trace(U613*J6*U61.');
C114 = trace(U414*J4*U41.')+trace(U514*J5*U51.')+trace(U614*J6*U61.');
C115 = trace(U515*J5*U51.')+trace(U615*J6*U61.');
C116 = trace(U616*J6*U61.');

C121 = trace(U221*J2*U21.')+trace(U321*J3*U31.')+trace(U421*J4*U41.')+trace(U521*J5*U51.')+trace(U621*J6*U61.');
C122 = trace(U222*J2*U21.')+trace(U322*J3*U31.')+trace(U422*J4*U41.')+trace(U522*J5*U51.')+trace(U622*J6*U61.');
C123 = trace(U323*J3*U31.')+trace(U423*J4*U41.')+trace(U523*J5*U51.')+trace(U623*J6*U61.');
C124 = trace(U424*J4*U41.')+trace(U524*J5*U51.')+trace(U624*J6*U61.');
C125 = trace(U525*J5*U51.')+trace(U625*J6*U61.');
C126 = trace(U626*J6*U61.');

C131 = trace(U331*J3*U31.')+trace(U431*J4*U41.')+trace(U531*J5*U51.')+trace(U631*J6*U61.');
C132 = trace(U332*J3*U31.')+trace(U432*J4*U41.')+trace(U532*J5*U51.')+trace(U632*J6*U61.');
C133 = trace(U333*J3*U31.')+trace(U433*J4*U41.')+trace(U533*J5*U51.')+trace(U633*J6*U61.');
C134 = trace(U434*J4*U41.')+trace(U534*J5*U51.')+trace(U634*J6*U61.');
C135 = trace(U535*J5*U51.')+trace(U635*J6*U61.');
C136 = trace(U636*J6*U61.');

C141 = trace(U441*J4*U41.')+trace(U541*J5*U51.')+trace(U641*J6*U61.');
C142 = trace(U442*J4*U41.')+trace(U542*J5*U51.')+trace(U642*J6*U61.');
C143 = trace(U443*J4*U41.')+trace(U543*J5*U51.')+trace(U643*J6*U61.');
C144 = trace(U444*J4*U41.')+trace(U544*J5*U51.')+trace(U644*J6*U61.');
C145 = trace(U545*J5*U51.')+trace(U645*J6*U61.');
C146 = trace(U646*J6*U61.');

C151 = trace(U551*J5*U51.')+trace(U651*J6*U61.');
C152 = trace(U552*J5*U51.')+trace(U652*J6*U61.');
C153 = trace(U553*J5*U51.')+trace(U653*J6*U61.');
C154 = trace(U554*J5*U51.')+trace(U654*J6*U61.');
C155 = trace(U555*J5*U51.')+trace(U655*J6*U61.');
C156 = trace(U656*J6*U61.');

C161 = trace(U661*J6*U61.');
C162 = trace(U662*J6*U61.');
C163 = trace(U663*J6*U61.');
C164 = trace(U664*J6*U61.');
C165 = trace(U665*J6*U61.');
C166 = trace(U666*J6*U61.');

C211 = trace(U211*J2*U22.')+trace(U311*J3*U32.')+trace(U411*J4*U42.')+trace(U511*J5*U52.')+trace(U611*J6*U62.');
C212 = trace(U212*J2*U22.')+trace(U312*J3*U32.')+trace(U412*J4*U42.')+trace(U512*J5*U52.')+trace(U612*J6*U62.');
C213 = trace(U313*J3*U32.')+trace(U413*J4*U42.')+trace(U513*J5*U52.')+trace(U613*J6*U62.');
C214 = trace(U414*J4*U42.')+trace(U514*J5*U52.')+trace(U614*J6*U62.');
C215 = trace(U515*J5*U52.')+trace(U615*J6*U62.');
C216 = trace(U616*J6*U62.');

C221 = trace(U221*J2*U22.')+trace(U321*J3*U32.')+trace(U421*J4*U42.')+trace(U521*J5*U52.')+trace(U621*J6*U62.');
C222 = trace(U222*J2*U22.')+trace(U322*J3*U32.')+trace(U422*J4*U42.')+trace(U522*J5*U52.')+trace(U622*J6*U62.');
C223 = trace(U323*J3*U32.')+trace(U423*J4*U42.')+trace(U523*J5*U52.')+trace(U623*J6*U62.');
C224 = trace(U424*J4*U42.')+trace(U524*J5*U52.')+trace(U624*J6*U62.');
C225 = trace(U525*J5*U52.')+trace(U625*J6*U62.');
C226 = trace(U626*J6*U62.');

C231 = trace(U331*J3*U32.')+trace(U431*J4*U42.')+trace(U531*J5*U52.')+trace(U631*J6*U62.');
C232 = trace(U332*J3*U32.')+trace(U432*J4*U42.')+trace(U532*J5*U52.')+trace(U632*J6*U62.');
C233 = trace(U333*J3*U32.')+trace(U433*J4*U42.')+trace(U533*J5*U52.')+trace(U633*J6*U62.');
C234 = trace(U434*J4*U42.')+trace(U534*J5*U52.')+trace(U634*J6*U62.');
C235 = trace(U535*J5*U52.')+trace(U635*J6*U62.');
C236 = trace(U636*J6*U62.');

C241 = trace(U441*J4*U42.')+trace(U541*J5*U52.')+trace(U641*J6*U62.');
C242 = trace(U442*J4*U42.')+trace(U542*J5*U52.')+trace(U642*J6*U62.');
C243 = trace(U443*J4*U42.')+trace(U543*J5*U52.')+trace(U643*J6*U62.');
C244 = trace(U444*J4*U42.')+trace(U544*J5*U52.')+trace(U644*J6*U62.');
C245 = trace(U545*J5*U52.')+trace(U645*J6*U62.');
C246 = trace(U646*J6*U62.');

C251 = trace(U551*J5*U52.')+trace(U651*J6*U62.');
C252 = trace(U552*J5*U52.')+trace(U652*J6*U62.');
C253 = trace(U553*J5*U52.')+trace(U653*J6*U62.');
C254 = trace(U554*J5*U52.')+trace(U654*J6*U62.');
C255 = trace(U555*J5*U52.')+trace(U655*J6*U62.');
C256 = trace(U656*J6*U62.');

C261 = trace(U661*J6*U62.');
C262 = trace(U662*J6*U62.');
C263 = trace(U663*J6*U62.');
C264 = trace(U664*J6*U62.');
C265 = trace(U665*J6*U62.');
C266 = trace(U666*J6*U62.');

C311 = trace(U311*J3*U33.')+trace(U411*J4*U43.')+trace(U511*J5*U53.')+trace(U611*J6*U63.');
C312 = trace(U312*J3*U33.')+trace(U412*J4*U43.')+trace(U512*J5*U53.')+trace(U612*J6*U63.');
C313 = trace(U313*J3*U33.')+trace(U413*J4*U43.')+trace(U513*J5*U53.')+trace(U613*J6*U63.');
C314 = trace(U414*J4*U43.')+trace(U514*J5*U53.')+trace(U614*J6*U63.');
C315 = trace(U515*J5*U53.')+trace(U615*J6*U63.');
C316 = trace(U616*J6*U63.');

C321 = trace(U321*J3*U33.')+trace(U421*J4*U43.')+trace(U521*J5*U53.')+trace(U621*J6*U63.');
C322 = trace(U322*J3*U33.')+trace(U422*J4*U43.')+trace(U522*J5*U53.')+trace(U622*J6*U63.');
C323 = trace(U323*J3*U33.')+trace(U423*J4*U43.')+trace(U523*J5*U53.')+trace(U623*J6*U63.');
C324 = trace(U424*J4*U43.')+trace(U524*J5*U53.')+trace(U624*J6*U63.');
C325 = trace(U525*J5*U53.')+trace(U625*J6*U63.');
C326 = trace(U626*J6*U63.');

C331 = trace(U331*J3*U33.')+trace(U431*J4*U43.')+trace(U531*J5*U53.')+trace(U631*J6*U63.');
C332 = trace(U332*J3*U33.')+trace(U432*J4*U43.')+trace(U532*J5*U53.')+trace(U632*J6*U63.');
C333 = trace(U333*J3*U33.')+trace(U433*J4*U43.')+trace(U533*J5*U53.')+trace(U633*J6*U63.');
C334 = trace(U434*J4*U43.')+trace(U534*J5*U53.')+trace(U634*J6*U63.');
C335 = trace(U535*J5*U53.')+trace(U635*J6*U63.');
C336 = trace(U636*J6*U63.');

C341 = trace(U441*J4*U43.')+trace(U541*J5*U53.')+trace(U641*J6*U63.');
C342 = trace(U442*J4*U43.')+trace(U542*J5*U53.')+trace(U642*J6*U63.');
C343 = trace(U443*J4*U43.')+trace(U543*J5*U53.')+trace(U643*J6*U63.');
C344 = trace(U444*J4*U43.')+trace(U544*J5*U53.')+trace(U644*J6*U63.');
C345 = trace(U545*J5*U53.')+trace(U645*J6*U63.');
C346 = trace(U646*J6*U63.');

C351 = trace(U551*J5*U53.')+trace(U651*J6*U63.');
C352 = trace(U552*J5*U53.')+trace(U652*J6*U63.');
C353 = trace(U553*J5*U53.')+trace(U653*J6*U63.');
C354 = trace(U554*J5*U53.')+trace(U654*J6*U63.');
C355 = trace(U555*J5*U53.')+trace(U655*J6*U63.');
C356 = trace(U656*J6*U63.');

C361 = trace(U661*J6*U63.');
C362 = trace(U662*J6*U63.');
C363 = trace(U663*J6*U63.');
C364 = trace(U664*J6*U63.');
C365 = trace(U665*J6*U63.');
C366 = trace(U666*J6*U63.');

C411 = trace(U411*J4*U44.')+trace(U511*J5*U54.')+trace(U611*J6*U64.');
C412 = trace(U412*J4*U44.')+trace(U512*J5*U54.')+trace(U612*J6*U64.');
C413 = trace(U413*J4*U44.')+trace(U513*J5*U54.')+trace(U613*J6*U64.');
C414 = trace(U414*J4*U44.')+trace(U514*J5*U54.')+trace(U614*J6*U64.');
C415 = trace(U515*J5*U54.')+trace(U615*J6*U64.');
C416 = trace(U616*J6*U64.');

C421 = trace(U421*J4*U44.')+trace(U521*J5*U54.')+trace(U621*J6*U64.');
C422 = trace(U422*J4*U44.')+trace(U522*J5*U54.')+trace(U622*J6*U64.');
C423 = trace(U423*J4*U44.')+trace(U523*J5*U54.')+trace(U623*J6*U64.');
C424 = trace(U424*J4*U44.')+trace(U524*J5*U54.')+trace(U624*J6*U64.');
C425 = trace(U525*J5*U54.')+trace(U625*J6*U64.');
C426 = trace(U626*J6*U64.');

C431 = trace(U431*J4*U44.')+trace(U531*J5*U54.')+trace(U631*J6*U64.');
C432 = trace(U432*J4*U44.')+trace(U532*J5*U54.')+trace(U632*J6*U64.');
C433 = trace(U433*J4*U44.')+trace(U533*J5*U54.')+trace(U633*J6*U64.');
C434 = trace(U434*J4*U44.')+trace(U534*J5*U54.')+trace(U634*J6*U64.');
C435 = trace(U535*J5*U54.')+trace(U635*J6*U64.');
C436 = trace(U636*J6*U64.');

C441 = trace(U441*J4*U44.')+trace(U541*J5*U54.')+trace(U641*J6*U64.');
C442 = trace(U442*J4*U44.')+trace(U542*J5*U54.')+trace(U642*J6*U64.');
C443 = trace(U443*J4*U44.')+trace(U543*J5*U54.')+trace(U643*J6*U64.');
C444 = trace(U444*J4*U44.')+trace(U544*J5*U54.')+trace(U644*J6*U64.');
C445 = trace(U545*J5*U54.')+trace(U645*J6*U64.');
C446 = trace(U646*J6*U64.');

C451 = trace(U551*J5*U54.')+trace(U651*J6*U64.');
C452 = trace(U552*J5*U54.')+trace(U652*J6*U64.');
C453 = trace(U553*J5*U54.')+trace(U653*J6*U64.');
C454 = trace(U554*J5*U54.')+trace(U654*J6*U64.');
C455 = trace(U555*J5*U54.')+trace(U655*J6*U64.');
C456 = trace(U656*J6*U64.');

C461 = trace(U661*J6*U64.');
C462 = trace(U662*J6*U64.');
C463 = trace(U663*J6*U64.');
C464 = trace(U664*J6*U64.');
C465 = trace(U665*J6*U64.');
C466 = trace(U666*J6*U64.');

C511 = trace(U511*J5*U55.')+trace(U611*J6*U65.');
C512 = trace(U512*J5*U55.')+trace(U612*J6*U65.');
C513 = trace(U513*J5*U55.')+trace(U613*J6*U65.');
C514 = trace(U514*J5*U55.')+trace(U614*J6*U65.');
C515 = trace(U515*J5*U55.')+trace(U615*J6*U65.');
C516 = trace(U616*J6*U65.');

C521 = trace(U521*J5*U55.')+trace(U621*J6*U65.');
C522 = trace(U522*J5*U55.')+trace(U622*J6*U65.');
C523 = trace(U523*J5*U55.')+trace(U623*J6*U65.');
C524 = trace(U524*J5*U55.')+trace(U624*J6*U65.');
C525 = trace(U525*J5*U55.')+trace(U625*J6*U65.');
C526 = trace(U626*J6*U65.');

C531 = trace(U531*J5*U55.')+trace(U631*J6*U65.');
C532 = trace(U532*J5*U55.')+trace(U632*J6*U65.');
C533 = trace(U533*J5*U55.')+trace(U633*J6*U65.');
C534 = trace(U534*J5*U55.')+trace(U634*J6*U65.');
C535 = trace(U535*J5*U55.')+trace(U635*J6*U65.');
C536 = trace(U636*J6*U65.');

C541 = trace(U541*J5*U55.')+trace(U641*J6*U65.');
C542 = trace(U542*J5*U55.')+trace(U642*J6*U65.');
C543 = trace(U543*J5*U55.')+trace(U643*J6*U65.');
C544 = trace(U544*J5*U55.')+trace(U644*J6*U65.');
C545 = trace(U545*J5*U55.')+trace(U645*J6*U65.');
C546 = trace(U646*J6*U65.');

C551 = trace(U551*J5*U55.')+trace(U651*J6*U65.');
C552 = trace(U552*J5*U55.')+trace(U652*J6*U65.');
C553 = trace(U553*J5*U55.')+trace(U653*J6*U65.');
C554 = trace(U554*J5*U55.')+trace(U654*J6*U65.');
C555 = trace(U555*J5*U55.')+trace(U655*J6*U65.');
C556 = trace(U656*J6*U65.');

C561 = trace(U661*J6*U65.');
C562 = trace(U662*J6*U65.');
C563 = trace(U663*J6*U65.');
C564 = trace(U664*J6*U65.');
C565 = trace(U665*J6*U65.');
C566 = trace(U666*J6*U65.');

C611 = trace(U611*J6*U66.');
C612 = trace(U612*J6*U66.');
C613 = trace(U613*J6*U66.');
C614 = trace(U614*J6*U66.');
C615 = trace(U615*J6*U66.');
C616 = trace(U616*J6*U66.');

C621 = trace(U621*J6*U66.');
C622 = trace(U622*J6*U66.');
C623 = trace(U623*J6*U66.');
C624 = trace(U624*J6*U66.');
C625 = trace(U625*J6*U66.');
C626 = trace(U626*J6*U66.');

C631 = trace(U631*J6*U66.');
C632 = trace(U632*J6*U66.');
C633 = trace(U633*J6*U66.');
C634 = trace(U634*J6*U66.');
C635 = trace(U635*J6*U66.');
C636 = trace(U636*J6*U66.');

C641 = trace(U641*J6*U66.');
C642 = trace(U642*J6*U66.');
C643 = trace(U643*J6*U66.');
C644 = trace(U644*J6*U66.');
C645 = trace(U645*J6*U66.');
C646 = trace(U646*J6*U66.');

C651 = trace(U651*J6*U66.');
C652 = trace(U652*J6*U66.');
C653 = trace(U653*J6*U66.');
C654 = trace(U654*J6*U66.');
C655 = trace(U655*J6*U66.');
C656 = trace(U656*J6*U66.');

C661 = trace(U661*J6*U66.');
C662 = trace(U662*J6*U66.');
C663 = trace(U663*J6*U66.');
C664 = trace(U664*J6*U66.');
C665 = trace(U665*J6*U66.');
C666 = trace(U666*J6*U66.');

C11 = C111*dq1+C112*dq2+C113*dq3+C114*dq4+C115*dq5+C116*dq6;
C12 = C121*dq1+C122*dq2+C123*dq3+C124*dq4+C125*dq5+C126*dq6;
C13 = C131*dq1+C132*dq2+C133*dq3+C134*dq4+C135*dq5+C136*dq6;
C14 = C141*dq1+C142*dq2+C143*dq3+C144*dq4+C145*dq5+C146*dq6;
C15 = C151*dq1+C152*dq2+C153*dq3+C154*dq4+C155*dq5+C156*dq6;
C16 = C161*dq1+C162*dq2+C163*dq3+C164*dq4+C165*dq5+C166*dq6;

C21 = C211*dq1+C212*dq2+C213*dq3+C214*dq4+C215*dq5+C216*dq6;
C22 = C221*dq1+C222*dq2+C223*dq3+C224*dq4+C225*dq5+C226*dq6;
C23 = C231*dq1+C232*dq2+C233*dq3+C234*dq4+C235*dq5+C236*dq6;
C24 = C241*dq1+C242*dq2+C243*dq3+C244*dq4+C245*dq5+C246*dq6;
C25 = C251*dq1+C252*dq2+C253*dq3+C254*dq4+C255*dq5+C256*dq6;
C26 = C261*dq1+C262*dq2+C263*dq3+C264*dq4+C265*dq5+C266*dq6;

C31 = C311*dq1+C312*dq2+C313*dq3+C314*dq4+C315*dq5+C316*dq6;
C32 = C321*dq1+C322*dq2+C323*dq3+C324*dq4+C325*dq5+C326*dq6;
C33 = C331*dq1+C332*dq2+C333*dq3+C334*dq4+C335*dq5+C336*dq6;
C34 = C341*dq1+C342*dq2+C343*dq3+C344*dq4+C345*dq5+C346*dq6;
C35 = C351*dq1+C352*dq2+C353*dq3+C354*dq4+C355*dq5+C356*dq6;
C36 = C361*dq1+C362*dq2+C363*dq3+C364*dq4+C365*dq5+C366*dq6;

C41 = C411*dq1+C412*dq2+C413*dq3+C414*dq4+C415*dq5+C416*dq6;
C42 = C421*dq1+C422*dq2+C423*dq3+C424*dq4+C425*dq5+C426*dq6;
C43 = C431*dq1+C432*dq2+C433*dq3+C434*dq4+C435*dq5+C436*dq6;
C44 = C441*dq1+C442*dq2+C443*dq3+C444*dq4+C445*dq5+C446*dq6;
C45 = C451*dq1+C452*dq2+C453*dq3+C454*dq4+C455*dq5+C456*dq6;
C46 = C461*dq1+C462*dq2+C463*dq3+C464*dq4+C465*dq5+C466*dq6;

C51 = C511*dq1+C512*dq2+C513*dq3+C514*dq4+C515*dq5+C516*dq6;
C52 = C521*dq1+C522*dq2+C523*dq3+C524*dq4+C525*dq5+C526*dq6;
C53 = C531*dq1+C532*dq2+C533*dq3+C534*dq4+C535*dq5+C536*dq6;
C54 = C541*dq1+C542*dq2+C543*dq3+C544*dq4+C545*dq5+C546*dq6;
C55 = C551*dq1+C552*dq2+C553*dq3+C554*dq4+C555*dq5+C556*dq6;
C56 = C561*dq1+C562*dq2+C563*dq3+C564*dq4+C565*dq5+C566*dq6;

C61 = C611*dq1+C612*dq2+C613*dq3+C614*dq4+C615*dq5+C616*dq6;
C62 = C621*dq1+C622*dq2+C623*dq3+C624*dq4+C625*dq5+C626*dq6;
C63 = C631*dq1+C632*dq2+C633*dq3+C634*dq4+C635*dq5+C636*dq6;
C64 = C641*dq1+C642*dq2+C643*dq3+C644*dq4+C645*dq5+C646*dq6;
C65 = C651*dq1+C652*dq2+C653*dq3+C654*dq4+C655*dq5+C656*dq6;
C66 = C661*dq1+C662*dq2+C663*dq3+C664*dq4+C665*dq5+C666*dq6;


C = [C11, C12, C13, C14, C15, C16;
        C21, C22, C23, C24, C25, C26;
        C31, C32, C33, C34, C35, C36;
        C41, C42, C43, C44, C45, C46;
        C51, C52, C53, C54, C55, C56;
        C61, C62, C63, C64, C65, C66];

% yi

G1 = -m1*g'*U11*r1-m2*g'*U21*r2-m3*g'*U31*r3-m4*g'*U41*r4-m5*g'*U51*r5-m6*g'*U61*r6;
G2 = -m2*g'*U22*r2-m3*g'*U32*r3-m4*g'*U42*r4-m5*g'*U52*r5-m6*g'*U62*r6;
G3 = -m3*g'*U33*r3-m4*g'*U43*r4-m5*g'*U53*r5-m6*g'*U63*r6;
G4 = -m4*g'*U44*r4-m5*g'*U54*r5-m6*g'*U64*r6;
G5 = -m5*g'*U55*r5-m6*g'*U65*r6;
G6 = -m6*g'*U66*r6;

G = [G1;  G2;  G3;  G4;  G5;  G6];
end
