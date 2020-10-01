clear
close all
clc

%% 机器人（位置）运动学（MDH）正解代码
Q=[q1 q2 q3 q4 q5 q6]; %转动角度
a=[a1,a2,a3,a4,a5,a6];  %扭转角
d=[d1,d2,d3,d4,d5,d6];   %偏置
aa=[aa1,aa2,aa3,aa4,aa5,aa6];%连杆参数
T=[];
w=eye(4);
Q=Q.*pi/180;              
for i=1:1:6
    T11=[rotx(a(i)),[0;0;0];[0,0,0,1]]*[1,0,0,aa(i);0,1,0,0;0,0,1,0;0,0,0,1]*[rotz(Q(i)),[0;0;0];[0,0,0,1]]*transl([0;0;d(i)]);
    w=w*T11;
end
T=[T,t1*w*t2];