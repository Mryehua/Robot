function [sys,x0,str,ts]=s_function(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 12; % 角度与角速度
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;% 角度与角速度
sizes.NumInputs      = 6; % 关节力矩
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);
x0=[0,0,0,0,0,0,0,0,0,0,0,0];
str=[];
ts=[];
function sys=mdlDerivatives(t,x,u)
tol=[u(1);u(2);u(3);u(4);u(5);u(6)];
q1=x(1);
dq1=x(2);
q2=x(3);
dq2=x(4);
q3=x(5);
dq3=x(6);
q4=x(7);
dq4=x(8);
q5=x(9);
dq5=x(10);
q6=x(11);
dq6=x(12);

[M,C,G]=dynamic_model(q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6);
S=inv(M)*(tol-C*[dq1;dq2;dq3;dq4;dq5;dq6]-G);      

sys(1)=x(2);% dq1
sys(3)=x(4); %dq2
sys(5)=x(6); %dq3
sys(7)=x(8);% dq4
sys(9)=x(10); %dq5
sys(11)=x(12); %dq6
sys(2)=S(1);% ddq1
sys(4)=S(2);% ddq2
sys(6)=S(3);% ddq3
sys(8)=S(4);% ddq4
sys(10)=S(5);% ddq5
sys(12)=S(6);% ddq6

function sys=mdlOutputs(t,x,u)
sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);
sys(4)=x(4);
sys(5)=x(5);
sys(6)=x(6);
sys(7)=x(7);
sys(8)=x(8);
sys(9)=x(9);
sys(10)=x(10);
sys(11)=x(11);
sys(12)=x(12);