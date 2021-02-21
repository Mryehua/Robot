function [sys,x0,str,ts] = input(t,x,u,flag)

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

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs =18;
sizes.NumInputs = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(t,x,u)
q1_d=5*sin(2*pi*t);
q2_d=5*sin(2*pi*t);
q3_d=5*sin(2*pi*t);
q4_d=5*sin(2*pi*t);
q5_d=5*sin(2*pi*t);
q6_d=5*sin(2*pi*t);

dq1_d=5*2*pi*cos(2*pi*t);
dq2_d=5*2*pi*cos(2*pi*t);
dq3_d=5*2*pi*cos(2*pi*t);
dq4_d=5*2*pi*cos(2*pi*t);
dq5_d=5*2*pi*cos(2*pi*t);
dq6_d=5*2*pi*cos(2*pi*t);

ddq1_d=-5*(2*pi)^2*sin(2*pi*t);
ddq2_d=-5*(2*pi)^2*sin(2*pi*t);
ddq3_d=-5*(2*pi)^2*sin(2*pi*t);
ddq4_d=-5*(2*pi)^2*sin(2*pi*t);
ddq5_d=-5*(2*pi)^2*sin(2*pi*t);
ddq6_d=-5*(2*pi)^2*sin(2*pi*t);

sys(1)=q1_d;
sys(4)=q2_d;
sys(7)=q3_d;
sys(10)=q4_d;
sys(13)=q5_d;
sys(16)=q6_d;

sys(2)=dq1_d;
sys(5)=dq2_d;
sys(8)=dq3_d;
sys(11)=dq4_d;
sys(14)=dq5_d;
sys(17)=dq6_d;

sys(3)=ddq1_d;
sys(6)=ddq2_d;
sys(9)=ddq3_d;
sys(12)=ddq4_d;
sys(15)=ddq5_d;
sys(18)=ddq6_d;




