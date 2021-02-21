clear
close all
clc

%% 机器人（位置）运动学(pieper)逆解
syms a1 a2 a3 a4 a5 a6 d1 d2 d3 d4 d5 d6 aa1 aa2 aa3 aa4 aa5 aa6 real
sol = [1 1 1];  % left, up, noflip 初始化
%{
for c=configuration
    switch c
        case  'l'
            sol(1)=1;
        case  'r'
            sol(1)=2;
        case  'u'
            sol(2)=1;
        case  'd'
            sol(2)=2;
        case  'n'
            sol(3)=1;
        case  'f'
            sol(3)=2;
    end
end
%}

nx=T(1,1);
ny=T(2,1);
nz=T(3,1);
ox=T(1,2);
oy=T(2,2);
oz=T(3,2);
ax=T(1,3);
ay=T(2,3);
az=T(3,3);
px=T(1,4);
py=T(2,4);
pz=T(3,4);

%theta1
if sol(1)==1
    theta1=atan2(py,px)-atan2(0,1);
else
    theta1=atan2(py,px)-atan2(0,-1);
end
theta(1)=theta1;
%theta3
k=((-a1+cos(theta(1))*px+sin(theta(1))*py)^2+pz^2-a2^2-d4^2-a3^2)/(2*a2);
if sol(2)==1
    theta3=atan2(k,sqrt(a3^2+d4^2-k^2))-atan2(a3,d4);
else
    theta3=atan2(k,-sqrt(a3^2+d4^2-k^2))-atan2(a3,d4);
end
theta(3)=theta3;
%theta2
s23=(pz*(a3+a2*cos(theta(3)))+(d4+a2*sin(theta(3)))*(cos(theta(1))*px+sin(theta(1))*py-a1));
c23=-(d4+a2*sin(theta(3)))*pz+(cos(theta(1))*px+sin(theta(1))*py-a1)*(a3+a2*cos(theta(3)));
theta(2)=atan2(s23,c23)-theta(3);

c4s5=cos(theta(1))*cos(theta(2)+theta(3))*ax+sin(theta(1))*cos(theta(2)+theta(3))*ay+sin(theta(2)+theta(3))*az;
s4s5=sin(theta(1))*ax-cos(theta(1))*ay;
if c4s5==0 && s4s5==0
    %奇异位形，这时theta(4)应该取前一时刻的值，
    theta(5)=0;
    theta(4)=0;
    theta(6)=0;
else
    theta(4)=atan2(s4s5,c4s5);
    
s5=(cos(theta(1))*cos(theta(2)+theta(3))*cos(theta(4))+sin(theta(1))*sin(theta(4)))*ax+(sin(theta(1))*cos(theta(2)+theta(3))*cos(theta(4))-cos(theta(1))*sin(theta(4)))*ay+sin(theta(2)+theta(3))*cos(theta(4))*az;
c5=cos(theta(1))*sin(theta(2)+theta(3))*ax+sin(theta(1))*sin(theta(2)+theta(3))*ay-cos(theta(2)+theta(3))*az;
theta(5)=atan2(s5,c5);

    s5s6=cos(theta(1))*sin(theta(2)+theta(3))*ox+sin(theta(1))*sin(theta(2)+theta(3))*oy-cos(theta(2)+theta(3))*oz;
    s5c6=-cos(theta(1))*sin(theta(2)+theta(3))*nx-sin(theta(1))*sin(theta(2)+theta(3))*ny+cos(theta(2)+theta(3))*nz;
    theta(6)=atan2(s5s6,s5c6);
end
if sol(3)==1
    q=[theta(1) theta(2) theta(3) theta(4) -theta(5) theta(6)];
else
    q=[theta(1) theta(2) theta(3) theta(4)+pi theta(5) theta(6)+pi];
end
q=[q(1) q(2) q(3) q(4) q(5) q(6)].*(180/pi);

