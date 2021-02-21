function plotframe(T)
%显示坐标轴，红色为X轴，绿色为Y轴，蓝色为Z轴，单个轴的长度为50
xp=T(1:3,4)+50*T(1:3,1);
yp=T(1:3,4)+50*T(1:3,2);
zp=T(1:3,4)+50*T(1:3,3);
plot3([T(1,4),xp(1)],[T(2,4),xp(2)],[T(3,4),xp(3)],'r-');
hold on
plot3([T(1,4),yp(1)],[T(2,4),yp(2)],[T(3,4),yp(3)],'g-');
hold on
plot3([T(1,4),zp(1)],[T(2,4),zp(2)],[T(3,4),zp(3)],'b-');
hold on
end