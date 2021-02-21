%%  GSC6_with_Load配套使用，这个模型是机器人带有一个2kg的负载时停机状态，
clear;
clc
GSC6_with_load = importrobot('GSC_with_load.urdf');   %导入urdf文件
showdetails(GSC6_with_load)  %显示连杆间的父子关系
show(GSC6_with_load,'Frames','on','Visuals','on')  %figure显示
GSC6_with_tool_collision_SM = smimport('GSC_with_load.urdf');

%%  GSC_6dof_initial_robot配套使用
clear;
clc
GSC6_initial_robot = importrobot('GSC_6dof_initial_robot.urdf');   %导入urdf文件
showdetails(GSC6_initial_robot)  %显示连杆间的父子关系
show(GSC6_initial_robot,'Frames','on','Visuals','on')  %figure显示
GSC6_with_tool_collision_SM = smimport('GSC_6dof_initial_robot.urdf');