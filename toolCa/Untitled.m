clear
close all
clc

% （8-20之间）距离阈值越大，x轴方向的误差越大，y轴方向误差越小，Z轴方向误差越大


%% 工具系 eye-to-hand 标定  B-B系  1-1系  2-工具安装系
basedata1=importdata('da1.txt');
basedata2=importdata('da2.txt');
basedata3=importdata('da3.txt');
basedata4=importdata('da4.txt');
basedata5=importdata('data_1.txt');
basedata6=importdata('data_2.txt');
basedata7=importdata('data_3.txt');
basedata8=importdata('data_4.txt');

basedata10=importdata('data_6.txt');
basedata11=importdata('data_7.txt');
basedata12=importdata('data_8.txt');
basedata13=importdata('data_9.txt');

figure(1)
subplot(3,3,1)
plot(1:length(basedata1),basedata1(:,1),'r*');
grid on

subplot(3,3,2)
plot(1:length(basedata1),basedata1(:,2),'r*');
grid on

subplot(3,3,3)
plot(1:length(basedata1),basedata1(:,3),'r*');
grid on

subplot(3,3,4)
plot(1:length(basedata2),basedata2(:,1),'r*');
grid on
subplot(3,3,5)
plot(1:length(basedata2),basedata2(:,2),'r*');
grid on
subplot(3,3,6)
plot(1:length(basedata2),basedata2(:,3),'r*');
grid on

subplot(3,3,7)
plot(1:length(basedata3),basedata3(:,1),'r*');
grid on
subplot(3,3,8)
plot(1:length(basedata3),basedata3(:,2),'r*');
grid on
subplot(3,3,9)
plot(1:length(basedata3),basedata3(:,3),'r*');
grid on


figure(2)
subplot(3,3,1)
plot(1:length(basedata4),basedata4(:,1),'r*');
grid on
subplot(3,3,2)
plot(1:length(basedata4),basedata4(:,2),'r*');
grid on
subplot(3,3,3)
plot(1:length(basedata4),basedata4(:,3),'r*');
grid on

subplot(3,3,4)
plot(1:length(basedata5),basedata5(:,1),'r*');
grid on
subplot(3,3,5)
plot(1:length(basedata5),basedata5(:,2),'r*');
grid on
subplot(3,3,6)
plot(1:length(basedata5),basedata5(:,3),'r*');
grid on

subplot(3,3,7)
plot(1:length(basedata6),basedata6(:,1),'r*');
grid on
subplot(3,3,8)
plot(1:length(basedata6),basedata6(:,2),'r*');
grid on
subplot(3,3,9)
plot(1:length(basedata6),basedata6(:,3),'r*');
grid on


% 斜率
% 
% Lv_data1=basedata1(2:end,:)-basedata1(1:end-1,:);
% Lv_data2=basedata2(2:end,:)-basedata2(1:end-1,:);
% figure(3)
% subplot(3,3,1)
% plot(1:length(Lv_data1),Lv_data1(:,1),'r*');
% grid on
% subplot(3,3,2)
% plot(1:length(Lv_data1),Lv_data1(:,2),'r*');
% grid on
% subplot(3,3,3)
% plot(1:length(Lv_data1),Lv_data1(:,3),'r*');
% grid on
% 
% subplot(3,3,4)
% plot(1:length(Lv_data2),Lv_data2(:,1),'r*');
% grid on
% subplot(3,3,5)
% plot(1:length(Lv_data2),Lv_data2(:,2),'r*');
% grid on
% subplot(3,3,6)
% plot(1:length(Lv_data2),Lv_data2(:,3),'r*');
% grid on






%{
TB1=[eye(3),[-(1084.4-110/2-118.79/2);120.6+118.79/2;0];[0,0,0,1]];
T12=[eye(3),[0;-464.43;726.39+14];[0,0,0,1]];
T2t=[eye(3)*rotz(-90),[30/2;40;60];[0,0,0,1]];

tp1=[mean(basedata1(:,1));mean(basedata1(:,2));mean(basedata1(:,3));1];
tp2=[mean(basedata2(:,1));mean(basedata2(:,2));mean(basedata2(:,3));1];
tp3=[mean(basedata3(:,1));mean(basedata3(:,2));mean(basedata3(:,3));1];
tp4=[mean(basedata4(:,1));mean(basedata4(:,2));mean(basedata4(:,3));1];
tp5=[mean(basedata5(:,1));mean(basedata5(:,2));mean(basedata5(:,3));1];
tp6=[mean(basedata6(:,1));mean(basedata6(:,2));mean(basedata6(:,3));1];
tp7=[mean(basedata7(:,1));mean(basedata7(:,2));mean(basedata7(:,3));1];
tp8=[mean(basedata8(:,1));mean(basedata8(:,2));mean(basedata8(:,3));1];
%{
tp10=[mean(basedata10(:,1));mean(basedata10(:,2));mean(basedata10(:,3));1];
tp11=[mean(basedata11(:,1));mean(basedata11(:,2));mean(basedata11(:,3));1];
tp12=[mean(basedata12(:,1));mean(basedata12(:,2));mean(basedata12(:,3));1];
tp13=[mean(basedata13(:,1));mean(basedata13(:,2));mean(basedata13(:,3));1];

tp_new1=TB1*T12*T2t*tp1;
tp_new2=TB1*T12*T2t*tp2;
tp_new3=TB1*T12*T2t*tp3;
tp_new4=TB1*T12*T2t*tp4;
tp_new5=TB1*T12*T2t*tp5;
tp_new6=TB1*T12*T2t*tp6;
tp_new7=TB1*T12*T2t*tp7;
tp_new8=TB1*T12*T2t*tp8;

tp_new10=TB1*T12*T2t*tp10;
tp_new11=TB1*T12*T2t*tp11;
tp_new12=TB1*T12*T2t*tp12;
tp_new13=TB1*T12*T2t*tp13;
%}
% 验证点
TBm=[eye(3),[-(1084.4-118.79/2+105);120.6+118.79/2+20;0];[0,0,0,1]];

% distance 8 
xx1=425;yy1=345;zz=110; % da1
xx2=200;yy2=295;    % da2
xx3=110;yy3=502.5;   % da3
xx4=(400-260)/2+260;yy4=(710-500)/2+500; %da4
xx5=185;yy5=(580-375)/2+375;  % data_1
xx6=(250-110)/2+110;yy6=(566-362)/2+362; % data_2
xx7=(260-120)/2+120;yy7=(580-375)/2+375;  % data_3
xx8=(225-90)/2+90;yy8=(530-325)/2+325; % data_4
xx=[xx1,xx2,xx3,xx4,xx5,xx6,xx7,xx8];
yy=[yy1,yy2,yy3,yy4,yy5,yy6,yy7,yy8];

%distance 8、12、16、20
xx10=(260-120)/2+120;yy10=(510-305)/2+305;                           % data_6 7 8 9

Btp1=TBm*[xx1;-yy1;zz;1];
Btp2=TBm*[xx2;-yy2;zz;1];
Btp3=TBm*[xx3;-yy3;zz;1];
Btp4=TBm*[xx4;-yy4;zz;1];
Btp5=TBm*[xx5;-yy5;zz;1];
Btp6=TBm*[xx6;-yy6;zz;1];
Btp7=TBm*[xx7;-yy7;zz;1];
Btp8=TBm*[xx8;-yy8;zz;1];

Btp10=TBm*[xx10;-yy10;zz;1]; % 11 12 13






plot(xx6,yy6,'r*')
grid on
hold on
plot(tp6(1),tp6(2),'b*')
%}



