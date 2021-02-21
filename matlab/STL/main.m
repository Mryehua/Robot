clear
close all
clc

a=importdata('PtOut.txt');
plot3(a(:,2),a(:,3),a(:,4),'r-');
grid on


