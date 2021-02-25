#include "myrobot.h"

Myrobot::Myrobot(QObject *parent) : QObject(parent)
{
    // init

    a<<0,0,0,0,0,0;
    d<<0,0,0,0,0,0;
    alfa<<0,0,0,0,0,0;
    theta<<0,0,0,0,0,0;
    T01=Matrix44f::Identity(4,4);
    T12=Matrix44f::Identity(4,4);
    T23=Matrix44f::Identity(4,4);
    T34=Matrix44f::Identity(4,4);
    T45=Matrix44f::Identity(4,4);
    T56=Matrix44f::Identity(4,4);
    baseframe=Matrix44f::Identity(4,4);
    toolframe=Matrix44f::Identity(4,4);
    pi=3.1415926;

}

Matrix44f Myrobot::Robot_Fkine(Vector61f Q)
{
    theta=Q;
    GetTi();
    return baseframe*T01*T12*T23*T34*T45*T56*toolframe;
}

void Myrobot::GetTi()
{
    T01=Mdh(theta[0],a[0],d[0],alfa[0]);
    T12=Mdh(theta[1],a[1],d[1],alfa[1]);
    T23=Mdh(theta[2],a[2],d[2],alfa[2]);
    T34=Mdh(theta[3],a[3],d[3],alfa[3]);
    T45=Mdh(theta[4],a[4],d[4],alfa[4]);
    T56=Mdh(theta[5],a[5],d[5],alfa[5]);

}

Matrix44f Myrobot::Mdh(double qi,double ai,double di,double alfai)
{
    Matrix44f Ti;
    Ti<<cos(qi),-sin(qi),0,ai,
            sin(qi)*cos(alfai),cos(qi)*cos(alfai),-sin(alfai),-di*sin(alfai),
            sin(qi)*sin(alfai),cos(qi)*sin(alfai),cos(alfai),di*cos(alfai),
            0,0,0,1;

    return Ti;
}

void Myrobot::SetVariable(Vector61f length, Vector61f deviation, Vector61f twist)
{
    a=length;
    d=deviation;
    alfa=twist;
}

Vector61f Myrobot::Robot_Ikine(Matrix44f T,Eigen::Vector3d sol)
{
    Matrix44f T06=baseframe.inverse()*T*toolframe.inverse();

    double nx=T06(0,0);
    double ny=T06(1,0);
    double nz=T06(2,0);

    double ox=T06(0,1);
    double oy=T06(1,1);
    double oz=T06(2,1);

    double ax=T06(0,2);
    double ay=T06(1,2);
    double az=T06(2,2);

    double px=T06(0,3);
    double py=T06(1,3);
    double pz=T06(2,3);

    double theta1,theta2,theta3,theta4,theta5,theta6;

    if(sol(0)==1)
        theta1=atan2(py,px)-atan2(0,1);
    else
        theta1=atan2(py,px)-atan2(0,-1);

    double k=(qPow(  -a(1)+cos(theta1)*px+sin(theta1)*py,2)+pz*pz-qPow(a(2),2)-qPow(d(3),2)-qPow(a(3),2))/(2*a(2));
    if(sol(1)==1)
        theta3=atan2(k,sqrt(qPow(a(3),2)+qPow(d(3),2)-k*k))-atan2(a(3),d(3));
    else
        theta3=atan2(k,-sqrt(qPow(a(3),2)+qPow(d(3),2)-k*k))-atan2(a(3),d(3));



    double s23=pz*( a(3)+a(2)*cos(theta3)) +(d(3)+a(2)*sin(theta3)) *(cos(theta1)*px+sin(theta1)*py-a(1));
    double c23=-(d(3)+a(2)*sin(theta3))*pz+(cos(theta1)*px+sin(theta1)*py-a(1))*(a(3)+a(2)*cos(theta3));
    theta2=atan2(s23,c23)-theta3;


    double c4s5=cos(theta1)*cos(theta2+theta3)*ax+sin(theta1)*cos(theta2+theta3)*ay+sin(theta2+theta3)*az;
    double s4s5=sin(theta1)*ax-cos(theta1)*ay;

    if(c4s5==1 && s4s5==1 )
    {
        //  奇异
        theta4=0;
        theta5=0;
        theta6=0;
    }
    else {
        theta4=atan2(s4s5,c4s5);

        double s5=(cos(theta1)*cos(theta2+theta3)*cos(theta4)+sin(theta1)*sin(theta4))*ax+(sin(theta1)*cos(theta2+theta3)*cos(theta4)-cos(theta1)*sin(theta4))*ay+sin(theta2+theta3)*cos(theta4)*az;
        double c5=cos(theta1)*sin(theta2+theta3)*ax+sin(theta1)*sin(theta2+theta3)*ay-cos(theta2+theta3)*az;
        theta5=atan2(s5,c5);


        double s5s6=cos(theta1)*sin(theta2+theta3)*ox+sin(theta1)*sin(theta2+theta3)*oy-cos(theta2+theta3)*oz;
        double s5c6=-cos(theta1)*sin(theta2+theta3)*nx-sin(theta1)*sin(theta2+theta3)*ny+cos(theta2+theta3)*nz;
        theta6=atan2(s5s6,s5c6);

        if(sol(2)==1)
            theta<<theta1,theta2,theta3,theta4,theta5,theta6;
        else
            theta<<theta1,theta2,theta3,theta4,-theta5,theta6+pi;
    }
    return theta;
}

void Myrobot::SetBaseFrame(Matrix44f base)
{

    baseframe=base;
}

void Myrobot::SetToolFrame(Matrix44f tool)
{

    toolframe=tool;
}







