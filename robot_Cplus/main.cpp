#include <QCoreApplication>
#include"myrobot.h"
#include<eigen/Eigen/Dense>
#include<iostream>
#include<QtMath>

typedef Eigen::Matrix<double,6,1> Vector61f ;
typedef Eigen::Matrix<double,4,4> Matrix44f ;

#define pi 3.1415926

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    Myrobot *rob=new Myrobot;

    Vector61f q,d,aa,alfa;
    q<<0,10*pi/180,20*pi/180,0,40*pi/180,60*pi/180;

    d<<   0,  0,      0,     451,   0,     0;
    aa<<  0,    0,      443,   42,     0,     0;
    alfa<<0,    pi/2,   0,     pi/2,  -pi/2, pi/2;

    Eigen::Vector3d flag;
    flag<<1,1,1;
    Matrix44f base,tool;
    base<<1,0,0,0,
          0,1,0,0,
          0,0,1,399,
            0,0,0,1;
    tool<<1,0,0,0,
            0,1,0,0,
            0,0,1,82,
            0,0,0,1;
    rob->SetBaseFrame(base);
    rob->SetToolFrame(tool);

    rob->SetVariable(aa,d,alfa);
    Matrix44f T06=rob->Robot_Fkine(q);
    Vector61f qq=rob->Robot_Ikine(T06,flag);
    std::cout<<T06<<std::endl;
    std::cout<<qq.transpose()<<std::endl;


    delete rob;
    return a.exec();
}
