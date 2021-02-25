#ifndef MYROBOT_H
#define MYROBOT_H

#include <QObject>
#include<eigen/Eigen/Dense>
#include<iostream>
#include<QtMath>

typedef Eigen::Matrix<double,6,1> Vector61f ;
typedef Eigen::Matrix<double,4,4> Matrix44f ;

class Myrobot : public QObject
{
    Q_OBJECT
public:
    explicit Myrobot(QObject *parent = nullptr);

    Matrix44f Robot_Fkine(Vector61f Q); //fkine
    Vector61f Robot_Ikine(Matrix44f T,Eigen::Vector3d sol);  // ikine

    void SetVariable(Vector61f length,Vector61f deviation,Vector61f twist);  //set  DH variable
    void SetBaseFrame(Matrix44f base);
    void SetToolFrame(Matrix44f tool);

private:
    Matrix44f T01,T12,T23,T34,T45,T56;
    Vector61f a,d,alfa,theta;
    double pi;
    Matrix44f baseframe,toolframe;

    Matrix44f Mdh(double qi,double ai,double di,double alfai);
    void GetTi();

signals:

};

#endif // MYROBOT_H
