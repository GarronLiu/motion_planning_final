#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff=Eigen::MatrixXd::Zero(m, 3 * p_num1d);
 
  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  int dim = m*6;
   Eigen::MatrixXd M = Eigen::MatrixXd::Zero(dim , dim);
   Eigen::MatrixXd  b = Eigen::MatrixXd::Zero(dim,3);
   Eigen::MatrixXd coefficientMatrix= Eigen::MatrixXd::Zero(dim,3);
   VectorXd initialPos=Path.row(0);
   VectorXd initialVel=Vel.row(0);
  VectorXd initialAcc=Acc.row(0);
  VectorXd terminalPos=Path.row(Path.rows()-1);
   VectorXd terminalVel=Vel.row(1);
  VectorXd terminalAcc=Acc.row(1);
   //填入F_0分块矩阵
   M.block(0,0,3,6)<< 1,  0,  0,  0,  0,  0,\
               0,  1,  0,  0,  0,  0,\
               0,  0,  2,  0,  0,  0;
   //填入对应右端项
   b.block(0,0,3,3)<<initialPos(0),initialPos(1),initialPos(2),\
                                        initialVel(0),initialVel(1),initialVel(2),\
                                        initialAcc(0),initialAcc(1),initialAcc(2);
   //填入Ei和Fi分块矩阵
   for(int i =0;i<dim/6-1;i++)
   {
       int index = i*6;
       double t =Time(i);
    //    填入Fi分块矩阵
           M.block(index+3,index+6,6,6)<< 0,  0,  0,  0,  0,  0,\
              -1,  0,  0,  0,  0,  0,\
               0, -1,  0,  0,  0,  0,\
               0,  0,  -2,  0,  0,  0,\
               0,  0,  0,  -6,  0,  0,\
               0,  0,  0,  0,  -24, 0;
           b.block(index+3,0,1,3)=Path.row(i+1);
    //    填入Ei分块矩阵
           M.block(index+3,index,6,6)<<1,    t,     pow(t,2),    pow(t,3),    pow(t,4),    pow(t,5),  \
                                                                        1,    t,     pow(t,2),    pow(t,3),    pow(t,4),    pow(t,5),  \
                                                                        0,    1,     2*t,      3*pow(t,2),    4*pow(t,3),    5*pow(t,4), \
                                                                        0,    0,    2,     6*t,    12*t*t,    20*pow(t,3) ,\
                                                                        0,    0,    0,     6,     24*t,    60*t*t,\
                                                                        0,    0,    0,     0,     24,     120*t;
   }
   //填入分块矩阵E_M
   double t=Time(dim/6-1);
   M.block(dim-3,dim-6,3,6)<<1,    t,     pow(t,2),    pow(t,3),    pow(t,4),    pow(t,5),  \
                                                            0,    1,     2*t,      3*pow(t,2),    4*pow(t,3),    5*pow(t,4), \
                                                            0,    0,    2,     6*t,    12*t*t,    20*pow(t,3);
   b.block(dim-3,0,3,3)<<terminalPos(0),terminalPos(1),terminalPos(2),\
                                        terminalVel(0),terminalVel(1),terminalVel(2),\
                                        terminalAcc(0),terminalAcc(1),terminalAcc(2);
  //  std::cout<<"M is "<<M<<endl;;
  //  std::cout<<"b is "<<b<<endl;
//    基于LU分解，求解三个方向上的分段系数
   for (int i=0;i<3;i++){
          coefficientMatrix.col(i)=M.lu().solve(b.col(i));
   }
  for(int i=0;i<m;i++)
  {
    for(int j=0;j<3;j++)
    {
      VectorXd mVec = coefficientMatrix.block(i*p_num1d,j,6,1);
      PolyCoeff.block(i,j*p_num1d,1,6)=mVec.transpose();
    }
  }
  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
    // cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}