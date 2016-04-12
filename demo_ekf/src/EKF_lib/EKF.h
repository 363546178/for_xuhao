#ifndef _EKF_H
#define _EKF_H

#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>
#include </usr/local/include/eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class EKF
{
public:
    EKF(int _n, int _m,
        void (*_f_func)(MatrixXd&,MatrixXd&,VectorXd&,const VectorXd&, const float dt),
        void (*_h_func)(MatrixXd&,VectorXd&,const VectorXd&, const VectorXd&),
        void (*_c_func)(const VectorXd&,VectorXd&),
        float _q, float _r);
    ~EKF();

    void setQ(MatrixXd _Q);
    void setR(MatrixXd _R);

    void setQ(float _Q);
    void setR(float _R);

    void setQ(int _n, float* _Q);
    void setR(int _m, float* _R);

    void predict(VectorXd& state, const VectorXd& imu, const float dt);
    void update(const VectorXd& state, const VectorXd& vicon);
    void correct(VectorXd& state);

private:

    int n;  //number of state
    int m;  //number of measurement

    void init_filter(int _n, int _m);

    // function pointer to process function
    //      f_func(    F,       G,       state,       read in,          dt)
    void (*f_func)(MatrixXd&,MatrixXd&,VectorXd&,const VectorXd&, const float dt);
    // function pointer to measurement function
    //      h_func(      H,    error,       state,           read in)
    void (*h_func)(MatrixXd&,VectorXd&,const VectorXd&, const VectorXd&);
    // function pointer to state correct function
    //     c_func(      error,       state)
    void (*c_func)(const VectorXd&,VectorXd&);

    MatrixXd F_mat;
    MatrixXd Phi_mat;
    MatrixXd G_mat;
    MatrixXd H_mat;

    MatrixXd P_mat;

    MatrixXd Q_mat;
    MatrixXd R_mat;
    MatrixXd Kg;

    VectorXd error_in;
    VectorXd error_out;

};
#endif
