#include <iostream>
#include <stdio.h>

#include <Eigen/Dense>

#include "math_uilts.h"

using namespace std;
using namespace Eigen;

typedef struct
{
    double time_stamp;
    VectorXd x;
}state;

class EKF
{
public:
    EKF(int _n, int _m, float _q, float _r);

    // q_vb | p_v | v_v | bias_a_b |
    //  4   |  3  |  3  |   3      |
    VectorXd x;

    Matrix3d R_ve;
    Quaterniond q_ve;

    void init_filter(int _n, int _m);

    void setQ(MatrixXd _Q);
    void setR(MatrixXd _R);

    void setQ(float _Q);
    void setR(float _R);

    void init_ekf_state(Quaterniond q_vb_init, Vector3d p_v_init);
    void predict(const Quaterniond q_eb, const Vector3d w_b, const Vector3d a_e, const float dt);
    void measrue_update(const Quaterniond q_vb_meas,const Vector3d p_v_meas);
    void correct();

    const Quaterniond get_orientation_q_vb();
    const Vector3d get_position_v();
    const Vector3d get_velocity_v();
    const Vector3d get_bias_a();
    const Vector3d get_accelerate_e();

    int n;  //number of state
    int m;  //number of measurement

    bool is_init;

    MatrixXd P_post;
    MatrixXd F_mat;
    MatrixXd Phi_mat;
    MatrixXd G_mat;
    MatrixXd H_mat;

    VectorXd error_in;
    VectorXd error_out;

private:
    Vector3d a_e;

    MatrixXd P_pre;

    MatrixXd A_mat;
    MatrixXd Q_mat;
    MatrixXd R_mat;
    MatrixXd Kg;

};
