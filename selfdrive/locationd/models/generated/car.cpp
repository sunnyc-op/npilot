#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6777767383690293166) {
   out_6777767383690293166[0] = delta_x[0] + nom_x[0];
   out_6777767383690293166[1] = delta_x[1] + nom_x[1];
   out_6777767383690293166[2] = delta_x[2] + nom_x[2];
   out_6777767383690293166[3] = delta_x[3] + nom_x[3];
   out_6777767383690293166[4] = delta_x[4] + nom_x[4];
   out_6777767383690293166[5] = delta_x[5] + nom_x[5];
   out_6777767383690293166[6] = delta_x[6] + nom_x[6];
   out_6777767383690293166[7] = delta_x[7] + nom_x[7];
   out_6777767383690293166[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5397592667997122704) {
   out_5397592667997122704[0] = -nom_x[0] + true_x[0];
   out_5397592667997122704[1] = -nom_x[1] + true_x[1];
   out_5397592667997122704[2] = -nom_x[2] + true_x[2];
   out_5397592667997122704[3] = -nom_x[3] + true_x[3];
   out_5397592667997122704[4] = -nom_x[4] + true_x[4];
   out_5397592667997122704[5] = -nom_x[5] + true_x[5];
   out_5397592667997122704[6] = -nom_x[6] + true_x[6];
   out_5397592667997122704[7] = -nom_x[7] + true_x[7];
   out_5397592667997122704[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1600455802523382261) {
   out_1600455802523382261[0] = 1.0;
   out_1600455802523382261[1] = 0;
   out_1600455802523382261[2] = 0;
   out_1600455802523382261[3] = 0;
   out_1600455802523382261[4] = 0;
   out_1600455802523382261[5] = 0;
   out_1600455802523382261[6] = 0;
   out_1600455802523382261[7] = 0;
   out_1600455802523382261[8] = 0;
   out_1600455802523382261[9] = 0;
   out_1600455802523382261[10] = 1.0;
   out_1600455802523382261[11] = 0;
   out_1600455802523382261[12] = 0;
   out_1600455802523382261[13] = 0;
   out_1600455802523382261[14] = 0;
   out_1600455802523382261[15] = 0;
   out_1600455802523382261[16] = 0;
   out_1600455802523382261[17] = 0;
   out_1600455802523382261[18] = 0;
   out_1600455802523382261[19] = 0;
   out_1600455802523382261[20] = 1.0;
   out_1600455802523382261[21] = 0;
   out_1600455802523382261[22] = 0;
   out_1600455802523382261[23] = 0;
   out_1600455802523382261[24] = 0;
   out_1600455802523382261[25] = 0;
   out_1600455802523382261[26] = 0;
   out_1600455802523382261[27] = 0;
   out_1600455802523382261[28] = 0;
   out_1600455802523382261[29] = 0;
   out_1600455802523382261[30] = 1.0;
   out_1600455802523382261[31] = 0;
   out_1600455802523382261[32] = 0;
   out_1600455802523382261[33] = 0;
   out_1600455802523382261[34] = 0;
   out_1600455802523382261[35] = 0;
   out_1600455802523382261[36] = 0;
   out_1600455802523382261[37] = 0;
   out_1600455802523382261[38] = 0;
   out_1600455802523382261[39] = 0;
   out_1600455802523382261[40] = 1.0;
   out_1600455802523382261[41] = 0;
   out_1600455802523382261[42] = 0;
   out_1600455802523382261[43] = 0;
   out_1600455802523382261[44] = 0;
   out_1600455802523382261[45] = 0;
   out_1600455802523382261[46] = 0;
   out_1600455802523382261[47] = 0;
   out_1600455802523382261[48] = 0;
   out_1600455802523382261[49] = 0;
   out_1600455802523382261[50] = 1.0;
   out_1600455802523382261[51] = 0;
   out_1600455802523382261[52] = 0;
   out_1600455802523382261[53] = 0;
   out_1600455802523382261[54] = 0;
   out_1600455802523382261[55] = 0;
   out_1600455802523382261[56] = 0;
   out_1600455802523382261[57] = 0;
   out_1600455802523382261[58] = 0;
   out_1600455802523382261[59] = 0;
   out_1600455802523382261[60] = 1.0;
   out_1600455802523382261[61] = 0;
   out_1600455802523382261[62] = 0;
   out_1600455802523382261[63] = 0;
   out_1600455802523382261[64] = 0;
   out_1600455802523382261[65] = 0;
   out_1600455802523382261[66] = 0;
   out_1600455802523382261[67] = 0;
   out_1600455802523382261[68] = 0;
   out_1600455802523382261[69] = 0;
   out_1600455802523382261[70] = 1.0;
   out_1600455802523382261[71] = 0;
   out_1600455802523382261[72] = 0;
   out_1600455802523382261[73] = 0;
   out_1600455802523382261[74] = 0;
   out_1600455802523382261[75] = 0;
   out_1600455802523382261[76] = 0;
   out_1600455802523382261[77] = 0;
   out_1600455802523382261[78] = 0;
   out_1600455802523382261[79] = 0;
   out_1600455802523382261[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5233723299563261311) {
   out_5233723299563261311[0] = state[0];
   out_5233723299563261311[1] = state[1];
   out_5233723299563261311[2] = state[2];
   out_5233723299563261311[3] = state[3];
   out_5233723299563261311[4] = state[4];
   out_5233723299563261311[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5233723299563261311[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5233723299563261311[7] = state[7];
   out_5233723299563261311[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7948139696811054819) {
   out_7948139696811054819[0] = 1;
   out_7948139696811054819[1] = 0;
   out_7948139696811054819[2] = 0;
   out_7948139696811054819[3] = 0;
   out_7948139696811054819[4] = 0;
   out_7948139696811054819[5] = 0;
   out_7948139696811054819[6] = 0;
   out_7948139696811054819[7] = 0;
   out_7948139696811054819[8] = 0;
   out_7948139696811054819[9] = 0;
   out_7948139696811054819[10] = 1;
   out_7948139696811054819[11] = 0;
   out_7948139696811054819[12] = 0;
   out_7948139696811054819[13] = 0;
   out_7948139696811054819[14] = 0;
   out_7948139696811054819[15] = 0;
   out_7948139696811054819[16] = 0;
   out_7948139696811054819[17] = 0;
   out_7948139696811054819[18] = 0;
   out_7948139696811054819[19] = 0;
   out_7948139696811054819[20] = 1;
   out_7948139696811054819[21] = 0;
   out_7948139696811054819[22] = 0;
   out_7948139696811054819[23] = 0;
   out_7948139696811054819[24] = 0;
   out_7948139696811054819[25] = 0;
   out_7948139696811054819[26] = 0;
   out_7948139696811054819[27] = 0;
   out_7948139696811054819[28] = 0;
   out_7948139696811054819[29] = 0;
   out_7948139696811054819[30] = 1;
   out_7948139696811054819[31] = 0;
   out_7948139696811054819[32] = 0;
   out_7948139696811054819[33] = 0;
   out_7948139696811054819[34] = 0;
   out_7948139696811054819[35] = 0;
   out_7948139696811054819[36] = 0;
   out_7948139696811054819[37] = 0;
   out_7948139696811054819[38] = 0;
   out_7948139696811054819[39] = 0;
   out_7948139696811054819[40] = 1;
   out_7948139696811054819[41] = 0;
   out_7948139696811054819[42] = 0;
   out_7948139696811054819[43] = 0;
   out_7948139696811054819[44] = 0;
   out_7948139696811054819[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7948139696811054819[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7948139696811054819[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7948139696811054819[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7948139696811054819[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7948139696811054819[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7948139696811054819[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7948139696811054819[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7948139696811054819[53] = -9.8000000000000007*dt;
   out_7948139696811054819[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7948139696811054819[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7948139696811054819[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7948139696811054819[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7948139696811054819[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7948139696811054819[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7948139696811054819[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7948139696811054819[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7948139696811054819[62] = 0;
   out_7948139696811054819[63] = 0;
   out_7948139696811054819[64] = 0;
   out_7948139696811054819[65] = 0;
   out_7948139696811054819[66] = 0;
   out_7948139696811054819[67] = 0;
   out_7948139696811054819[68] = 0;
   out_7948139696811054819[69] = 0;
   out_7948139696811054819[70] = 1;
   out_7948139696811054819[71] = 0;
   out_7948139696811054819[72] = 0;
   out_7948139696811054819[73] = 0;
   out_7948139696811054819[74] = 0;
   out_7948139696811054819[75] = 0;
   out_7948139696811054819[76] = 0;
   out_7948139696811054819[77] = 0;
   out_7948139696811054819[78] = 0;
   out_7948139696811054819[79] = 0;
   out_7948139696811054819[80] = 1;
}
void h_25(double *state, double *unused, double *out_2103105793419728366) {
   out_2103105793419728366[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1472224050795332230) {
   out_1472224050795332230[0] = 0;
   out_1472224050795332230[1] = 0;
   out_1472224050795332230[2] = 0;
   out_1472224050795332230[3] = 0;
   out_1472224050795332230[4] = 0;
   out_1472224050795332230[5] = 0;
   out_1472224050795332230[6] = 1;
   out_1472224050795332230[7] = 0;
   out_1472224050795332230[8] = 0;
}
void h_24(double *state, double *unused, double *out_6612164286682862269) {
   out_6612164286682862269[0] = state[4];
   out_6612164286682862269[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6341038915823039082) {
   out_6341038915823039082[0] = 0;
   out_6341038915823039082[1] = 0;
   out_6341038915823039082[2] = 0;
   out_6341038915823039082[3] = 0;
   out_6341038915823039082[4] = 1;
   out_6341038915823039082[5] = 0;
   out_6341038915823039082[6] = 0;
   out_6341038915823039082[7] = 0;
   out_6341038915823039082[8] = 0;
   out_6341038915823039082[9] = 0;
   out_6341038915823039082[10] = 0;
   out_6341038915823039082[11] = 0;
   out_6341038915823039082[12] = 0;
   out_6341038915823039082[13] = 0;
   out_6341038915823039082[14] = 1;
   out_6341038915823039082[15] = 0;
   out_6341038915823039082[16] = 0;
   out_6341038915823039082[17] = 0;
}
void h_30(double *state, double *unused, double *out_2378299855704234255) {
   out_2378299855704234255[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5999920380922940428) {
   out_5999920380922940428[0] = 0;
   out_5999920380922940428[1] = 0;
   out_5999920380922940428[2] = 0;
   out_5999920380922940428[3] = 0;
   out_5999920380922940428[4] = 1;
   out_5999920380922940428[5] = 0;
   out_5999920380922940428[6] = 0;
   out_5999920380922940428[7] = 0;
   out_5999920380922940428[8] = 0;
}
void h_26(double *state, double *unused, double *out_2623373411956611390) {
   out_2623373411956611390[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5213727369669388454) {
   out_5213727369669388454[0] = 0;
   out_5213727369669388454[1] = 0;
   out_5213727369669388454[2] = 0;
   out_5213727369669388454[3] = 0;
   out_5213727369669388454[4] = 0;
   out_5213727369669388454[5] = 0;
   out_5213727369669388454[6] = 0;
   out_5213727369669388454[7] = 1;
   out_5213727369669388454[8] = 0;
}
void h_27(double *state, double *unused, double *out_4295935723994120780) {
   out_4295935723994120780[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8174683692723365339) {
   out_8174683692723365339[0] = 0;
   out_8174683692723365339[1] = 0;
   out_8174683692723365339[2] = 0;
   out_8174683692723365339[3] = 1;
   out_8174683692723365339[4] = 0;
   out_8174683692723365339[5] = 0;
   out_8174683692723365339[6] = 0;
   out_8174683692723365339[7] = 0;
   out_8174683692723365339[8] = 0;
}
void h_29(double *state, double *unused, double *out_658486195740098430) {
   out_658486195740098430[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5489689036608548244) {
   out_5489689036608548244[0] = 0;
   out_5489689036608548244[1] = 1;
   out_5489689036608548244[2] = 0;
   out_5489689036608548244[3] = 0;
   out_5489689036608548244[4] = 0;
   out_5489689036608548244[5] = 0;
   out_5489689036608548244[6] = 0;
   out_5489689036608548244[7] = 0;
   out_5489689036608548244[8] = 0;
}
void h_28(double *state, double *unused, double *out_102016357287508232) {
   out_102016357287508232[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3526058765043221993) {
   out_3526058765043221993[0] = 1;
   out_3526058765043221993[1] = 0;
   out_3526058765043221993[2] = 0;
   out_3526058765043221993[3] = 0;
   out_3526058765043221993[4] = 0;
   out_3526058765043221993[5] = 0;
   out_3526058765043221993[6] = 0;
   out_3526058765043221993[7] = 0;
   out_3526058765043221993[8] = 0;
}
void h_31(double *state, double *unused, double *out_6201409091699100230) {
   out_6201409091699100230[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5839935471902739930) {
   out_5839935471902739930[0] = 0;
   out_5839935471902739930[1] = 0;
   out_5839935471902739930[2] = 0;
   out_5839935471902739930[3] = 0;
   out_5839935471902739930[4] = 0;
   out_5839935471902739930[5] = 0;
   out_5839935471902739930[6] = 0;
   out_5839935471902739930[7] = 0;
   out_5839935471902739930[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_6777767383690293166) {
  err_fun(nom_x, delta_x, out_6777767383690293166);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5397592667997122704) {
  inv_err_fun(nom_x, true_x, out_5397592667997122704);
}
void car_H_mod_fun(double *state, double *out_1600455802523382261) {
  H_mod_fun(state, out_1600455802523382261);
}
void car_f_fun(double *state, double dt, double *out_5233723299563261311) {
  f_fun(state,  dt, out_5233723299563261311);
}
void car_F_fun(double *state, double dt, double *out_7948139696811054819) {
  F_fun(state,  dt, out_7948139696811054819);
}
void car_h_25(double *state, double *unused, double *out_2103105793419728366) {
  h_25(state, unused, out_2103105793419728366);
}
void car_H_25(double *state, double *unused, double *out_1472224050795332230) {
  H_25(state, unused, out_1472224050795332230);
}
void car_h_24(double *state, double *unused, double *out_6612164286682862269) {
  h_24(state, unused, out_6612164286682862269);
}
void car_H_24(double *state, double *unused, double *out_6341038915823039082) {
  H_24(state, unused, out_6341038915823039082);
}
void car_h_30(double *state, double *unused, double *out_2378299855704234255) {
  h_30(state, unused, out_2378299855704234255);
}
void car_H_30(double *state, double *unused, double *out_5999920380922940428) {
  H_30(state, unused, out_5999920380922940428);
}
void car_h_26(double *state, double *unused, double *out_2623373411956611390) {
  h_26(state, unused, out_2623373411956611390);
}
void car_H_26(double *state, double *unused, double *out_5213727369669388454) {
  H_26(state, unused, out_5213727369669388454);
}
void car_h_27(double *state, double *unused, double *out_4295935723994120780) {
  h_27(state, unused, out_4295935723994120780);
}
void car_H_27(double *state, double *unused, double *out_8174683692723365339) {
  H_27(state, unused, out_8174683692723365339);
}
void car_h_29(double *state, double *unused, double *out_658486195740098430) {
  h_29(state, unused, out_658486195740098430);
}
void car_H_29(double *state, double *unused, double *out_5489689036608548244) {
  H_29(state, unused, out_5489689036608548244);
}
void car_h_28(double *state, double *unused, double *out_102016357287508232) {
  h_28(state, unused, out_102016357287508232);
}
void car_H_28(double *state, double *unused, double *out_3526058765043221993) {
  H_28(state, unused, out_3526058765043221993);
}
void car_h_31(double *state, double *unused, double *out_6201409091699100230) {
  h_31(state, unused, out_6201409091699100230);
}
void car_H_31(double *state, double *unused, double *out_5839935471902739930) {
  H_31(state, unused, out_5839935471902739930);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
