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
void err_fun(double *nom_x, double *delta_x, double *out_9076953633350933155) {
   out_9076953633350933155[0] = delta_x[0] + nom_x[0];
   out_9076953633350933155[1] = delta_x[1] + nom_x[1];
   out_9076953633350933155[2] = delta_x[2] + nom_x[2];
   out_9076953633350933155[3] = delta_x[3] + nom_x[3];
   out_9076953633350933155[4] = delta_x[4] + nom_x[4];
   out_9076953633350933155[5] = delta_x[5] + nom_x[5];
   out_9076953633350933155[6] = delta_x[6] + nom_x[6];
   out_9076953633350933155[7] = delta_x[7] + nom_x[7];
   out_9076953633350933155[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6493249169715700303) {
   out_6493249169715700303[0] = -nom_x[0] + true_x[0];
   out_6493249169715700303[1] = -nom_x[1] + true_x[1];
   out_6493249169715700303[2] = -nom_x[2] + true_x[2];
   out_6493249169715700303[3] = -nom_x[3] + true_x[3];
   out_6493249169715700303[4] = -nom_x[4] + true_x[4];
   out_6493249169715700303[5] = -nom_x[5] + true_x[5];
   out_6493249169715700303[6] = -nom_x[6] + true_x[6];
   out_6493249169715700303[7] = -nom_x[7] + true_x[7];
   out_6493249169715700303[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6053903725589148651) {
   out_6053903725589148651[0] = 1.0;
   out_6053903725589148651[1] = 0;
   out_6053903725589148651[2] = 0;
   out_6053903725589148651[3] = 0;
   out_6053903725589148651[4] = 0;
   out_6053903725589148651[5] = 0;
   out_6053903725589148651[6] = 0;
   out_6053903725589148651[7] = 0;
   out_6053903725589148651[8] = 0;
   out_6053903725589148651[9] = 0;
   out_6053903725589148651[10] = 1.0;
   out_6053903725589148651[11] = 0;
   out_6053903725589148651[12] = 0;
   out_6053903725589148651[13] = 0;
   out_6053903725589148651[14] = 0;
   out_6053903725589148651[15] = 0;
   out_6053903725589148651[16] = 0;
   out_6053903725589148651[17] = 0;
   out_6053903725589148651[18] = 0;
   out_6053903725589148651[19] = 0;
   out_6053903725589148651[20] = 1.0;
   out_6053903725589148651[21] = 0;
   out_6053903725589148651[22] = 0;
   out_6053903725589148651[23] = 0;
   out_6053903725589148651[24] = 0;
   out_6053903725589148651[25] = 0;
   out_6053903725589148651[26] = 0;
   out_6053903725589148651[27] = 0;
   out_6053903725589148651[28] = 0;
   out_6053903725589148651[29] = 0;
   out_6053903725589148651[30] = 1.0;
   out_6053903725589148651[31] = 0;
   out_6053903725589148651[32] = 0;
   out_6053903725589148651[33] = 0;
   out_6053903725589148651[34] = 0;
   out_6053903725589148651[35] = 0;
   out_6053903725589148651[36] = 0;
   out_6053903725589148651[37] = 0;
   out_6053903725589148651[38] = 0;
   out_6053903725589148651[39] = 0;
   out_6053903725589148651[40] = 1.0;
   out_6053903725589148651[41] = 0;
   out_6053903725589148651[42] = 0;
   out_6053903725589148651[43] = 0;
   out_6053903725589148651[44] = 0;
   out_6053903725589148651[45] = 0;
   out_6053903725589148651[46] = 0;
   out_6053903725589148651[47] = 0;
   out_6053903725589148651[48] = 0;
   out_6053903725589148651[49] = 0;
   out_6053903725589148651[50] = 1.0;
   out_6053903725589148651[51] = 0;
   out_6053903725589148651[52] = 0;
   out_6053903725589148651[53] = 0;
   out_6053903725589148651[54] = 0;
   out_6053903725589148651[55] = 0;
   out_6053903725589148651[56] = 0;
   out_6053903725589148651[57] = 0;
   out_6053903725589148651[58] = 0;
   out_6053903725589148651[59] = 0;
   out_6053903725589148651[60] = 1.0;
   out_6053903725589148651[61] = 0;
   out_6053903725589148651[62] = 0;
   out_6053903725589148651[63] = 0;
   out_6053903725589148651[64] = 0;
   out_6053903725589148651[65] = 0;
   out_6053903725589148651[66] = 0;
   out_6053903725589148651[67] = 0;
   out_6053903725589148651[68] = 0;
   out_6053903725589148651[69] = 0;
   out_6053903725589148651[70] = 1.0;
   out_6053903725589148651[71] = 0;
   out_6053903725589148651[72] = 0;
   out_6053903725589148651[73] = 0;
   out_6053903725589148651[74] = 0;
   out_6053903725589148651[75] = 0;
   out_6053903725589148651[76] = 0;
   out_6053903725589148651[77] = 0;
   out_6053903725589148651[78] = 0;
   out_6053903725589148651[79] = 0;
   out_6053903725589148651[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7510396694923395417) {
   out_7510396694923395417[0] = state[0];
   out_7510396694923395417[1] = state[1];
   out_7510396694923395417[2] = state[2];
   out_7510396694923395417[3] = state[3];
   out_7510396694923395417[4] = state[4];
   out_7510396694923395417[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7510396694923395417[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7510396694923395417[7] = state[7];
   out_7510396694923395417[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4314618042520952482) {
   out_4314618042520952482[0] = 1;
   out_4314618042520952482[1] = 0;
   out_4314618042520952482[2] = 0;
   out_4314618042520952482[3] = 0;
   out_4314618042520952482[4] = 0;
   out_4314618042520952482[5] = 0;
   out_4314618042520952482[6] = 0;
   out_4314618042520952482[7] = 0;
   out_4314618042520952482[8] = 0;
   out_4314618042520952482[9] = 0;
   out_4314618042520952482[10] = 1;
   out_4314618042520952482[11] = 0;
   out_4314618042520952482[12] = 0;
   out_4314618042520952482[13] = 0;
   out_4314618042520952482[14] = 0;
   out_4314618042520952482[15] = 0;
   out_4314618042520952482[16] = 0;
   out_4314618042520952482[17] = 0;
   out_4314618042520952482[18] = 0;
   out_4314618042520952482[19] = 0;
   out_4314618042520952482[20] = 1;
   out_4314618042520952482[21] = 0;
   out_4314618042520952482[22] = 0;
   out_4314618042520952482[23] = 0;
   out_4314618042520952482[24] = 0;
   out_4314618042520952482[25] = 0;
   out_4314618042520952482[26] = 0;
   out_4314618042520952482[27] = 0;
   out_4314618042520952482[28] = 0;
   out_4314618042520952482[29] = 0;
   out_4314618042520952482[30] = 1;
   out_4314618042520952482[31] = 0;
   out_4314618042520952482[32] = 0;
   out_4314618042520952482[33] = 0;
   out_4314618042520952482[34] = 0;
   out_4314618042520952482[35] = 0;
   out_4314618042520952482[36] = 0;
   out_4314618042520952482[37] = 0;
   out_4314618042520952482[38] = 0;
   out_4314618042520952482[39] = 0;
   out_4314618042520952482[40] = 1;
   out_4314618042520952482[41] = 0;
   out_4314618042520952482[42] = 0;
   out_4314618042520952482[43] = 0;
   out_4314618042520952482[44] = 0;
   out_4314618042520952482[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4314618042520952482[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4314618042520952482[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4314618042520952482[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4314618042520952482[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4314618042520952482[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4314618042520952482[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4314618042520952482[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4314618042520952482[53] = -9.8000000000000007*dt;
   out_4314618042520952482[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4314618042520952482[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4314618042520952482[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4314618042520952482[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4314618042520952482[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4314618042520952482[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4314618042520952482[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4314618042520952482[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4314618042520952482[62] = 0;
   out_4314618042520952482[63] = 0;
   out_4314618042520952482[64] = 0;
   out_4314618042520952482[65] = 0;
   out_4314618042520952482[66] = 0;
   out_4314618042520952482[67] = 0;
   out_4314618042520952482[68] = 0;
   out_4314618042520952482[69] = 0;
   out_4314618042520952482[70] = 1;
   out_4314618042520952482[71] = 0;
   out_4314618042520952482[72] = 0;
   out_4314618042520952482[73] = 0;
   out_4314618042520952482[74] = 0;
   out_4314618042520952482[75] = 0;
   out_4314618042520952482[76] = 0;
   out_4314618042520952482[77] = 0;
   out_4314618042520952482[78] = 0;
   out_4314618042520952482[79] = 0;
   out_4314618042520952482[80] = 1;
}
void h_25(double *state, double *unused, double *out_7579285510236742280) {
   out_7579285510236742280[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5043007725208902189) {
   out_5043007725208902189[0] = 0;
   out_5043007725208902189[1] = 0;
   out_5043007725208902189[2] = 0;
   out_5043007725208902189[3] = 0;
   out_5043007725208902189[4] = 0;
   out_5043007725208902189[5] = 0;
   out_5043007725208902189[6] = 1;
   out_5043007725208902189[7] = 0;
   out_5043007725208902189[8] = 0;
}
void h_24(double *state, double *unused, double *out_6868631688394465622) {
   out_6868631688394465622[0] = state[4];
   out_6868631688394465622[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1485082126406579053) {
   out_1485082126406579053[0] = 0;
   out_1485082126406579053[1] = 0;
   out_1485082126406579053[2] = 0;
   out_1485082126406579053[3] = 0;
   out_1485082126406579053[4] = 1;
   out_1485082126406579053[5] = 0;
   out_1485082126406579053[6] = 0;
   out_1485082126406579053[7] = 0;
   out_1485082126406579053[8] = 0;
   out_1485082126406579053[9] = 0;
   out_1485082126406579053[10] = 0;
   out_1485082126406579053[11] = 0;
   out_1485082126406579053[12] = 0;
   out_1485082126406579053[13] = 0;
   out_1485082126406579053[14] = 1;
   out_1485082126406579053[15] = 0;
   out_1485082126406579053[16] = 0;
   out_1485082126406579053[17] = 0;
}
void h_30(double *state, double *unused, double *out_9029916902865625447) {
   out_9029916902865625447[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1873682616282714566) {
   out_1873682616282714566[0] = 0;
   out_1873682616282714566[1] = 0;
   out_1873682616282714566[2] = 0;
   out_1873682616282714566[3] = 0;
   out_1873682616282714566[4] = 1;
   out_1873682616282714566[5] = 0;
   out_1873682616282714566[6] = 0;
   out_1873682616282714566[7] = 0;
   out_1873682616282714566[8] = 0;
}
void h_26(double *state, double *unused, double *out_8086407049963963012) {
   out_8086407049963963012[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8784511044082958413) {
   out_8784511044082958413[0] = 0;
   out_8784511044082958413[1] = 0;
   out_8784511044082958413[2] = 0;
   out_8784511044082958413[3] = 0;
   out_8784511044082958413[4] = 0;
   out_8784511044082958413[5] = 0;
   out_8784511044082958413[6] = 0;
   out_8784511044082958413[7] = 1;
   out_8784511044082958413[8] = 0;
}
void h_27(double *state, double *unused, double *out_4173763459425434773) {
   out_4173763459425434773[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7347109984152567170) {
   out_7347109984152567170[0] = 0;
   out_7347109984152567170[1] = 0;
   out_7347109984152567170[2] = 0;
   out_7347109984152567170[3] = 1;
   out_7347109984152567170[4] = 0;
   out_7347109984152567170[5] = 0;
   out_7347109984152567170[6] = 0;
   out_7347109984152567170[7] = 0;
   out_7347109984152567170[8] = 0;
}
void h_29(double *state, double *unused, double *out_4028243635204779802) {
   out_4028243635204779802[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9060472711022118203) {
   out_9060472711022118203[0] = 0;
   out_9060472711022118203[1] = 1;
   out_9060472711022118203[2] = 0;
   out_9060472711022118203[3] = 0;
   out_9060472711022118203[4] = 0;
   out_9060472711022118203[5] = 0;
   out_9060472711022118203[6] = 0;
   out_9060472711022118203[7] = 0;
   out_9060472711022118203[8] = 0;
}
void h_28(double *state, double *unused, double *out_1052143756409890174) {
   out_1052143756409890174[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7096842439456791952) {
   out_7096842439456791952[0] = 1;
   out_7096842439456791952[1] = 0;
   out_7096842439456791952[2] = 0;
   out_7096842439456791952[3] = 0;
   out_7096842439456791952[4] = 0;
   out_7096842439456791952[5] = 0;
   out_7096842439456791952[6] = 0;
   out_7096842439456791952[7] = 0;
   out_7096842439456791952[8] = 0;
}
void h_31(double *state, double *unused, double *out_2986185912718760151) {
   out_2986185912718760151[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5012361763331941761) {
   out_5012361763331941761[0] = 0;
   out_5012361763331941761[1] = 0;
   out_5012361763331941761[2] = 0;
   out_5012361763331941761[3] = 0;
   out_5012361763331941761[4] = 0;
   out_5012361763331941761[5] = 0;
   out_5012361763331941761[6] = 0;
   out_5012361763331941761[7] = 0;
   out_5012361763331941761[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_9076953633350933155) {
  err_fun(nom_x, delta_x, out_9076953633350933155);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6493249169715700303) {
  inv_err_fun(nom_x, true_x, out_6493249169715700303);
}
void car_H_mod_fun(double *state, double *out_6053903725589148651) {
  H_mod_fun(state, out_6053903725589148651);
}
void car_f_fun(double *state, double dt, double *out_7510396694923395417) {
  f_fun(state,  dt, out_7510396694923395417);
}
void car_F_fun(double *state, double dt, double *out_4314618042520952482) {
  F_fun(state,  dt, out_4314618042520952482);
}
void car_h_25(double *state, double *unused, double *out_7579285510236742280) {
  h_25(state, unused, out_7579285510236742280);
}
void car_H_25(double *state, double *unused, double *out_5043007725208902189) {
  H_25(state, unused, out_5043007725208902189);
}
void car_h_24(double *state, double *unused, double *out_6868631688394465622) {
  h_24(state, unused, out_6868631688394465622);
}
void car_H_24(double *state, double *unused, double *out_1485082126406579053) {
  H_24(state, unused, out_1485082126406579053);
}
void car_h_30(double *state, double *unused, double *out_9029916902865625447) {
  h_30(state, unused, out_9029916902865625447);
}
void car_H_30(double *state, double *unused, double *out_1873682616282714566) {
  H_30(state, unused, out_1873682616282714566);
}
void car_h_26(double *state, double *unused, double *out_8086407049963963012) {
  h_26(state, unused, out_8086407049963963012);
}
void car_H_26(double *state, double *unused, double *out_8784511044082958413) {
  H_26(state, unused, out_8784511044082958413);
}
void car_h_27(double *state, double *unused, double *out_4173763459425434773) {
  h_27(state, unused, out_4173763459425434773);
}
void car_H_27(double *state, double *unused, double *out_7347109984152567170) {
  H_27(state, unused, out_7347109984152567170);
}
void car_h_29(double *state, double *unused, double *out_4028243635204779802) {
  h_29(state, unused, out_4028243635204779802);
}
void car_H_29(double *state, double *unused, double *out_9060472711022118203) {
  H_29(state, unused, out_9060472711022118203);
}
void car_h_28(double *state, double *unused, double *out_1052143756409890174) {
  h_28(state, unused, out_1052143756409890174);
}
void car_H_28(double *state, double *unused, double *out_7096842439456791952) {
  H_28(state, unused, out_7096842439456791952);
}
void car_h_31(double *state, double *unused, double *out_2986185912718760151) {
  h_31(state, unused, out_2986185912718760151);
}
void car_H_31(double *state, double *unused, double *out_5012361763331941761) {
  H_31(state, unused, out_5012361763331941761);
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
