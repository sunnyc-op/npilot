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
void err_fun(double *nom_x, double *delta_x, double *out_3405411849062960531) {
   out_3405411849062960531[0] = delta_x[0] + nom_x[0];
   out_3405411849062960531[1] = delta_x[1] + nom_x[1];
   out_3405411849062960531[2] = delta_x[2] + nom_x[2];
   out_3405411849062960531[3] = delta_x[3] + nom_x[3];
   out_3405411849062960531[4] = delta_x[4] + nom_x[4];
   out_3405411849062960531[5] = delta_x[5] + nom_x[5];
   out_3405411849062960531[6] = delta_x[6] + nom_x[6];
   out_3405411849062960531[7] = delta_x[7] + nom_x[7];
   out_3405411849062960531[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5631930522123645329) {
   out_5631930522123645329[0] = -nom_x[0] + true_x[0];
   out_5631930522123645329[1] = -nom_x[1] + true_x[1];
   out_5631930522123645329[2] = -nom_x[2] + true_x[2];
   out_5631930522123645329[3] = -nom_x[3] + true_x[3];
   out_5631930522123645329[4] = -nom_x[4] + true_x[4];
   out_5631930522123645329[5] = -nom_x[5] + true_x[5];
   out_5631930522123645329[6] = -nom_x[6] + true_x[6];
   out_5631930522123645329[7] = -nom_x[7] + true_x[7];
   out_5631930522123645329[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2249019235334351941) {
   out_2249019235334351941[0] = 1.0;
   out_2249019235334351941[1] = 0;
   out_2249019235334351941[2] = 0;
   out_2249019235334351941[3] = 0;
   out_2249019235334351941[4] = 0;
   out_2249019235334351941[5] = 0;
   out_2249019235334351941[6] = 0;
   out_2249019235334351941[7] = 0;
   out_2249019235334351941[8] = 0;
   out_2249019235334351941[9] = 0;
   out_2249019235334351941[10] = 1.0;
   out_2249019235334351941[11] = 0;
   out_2249019235334351941[12] = 0;
   out_2249019235334351941[13] = 0;
   out_2249019235334351941[14] = 0;
   out_2249019235334351941[15] = 0;
   out_2249019235334351941[16] = 0;
   out_2249019235334351941[17] = 0;
   out_2249019235334351941[18] = 0;
   out_2249019235334351941[19] = 0;
   out_2249019235334351941[20] = 1.0;
   out_2249019235334351941[21] = 0;
   out_2249019235334351941[22] = 0;
   out_2249019235334351941[23] = 0;
   out_2249019235334351941[24] = 0;
   out_2249019235334351941[25] = 0;
   out_2249019235334351941[26] = 0;
   out_2249019235334351941[27] = 0;
   out_2249019235334351941[28] = 0;
   out_2249019235334351941[29] = 0;
   out_2249019235334351941[30] = 1.0;
   out_2249019235334351941[31] = 0;
   out_2249019235334351941[32] = 0;
   out_2249019235334351941[33] = 0;
   out_2249019235334351941[34] = 0;
   out_2249019235334351941[35] = 0;
   out_2249019235334351941[36] = 0;
   out_2249019235334351941[37] = 0;
   out_2249019235334351941[38] = 0;
   out_2249019235334351941[39] = 0;
   out_2249019235334351941[40] = 1.0;
   out_2249019235334351941[41] = 0;
   out_2249019235334351941[42] = 0;
   out_2249019235334351941[43] = 0;
   out_2249019235334351941[44] = 0;
   out_2249019235334351941[45] = 0;
   out_2249019235334351941[46] = 0;
   out_2249019235334351941[47] = 0;
   out_2249019235334351941[48] = 0;
   out_2249019235334351941[49] = 0;
   out_2249019235334351941[50] = 1.0;
   out_2249019235334351941[51] = 0;
   out_2249019235334351941[52] = 0;
   out_2249019235334351941[53] = 0;
   out_2249019235334351941[54] = 0;
   out_2249019235334351941[55] = 0;
   out_2249019235334351941[56] = 0;
   out_2249019235334351941[57] = 0;
   out_2249019235334351941[58] = 0;
   out_2249019235334351941[59] = 0;
   out_2249019235334351941[60] = 1.0;
   out_2249019235334351941[61] = 0;
   out_2249019235334351941[62] = 0;
   out_2249019235334351941[63] = 0;
   out_2249019235334351941[64] = 0;
   out_2249019235334351941[65] = 0;
   out_2249019235334351941[66] = 0;
   out_2249019235334351941[67] = 0;
   out_2249019235334351941[68] = 0;
   out_2249019235334351941[69] = 0;
   out_2249019235334351941[70] = 1.0;
   out_2249019235334351941[71] = 0;
   out_2249019235334351941[72] = 0;
   out_2249019235334351941[73] = 0;
   out_2249019235334351941[74] = 0;
   out_2249019235334351941[75] = 0;
   out_2249019235334351941[76] = 0;
   out_2249019235334351941[77] = 0;
   out_2249019235334351941[78] = 0;
   out_2249019235334351941[79] = 0;
   out_2249019235334351941[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1970153544911791805) {
   out_1970153544911791805[0] = state[0];
   out_1970153544911791805[1] = state[1];
   out_1970153544911791805[2] = state[2];
   out_1970153544911791805[3] = state[3];
   out_1970153544911791805[4] = state[4];
   out_1970153544911791805[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1970153544911791805[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1970153544911791805[7] = state[7];
   out_1970153544911791805[8] = state[8];
}
void F_fun(double *state, double dt, double *out_117042200783792997) {
   out_117042200783792997[0] = 1;
   out_117042200783792997[1] = 0;
   out_117042200783792997[2] = 0;
   out_117042200783792997[3] = 0;
   out_117042200783792997[4] = 0;
   out_117042200783792997[5] = 0;
   out_117042200783792997[6] = 0;
   out_117042200783792997[7] = 0;
   out_117042200783792997[8] = 0;
   out_117042200783792997[9] = 0;
   out_117042200783792997[10] = 1;
   out_117042200783792997[11] = 0;
   out_117042200783792997[12] = 0;
   out_117042200783792997[13] = 0;
   out_117042200783792997[14] = 0;
   out_117042200783792997[15] = 0;
   out_117042200783792997[16] = 0;
   out_117042200783792997[17] = 0;
   out_117042200783792997[18] = 0;
   out_117042200783792997[19] = 0;
   out_117042200783792997[20] = 1;
   out_117042200783792997[21] = 0;
   out_117042200783792997[22] = 0;
   out_117042200783792997[23] = 0;
   out_117042200783792997[24] = 0;
   out_117042200783792997[25] = 0;
   out_117042200783792997[26] = 0;
   out_117042200783792997[27] = 0;
   out_117042200783792997[28] = 0;
   out_117042200783792997[29] = 0;
   out_117042200783792997[30] = 1;
   out_117042200783792997[31] = 0;
   out_117042200783792997[32] = 0;
   out_117042200783792997[33] = 0;
   out_117042200783792997[34] = 0;
   out_117042200783792997[35] = 0;
   out_117042200783792997[36] = 0;
   out_117042200783792997[37] = 0;
   out_117042200783792997[38] = 0;
   out_117042200783792997[39] = 0;
   out_117042200783792997[40] = 1;
   out_117042200783792997[41] = 0;
   out_117042200783792997[42] = 0;
   out_117042200783792997[43] = 0;
   out_117042200783792997[44] = 0;
   out_117042200783792997[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_117042200783792997[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_117042200783792997[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_117042200783792997[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_117042200783792997[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_117042200783792997[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_117042200783792997[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_117042200783792997[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_117042200783792997[53] = -9.8000000000000007*dt;
   out_117042200783792997[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_117042200783792997[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_117042200783792997[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_117042200783792997[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_117042200783792997[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_117042200783792997[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_117042200783792997[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_117042200783792997[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_117042200783792997[62] = 0;
   out_117042200783792997[63] = 0;
   out_117042200783792997[64] = 0;
   out_117042200783792997[65] = 0;
   out_117042200783792997[66] = 0;
   out_117042200783792997[67] = 0;
   out_117042200783792997[68] = 0;
   out_117042200783792997[69] = 0;
   out_117042200783792997[70] = 1;
   out_117042200783792997[71] = 0;
   out_117042200783792997[72] = 0;
   out_117042200783792997[73] = 0;
   out_117042200783792997[74] = 0;
   out_117042200783792997[75] = 0;
   out_117042200783792997[76] = 0;
   out_117042200783792997[77] = 0;
   out_117042200783792997[78] = 0;
   out_117042200783792997[79] = 0;
   out_117042200783792997[80] = 1;
}
void h_25(double *state, double *unused, double *out_2527274791798040583) {
   out_2527274791798040583[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4939095172763981300) {
   out_4939095172763981300[0] = 0;
   out_4939095172763981300[1] = 0;
   out_4939095172763981300[2] = 0;
   out_4939095172763981300[3] = 0;
   out_4939095172763981300[4] = 0;
   out_4939095172763981300[5] = 0;
   out_4939095172763981300[6] = 1;
   out_4939095172763981300[7] = 0;
   out_4939095172763981300[8] = 0;
}
void h_24(double *state, double *unused, double *out_3993072292709402740) {
   out_3993072292709402740[0] = state[4];
   out_3993072292709402740[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2761880749156831327) {
   out_2761880749156831327[0] = 0;
   out_2761880749156831327[1] = 0;
   out_2761880749156831327[2] = 0;
   out_2761880749156831327[3] = 0;
   out_2761880749156831327[4] = 1;
   out_2761880749156831327[5] = 0;
   out_2761880749156831327[6] = 0;
   out_2761880749156831327[7] = 0;
   out_2761880749156831327[8] = 0;
   out_2761880749156831327[9] = 0;
   out_2761880749156831327[10] = 0;
   out_2761880749156831327[11] = 0;
   out_2761880749156831327[12] = 0;
   out_2761880749156831327[13] = 0;
   out_2761880749156831327[14] = 1;
   out_2761880749156831327[15] = 0;
   out_2761880749156831327[16] = 0;
   out_2761880749156831327[17] = 0;
}
void h_30(double *state, double *unused, double *out_6164724320052062933) {
   out_6164724320052062933[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2420762214256732673) {
   out_2420762214256732673[0] = 0;
   out_2420762214256732673[1] = 0;
   out_2420762214256732673[2] = 0;
   out_2420762214256732673[3] = 0;
   out_2420762214256732673[4] = 1;
   out_2420762214256732673[5] = 0;
   out_2420762214256732673[6] = 0;
   out_2420762214256732673[7] = 0;
   out_2420762214256732673[8] = 0;
}
void h_26(double *state, double *unused, double *out_1963597352928310702) {
   out_1963597352928310702[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8680598491638037524) {
   out_8680598491638037524[0] = 0;
   out_8680598491638037524[1] = 0;
   out_8680598491638037524[2] = 0;
   out_8680598491638037524[3] = 0;
   out_8680598491638037524[4] = 0;
   out_8680598491638037524[5] = 0;
   out_8680598491638037524[6] = 0;
   out_8680598491638037524[7] = 1;
   out_8680598491638037524[8] = 0;
}
void h_27(double *state, double *unused, double *out_2081604746861687446) {
   out_2081604746861687446[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4595525526057157584) {
   out_4595525526057157584[0] = 0;
   out_4595525526057157584[1] = 0;
   out_4595525526057157584[2] = 0;
   out_4595525526057157584[3] = 1;
   out_4595525526057157584[4] = 0;
   out_4595525526057157584[5] = 0;
   out_4595525526057157584[6] = 0;
   out_4595525526057157584[7] = 0;
   out_4595525526057157584[8] = 0;
}
void h_29(double *state, double *unused, double *out_7056112537822163736) {
   out_7056112537822163736[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1910530869942340489) {
   out_1910530869942340489[0] = 0;
   out_1910530869942340489[1] = 1;
   out_1910530869942340489[2] = 0;
   out_1910530869942340489[3] = 0;
   out_1910530869942340489[4] = 0;
   out_1910530869942340489[5] = 0;
   out_1910530869942340489[6] = 0;
   out_1910530869942340489[7] = 0;
   out_1910530869942340489[8] = 0;
}
void h_28(double *state, double *unused, double *out_4112674769809816212) {
   out_4112674769809816212[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6992929887011871063) {
   out_6992929887011871063[0] = 1;
   out_6992929887011871063[1] = 0;
   out_6992929887011871063[2] = 0;
   out_6992929887011871063[3] = 0;
   out_6992929887011871063[4] = 0;
   out_6992929887011871063[5] = 0;
   out_6992929887011871063[6] = 0;
   out_6992929887011871063[7] = 0;
   out_6992929887011871063[8] = 0;
}
void h_31(double *state, double *unused, double *out_4440040372882991911) {
   out_4440040372882991911[0] = state[8];
}
void H_31(double *state, double *unused, double *out_9139937479838162616) {
   out_9139937479838162616[0] = 0;
   out_9139937479838162616[1] = 0;
   out_9139937479838162616[2] = 0;
   out_9139937479838162616[3] = 0;
   out_9139937479838162616[4] = 0;
   out_9139937479838162616[5] = 0;
   out_9139937479838162616[6] = 0;
   out_9139937479838162616[7] = 0;
   out_9139937479838162616[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3405411849062960531) {
  err_fun(nom_x, delta_x, out_3405411849062960531);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5631930522123645329) {
  inv_err_fun(nom_x, true_x, out_5631930522123645329);
}
void car_H_mod_fun(double *state, double *out_2249019235334351941) {
  H_mod_fun(state, out_2249019235334351941);
}
void car_f_fun(double *state, double dt, double *out_1970153544911791805) {
  f_fun(state,  dt, out_1970153544911791805);
}
void car_F_fun(double *state, double dt, double *out_117042200783792997) {
  F_fun(state,  dt, out_117042200783792997);
}
void car_h_25(double *state, double *unused, double *out_2527274791798040583) {
  h_25(state, unused, out_2527274791798040583);
}
void car_H_25(double *state, double *unused, double *out_4939095172763981300) {
  H_25(state, unused, out_4939095172763981300);
}
void car_h_24(double *state, double *unused, double *out_3993072292709402740) {
  h_24(state, unused, out_3993072292709402740);
}
void car_H_24(double *state, double *unused, double *out_2761880749156831327) {
  H_24(state, unused, out_2761880749156831327);
}
void car_h_30(double *state, double *unused, double *out_6164724320052062933) {
  h_30(state, unused, out_6164724320052062933);
}
void car_H_30(double *state, double *unused, double *out_2420762214256732673) {
  H_30(state, unused, out_2420762214256732673);
}
void car_h_26(double *state, double *unused, double *out_1963597352928310702) {
  h_26(state, unused, out_1963597352928310702);
}
void car_H_26(double *state, double *unused, double *out_8680598491638037524) {
  H_26(state, unused, out_8680598491638037524);
}
void car_h_27(double *state, double *unused, double *out_2081604746861687446) {
  h_27(state, unused, out_2081604746861687446);
}
void car_H_27(double *state, double *unused, double *out_4595525526057157584) {
  H_27(state, unused, out_4595525526057157584);
}
void car_h_29(double *state, double *unused, double *out_7056112537822163736) {
  h_29(state, unused, out_7056112537822163736);
}
void car_H_29(double *state, double *unused, double *out_1910530869942340489) {
  H_29(state, unused, out_1910530869942340489);
}
void car_h_28(double *state, double *unused, double *out_4112674769809816212) {
  h_28(state, unused, out_4112674769809816212);
}
void car_H_28(double *state, double *unused, double *out_6992929887011871063) {
  H_28(state, unused, out_6992929887011871063);
}
void car_h_31(double *state, double *unused, double *out_4440040372882991911) {
  h_31(state, unused, out_4440040372882991911);
}
void car_H_31(double *state, double *unused, double *out_9139937479838162616) {
  H_31(state, unused, out_9139937479838162616);
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
