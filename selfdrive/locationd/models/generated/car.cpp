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
void err_fun(double *nom_x, double *delta_x, double *out_1587320949155634031) {
   out_1587320949155634031[0] = delta_x[0] + nom_x[0];
   out_1587320949155634031[1] = delta_x[1] + nom_x[1];
   out_1587320949155634031[2] = delta_x[2] + nom_x[2];
   out_1587320949155634031[3] = delta_x[3] + nom_x[3];
   out_1587320949155634031[4] = delta_x[4] + nom_x[4];
   out_1587320949155634031[5] = delta_x[5] + nom_x[5];
   out_1587320949155634031[6] = delta_x[6] + nom_x[6];
   out_1587320949155634031[7] = delta_x[7] + nom_x[7];
   out_1587320949155634031[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7610236178695361162) {
   out_7610236178695361162[0] = -nom_x[0] + true_x[0];
   out_7610236178695361162[1] = -nom_x[1] + true_x[1];
   out_7610236178695361162[2] = -nom_x[2] + true_x[2];
   out_7610236178695361162[3] = -nom_x[3] + true_x[3];
   out_7610236178695361162[4] = -nom_x[4] + true_x[4];
   out_7610236178695361162[5] = -nom_x[5] + true_x[5];
   out_7610236178695361162[6] = -nom_x[6] + true_x[6];
   out_7610236178695361162[7] = -nom_x[7] + true_x[7];
   out_7610236178695361162[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2153325158078460112) {
   out_2153325158078460112[0] = 1.0;
   out_2153325158078460112[1] = 0;
   out_2153325158078460112[2] = 0;
   out_2153325158078460112[3] = 0;
   out_2153325158078460112[4] = 0;
   out_2153325158078460112[5] = 0;
   out_2153325158078460112[6] = 0;
   out_2153325158078460112[7] = 0;
   out_2153325158078460112[8] = 0;
   out_2153325158078460112[9] = 0;
   out_2153325158078460112[10] = 1.0;
   out_2153325158078460112[11] = 0;
   out_2153325158078460112[12] = 0;
   out_2153325158078460112[13] = 0;
   out_2153325158078460112[14] = 0;
   out_2153325158078460112[15] = 0;
   out_2153325158078460112[16] = 0;
   out_2153325158078460112[17] = 0;
   out_2153325158078460112[18] = 0;
   out_2153325158078460112[19] = 0;
   out_2153325158078460112[20] = 1.0;
   out_2153325158078460112[21] = 0;
   out_2153325158078460112[22] = 0;
   out_2153325158078460112[23] = 0;
   out_2153325158078460112[24] = 0;
   out_2153325158078460112[25] = 0;
   out_2153325158078460112[26] = 0;
   out_2153325158078460112[27] = 0;
   out_2153325158078460112[28] = 0;
   out_2153325158078460112[29] = 0;
   out_2153325158078460112[30] = 1.0;
   out_2153325158078460112[31] = 0;
   out_2153325158078460112[32] = 0;
   out_2153325158078460112[33] = 0;
   out_2153325158078460112[34] = 0;
   out_2153325158078460112[35] = 0;
   out_2153325158078460112[36] = 0;
   out_2153325158078460112[37] = 0;
   out_2153325158078460112[38] = 0;
   out_2153325158078460112[39] = 0;
   out_2153325158078460112[40] = 1.0;
   out_2153325158078460112[41] = 0;
   out_2153325158078460112[42] = 0;
   out_2153325158078460112[43] = 0;
   out_2153325158078460112[44] = 0;
   out_2153325158078460112[45] = 0;
   out_2153325158078460112[46] = 0;
   out_2153325158078460112[47] = 0;
   out_2153325158078460112[48] = 0;
   out_2153325158078460112[49] = 0;
   out_2153325158078460112[50] = 1.0;
   out_2153325158078460112[51] = 0;
   out_2153325158078460112[52] = 0;
   out_2153325158078460112[53] = 0;
   out_2153325158078460112[54] = 0;
   out_2153325158078460112[55] = 0;
   out_2153325158078460112[56] = 0;
   out_2153325158078460112[57] = 0;
   out_2153325158078460112[58] = 0;
   out_2153325158078460112[59] = 0;
   out_2153325158078460112[60] = 1.0;
   out_2153325158078460112[61] = 0;
   out_2153325158078460112[62] = 0;
   out_2153325158078460112[63] = 0;
   out_2153325158078460112[64] = 0;
   out_2153325158078460112[65] = 0;
   out_2153325158078460112[66] = 0;
   out_2153325158078460112[67] = 0;
   out_2153325158078460112[68] = 0;
   out_2153325158078460112[69] = 0;
   out_2153325158078460112[70] = 1.0;
   out_2153325158078460112[71] = 0;
   out_2153325158078460112[72] = 0;
   out_2153325158078460112[73] = 0;
   out_2153325158078460112[74] = 0;
   out_2153325158078460112[75] = 0;
   out_2153325158078460112[76] = 0;
   out_2153325158078460112[77] = 0;
   out_2153325158078460112[78] = 0;
   out_2153325158078460112[79] = 0;
   out_2153325158078460112[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1127301623673412374) {
   out_1127301623673412374[0] = state[0];
   out_1127301623673412374[1] = state[1];
   out_1127301623673412374[2] = state[2];
   out_1127301623673412374[3] = state[3];
   out_1127301623673412374[4] = state[4];
   out_1127301623673412374[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1127301623673412374[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1127301623673412374[7] = state[7];
   out_1127301623673412374[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4211123534076608715) {
   out_4211123534076608715[0] = 1;
   out_4211123534076608715[1] = 0;
   out_4211123534076608715[2] = 0;
   out_4211123534076608715[3] = 0;
   out_4211123534076608715[4] = 0;
   out_4211123534076608715[5] = 0;
   out_4211123534076608715[6] = 0;
   out_4211123534076608715[7] = 0;
   out_4211123534076608715[8] = 0;
   out_4211123534076608715[9] = 0;
   out_4211123534076608715[10] = 1;
   out_4211123534076608715[11] = 0;
   out_4211123534076608715[12] = 0;
   out_4211123534076608715[13] = 0;
   out_4211123534076608715[14] = 0;
   out_4211123534076608715[15] = 0;
   out_4211123534076608715[16] = 0;
   out_4211123534076608715[17] = 0;
   out_4211123534076608715[18] = 0;
   out_4211123534076608715[19] = 0;
   out_4211123534076608715[20] = 1;
   out_4211123534076608715[21] = 0;
   out_4211123534076608715[22] = 0;
   out_4211123534076608715[23] = 0;
   out_4211123534076608715[24] = 0;
   out_4211123534076608715[25] = 0;
   out_4211123534076608715[26] = 0;
   out_4211123534076608715[27] = 0;
   out_4211123534076608715[28] = 0;
   out_4211123534076608715[29] = 0;
   out_4211123534076608715[30] = 1;
   out_4211123534076608715[31] = 0;
   out_4211123534076608715[32] = 0;
   out_4211123534076608715[33] = 0;
   out_4211123534076608715[34] = 0;
   out_4211123534076608715[35] = 0;
   out_4211123534076608715[36] = 0;
   out_4211123534076608715[37] = 0;
   out_4211123534076608715[38] = 0;
   out_4211123534076608715[39] = 0;
   out_4211123534076608715[40] = 1;
   out_4211123534076608715[41] = 0;
   out_4211123534076608715[42] = 0;
   out_4211123534076608715[43] = 0;
   out_4211123534076608715[44] = 0;
   out_4211123534076608715[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4211123534076608715[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4211123534076608715[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4211123534076608715[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4211123534076608715[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4211123534076608715[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4211123534076608715[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4211123534076608715[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4211123534076608715[53] = -9.8000000000000007*dt;
   out_4211123534076608715[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4211123534076608715[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4211123534076608715[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4211123534076608715[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4211123534076608715[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4211123534076608715[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4211123534076608715[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4211123534076608715[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4211123534076608715[62] = 0;
   out_4211123534076608715[63] = 0;
   out_4211123534076608715[64] = 0;
   out_4211123534076608715[65] = 0;
   out_4211123534076608715[66] = 0;
   out_4211123534076608715[67] = 0;
   out_4211123534076608715[68] = 0;
   out_4211123534076608715[69] = 0;
   out_4211123534076608715[70] = 1;
   out_4211123534076608715[71] = 0;
   out_4211123534076608715[72] = 0;
   out_4211123534076608715[73] = 0;
   out_4211123534076608715[74] = 0;
   out_4211123534076608715[75] = 0;
   out_4211123534076608715[76] = 0;
   out_4211123534076608715[77] = 0;
   out_4211123534076608715[78] = 0;
   out_4211123534076608715[79] = 0;
   out_4211123534076608715[80] = 1;
}
void h_25(double *state, double *unused, double *out_5804680800098537329) {
   out_5804680800098537329[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8042566109691692167) {
   out_8042566109691692167[0] = 0;
   out_8042566109691692167[1] = 0;
   out_8042566109691692167[2] = 0;
   out_8042566109691692167[3] = 0;
   out_8042566109691692167[4] = 0;
   out_8042566109691692167[5] = 0;
   out_8042566109691692167[6] = 1;
   out_8042566109691692167[7] = 0;
   out_8042566109691692167[8] = 0;
}
void h_24(double *state, double *unused, double *out_5424263938900647146) {
   out_5424263938900647146[0] = state[4];
   out_5424263938900647146[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6916074274185325760) {
   out_6916074274185325760[0] = 0;
   out_6916074274185325760[1] = 0;
   out_6916074274185325760[2] = 0;
   out_6916074274185325760[3] = 0;
   out_6916074274185325760[4] = 1;
   out_6916074274185325760[5] = 0;
   out_6916074274185325760[6] = 0;
   out_6916074274185325760[7] = 0;
   out_6916074274185325760[8] = 0;
   out_6916074274185325760[9] = 0;
   out_6916074274185325760[10] = 0;
   out_6916074274185325760[11] = 0;
   out_6916074274185325760[12] = 0;
   out_6916074274185325760[13] = 0;
   out_6916074274185325760[14] = 1;
   out_6916074274185325760[15] = 0;
   out_6916074274185325760[16] = 0;
   out_6916074274185325760[17] = 0;
}
void h_30(double *state, double *unused, double *out_6054235347134719124) {
   out_6054235347134719124[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5876481633890251251) {
   out_5876481633890251251[0] = 0;
   out_5876481633890251251[1] = 0;
   out_5876481633890251251[2] = 0;
   out_5876481633890251251[3] = 0;
   out_5876481633890251251[4] = 1;
   out_5876481633890251251[5] = 0;
   out_5876481633890251251[6] = 0;
   out_5876481633890251251[7] = 0;
   out_5876481633890251251[8] = 0;
}
void h_26(double *state, double *unused, double *out_7166601510739507545) {
   out_7166601510739507545[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6662674645143803225) {
   out_6662674645143803225[0] = 0;
   out_6662674645143803225[1] = 0;
   out_6662674645143803225[2] = 0;
   out_6662674645143803225[3] = 0;
   out_6662674645143803225[4] = 0;
   out_6662674645143803225[5] = 0;
   out_6662674645143803225[6] = 0;
   out_6662674645143803225[7] = 1;
   out_6662674645143803225[8] = 0;
}
void h_27(double *state, double *unused, double *out_1198081903694528450) {
   out_1198081903694528450[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3701718322089826340) {
   out_3701718322089826340[0] = 0;
   out_3701718322089826340[1] = 0;
   out_3701718322089826340[2] = 0;
   out_3701718322089826340[3] = 1;
   out_3701718322089826340[4] = 0;
   out_3701718322089826340[5] = 0;
   out_3701718322089826340[6] = 0;
   out_3701718322089826340[7] = 0;
   out_3701718322089826340[8] = 0;
}
void h_29(double *state, double *unused, double *out_9091021398579051809) {
   out_9091021398579051809[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6386712978204643435) {
   out_6386712978204643435[0] = 0;
   out_6386712978204643435[1] = 1;
   out_6386712978204643435[2] = 0;
   out_6386712978204643435[3] = 0;
   out_6386712978204643435[4] = 0;
   out_6386712978204643435[5] = 0;
   out_6386712978204643435[6] = 0;
   out_6386712978204643435[7] = 0;
   out_6386712978204643435[8] = 0;
}
void h_28(double *state, double *unused, double *out_2141591756596190885) {
   out_2141591756596190885[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1304313961135112861) {
   out_1304313961135112861[0] = 1;
   out_1304313961135112861[1] = 0;
   out_1304313961135112861[2] = 0;
   out_1304313961135112861[3] = 0;
   out_1304313961135112861[4] = 0;
   out_1304313961135112861[5] = 0;
   out_1304313961135112861[6] = 0;
   out_1304313961135112861[7] = 0;
   out_1304313961135112861[8] = 0;
}
void h_31(double *state, double *unused, double *out_5438847316625028317) {
   out_5438847316625028317[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6036466542910451749) {
   out_6036466542910451749[0] = 0;
   out_6036466542910451749[1] = 0;
   out_6036466542910451749[2] = 0;
   out_6036466542910451749[3] = 0;
   out_6036466542910451749[4] = 0;
   out_6036466542910451749[5] = 0;
   out_6036466542910451749[6] = 0;
   out_6036466542910451749[7] = 0;
   out_6036466542910451749[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1587320949155634031) {
  err_fun(nom_x, delta_x, out_1587320949155634031);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7610236178695361162) {
  inv_err_fun(nom_x, true_x, out_7610236178695361162);
}
void car_H_mod_fun(double *state, double *out_2153325158078460112) {
  H_mod_fun(state, out_2153325158078460112);
}
void car_f_fun(double *state, double dt, double *out_1127301623673412374) {
  f_fun(state,  dt, out_1127301623673412374);
}
void car_F_fun(double *state, double dt, double *out_4211123534076608715) {
  F_fun(state,  dt, out_4211123534076608715);
}
void car_h_25(double *state, double *unused, double *out_5804680800098537329) {
  h_25(state, unused, out_5804680800098537329);
}
void car_H_25(double *state, double *unused, double *out_8042566109691692167) {
  H_25(state, unused, out_8042566109691692167);
}
void car_h_24(double *state, double *unused, double *out_5424263938900647146) {
  h_24(state, unused, out_5424263938900647146);
}
void car_H_24(double *state, double *unused, double *out_6916074274185325760) {
  H_24(state, unused, out_6916074274185325760);
}
void car_h_30(double *state, double *unused, double *out_6054235347134719124) {
  h_30(state, unused, out_6054235347134719124);
}
void car_H_30(double *state, double *unused, double *out_5876481633890251251) {
  H_30(state, unused, out_5876481633890251251);
}
void car_h_26(double *state, double *unused, double *out_7166601510739507545) {
  h_26(state, unused, out_7166601510739507545);
}
void car_H_26(double *state, double *unused, double *out_6662674645143803225) {
  H_26(state, unused, out_6662674645143803225);
}
void car_h_27(double *state, double *unused, double *out_1198081903694528450) {
  h_27(state, unused, out_1198081903694528450);
}
void car_H_27(double *state, double *unused, double *out_3701718322089826340) {
  H_27(state, unused, out_3701718322089826340);
}
void car_h_29(double *state, double *unused, double *out_9091021398579051809) {
  h_29(state, unused, out_9091021398579051809);
}
void car_H_29(double *state, double *unused, double *out_6386712978204643435) {
  H_29(state, unused, out_6386712978204643435);
}
void car_h_28(double *state, double *unused, double *out_2141591756596190885) {
  h_28(state, unused, out_2141591756596190885);
}
void car_H_28(double *state, double *unused, double *out_1304313961135112861) {
  H_28(state, unused, out_1304313961135112861);
}
void car_h_31(double *state, double *unused, double *out_5438847316625028317) {
  h_31(state, unused, out_5438847316625028317);
}
void car_H_31(double *state, double *unused, double *out_6036466542910451749) {
  H_31(state, unused, out_6036466542910451749);
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
