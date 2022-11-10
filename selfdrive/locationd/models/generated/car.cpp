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
void err_fun(double *nom_x, double *delta_x, double *out_7307813249485127470) {
   out_7307813249485127470[0] = delta_x[0] + nom_x[0];
   out_7307813249485127470[1] = delta_x[1] + nom_x[1];
   out_7307813249485127470[2] = delta_x[2] + nom_x[2];
   out_7307813249485127470[3] = delta_x[3] + nom_x[3];
   out_7307813249485127470[4] = delta_x[4] + nom_x[4];
   out_7307813249485127470[5] = delta_x[5] + nom_x[5];
   out_7307813249485127470[6] = delta_x[6] + nom_x[6];
   out_7307813249485127470[7] = delta_x[7] + nom_x[7];
   out_7307813249485127470[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3794738096767726126) {
   out_3794738096767726126[0] = -nom_x[0] + true_x[0];
   out_3794738096767726126[1] = -nom_x[1] + true_x[1];
   out_3794738096767726126[2] = -nom_x[2] + true_x[2];
   out_3794738096767726126[3] = -nom_x[3] + true_x[3];
   out_3794738096767726126[4] = -nom_x[4] + true_x[4];
   out_3794738096767726126[5] = -nom_x[5] + true_x[5];
   out_3794738096767726126[6] = -nom_x[6] + true_x[6];
   out_3794738096767726126[7] = -nom_x[7] + true_x[7];
   out_3794738096767726126[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5274657662750201021) {
   out_5274657662750201021[0] = 1.0;
   out_5274657662750201021[1] = 0;
   out_5274657662750201021[2] = 0;
   out_5274657662750201021[3] = 0;
   out_5274657662750201021[4] = 0;
   out_5274657662750201021[5] = 0;
   out_5274657662750201021[6] = 0;
   out_5274657662750201021[7] = 0;
   out_5274657662750201021[8] = 0;
   out_5274657662750201021[9] = 0;
   out_5274657662750201021[10] = 1.0;
   out_5274657662750201021[11] = 0;
   out_5274657662750201021[12] = 0;
   out_5274657662750201021[13] = 0;
   out_5274657662750201021[14] = 0;
   out_5274657662750201021[15] = 0;
   out_5274657662750201021[16] = 0;
   out_5274657662750201021[17] = 0;
   out_5274657662750201021[18] = 0;
   out_5274657662750201021[19] = 0;
   out_5274657662750201021[20] = 1.0;
   out_5274657662750201021[21] = 0;
   out_5274657662750201021[22] = 0;
   out_5274657662750201021[23] = 0;
   out_5274657662750201021[24] = 0;
   out_5274657662750201021[25] = 0;
   out_5274657662750201021[26] = 0;
   out_5274657662750201021[27] = 0;
   out_5274657662750201021[28] = 0;
   out_5274657662750201021[29] = 0;
   out_5274657662750201021[30] = 1.0;
   out_5274657662750201021[31] = 0;
   out_5274657662750201021[32] = 0;
   out_5274657662750201021[33] = 0;
   out_5274657662750201021[34] = 0;
   out_5274657662750201021[35] = 0;
   out_5274657662750201021[36] = 0;
   out_5274657662750201021[37] = 0;
   out_5274657662750201021[38] = 0;
   out_5274657662750201021[39] = 0;
   out_5274657662750201021[40] = 1.0;
   out_5274657662750201021[41] = 0;
   out_5274657662750201021[42] = 0;
   out_5274657662750201021[43] = 0;
   out_5274657662750201021[44] = 0;
   out_5274657662750201021[45] = 0;
   out_5274657662750201021[46] = 0;
   out_5274657662750201021[47] = 0;
   out_5274657662750201021[48] = 0;
   out_5274657662750201021[49] = 0;
   out_5274657662750201021[50] = 1.0;
   out_5274657662750201021[51] = 0;
   out_5274657662750201021[52] = 0;
   out_5274657662750201021[53] = 0;
   out_5274657662750201021[54] = 0;
   out_5274657662750201021[55] = 0;
   out_5274657662750201021[56] = 0;
   out_5274657662750201021[57] = 0;
   out_5274657662750201021[58] = 0;
   out_5274657662750201021[59] = 0;
   out_5274657662750201021[60] = 1.0;
   out_5274657662750201021[61] = 0;
   out_5274657662750201021[62] = 0;
   out_5274657662750201021[63] = 0;
   out_5274657662750201021[64] = 0;
   out_5274657662750201021[65] = 0;
   out_5274657662750201021[66] = 0;
   out_5274657662750201021[67] = 0;
   out_5274657662750201021[68] = 0;
   out_5274657662750201021[69] = 0;
   out_5274657662750201021[70] = 1.0;
   out_5274657662750201021[71] = 0;
   out_5274657662750201021[72] = 0;
   out_5274657662750201021[73] = 0;
   out_5274657662750201021[74] = 0;
   out_5274657662750201021[75] = 0;
   out_5274657662750201021[76] = 0;
   out_5274657662750201021[77] = 0;
   out_5274657662750201021[78] = 0;
   out_5274657662750201021[79] = 0;
   out_5274657662750201021[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3406949635768903926) {
   out_3406949635768903926[0] = state[0];
   out_3406949635768903926[1] = state[1];
   out_3406949635768903926[2] = state[2];
   out_3406949635768903926[3] = state[3];
   out_3406949635768903926[4] = state[4];
   out_3406949635768903926[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3406949635768903926[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3406949635768903926[7] = state[7];
   out_3406949635768903926[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8623400185762976873) {
   out_8623400185762976873[0] = 1;
   out_8623400185762976873[1] = 0;
   out_8623400185762976873[2] = 0;
   out_8623400185762976873[3] = 0;
   out_8623400185762976873[4] = 0;
   out_8623400185762976873[5] = 0;
   out_8623400185762976873[6] = 0;
   out_8623400185762976873[7] = 0;
   out_8623400185762976873[8] = 0;
   out_8623400185762976873[9] = 0;
   out_8623400185762976873[10] = 1;
   out_8623400185762976873[11] = 0;
   out_8623400185762976873[12] = 0;
   out_8623400185762976873[13] = 0;
   out_8623400185762976873[14] = 0;
   out_8623400185762976873[15] = 0;
   out_8623400185762976873[16] = 0;
   out_8623400185762976873[17] = 0;
   out_8623400185762976873[18] = 0;
   out_8623400185762976873[19] = 0;
   out_8623400185762976873[20] = 1;
   out_8623400185762976873[21] = 0;
   out_8623400185762976873[22] = 0;
   out_8623400185762976873[23] = 0;
   out_8623400185762976873[24] = 0;
   out_8623400185762976873[25] = 0;
   out_8623400185762976873[26] = 0;
   out_8623400185762976873[27] = 0;
   out_8623400185762976873[28] = 0;
   out_8623400185762976873[29] = 0;
   out_8623400185762976873[30] = 1;
   out_8623400185762976873[31] = 0;
   out_8623400185762976873[32] = 0;
   out_8623400185762976873[33] = 0;
   out_8623400185762976873[34] = 0;
   out_8623400185762976873[35] = 0;
   out_8623400185762976873[36] = 0;
   out_8623400185762976873[37] = 0;
   out_8623400185762976873[38] = 0;
   out_8623400185762976873[39] = 0;
   out_8623400185762976873[40] = 1;
   out_8623400185762976873[41] = 0;
   out_8623400185762976873[42] = 0;
   out_8623400185762976873[43] = 0;
   out_8623400185762976873[44] = 0;
   out_8623400185762976873[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8623400185762976873[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8623400185762976873[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8623400185762976873[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8623400185762976873[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8623400185762976873[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8623400185762976873[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8623400185762976873[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8623400185762976873[53] = -9.8000000000000007*dt;
   out_8623400185762976873[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8623400185762976873[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8623400185762976873[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8623400185762976873[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8623400185762976873[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8623400185762976873[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8623400185762976873[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8623400185762976873[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8623400185762976873[62] = 0;
   out_8623400185762976873[63] = 0;
   out_8623400185762976873[64] = 0;
   out_8623400185762976873[65] = 0;
   out_8623400185762976873[66] = 0;
   out_8623400185762976873[67] = 0;
   out_8623400185762976873[68] = 0;
   out_8623400185762976873[69] = 0;
   out_8623400185762976873[70] = 1;
   out_8623400185762976873[71] = 0;
   out_8623400185762976873[72] = 0;
   out_8623400185762976873[73] = 0;
   out_8623400185762976873[74] = 0;
   out_8623400185762976873[75] = 0;
   out_8623400185762976873[76] = 0;
   out_8623400185762976873[77] = 0;
   out_8623400185762976873[78] = 0;
   out_8623400185762976873[79] = 0;
   out_8623400185762976873[80] = 1;
}
void h_25(double *state, double *unused, double *out_3690229574354332810) {
   out_3690229574354332810[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8370508047571984746) {
   out_8370508047571984746[0] = 0;
   out_8370508047571984746[1] = 0;
   out_8370508047571984746[2] = 0;
   out_8370508047571984746[3] = 0;
   out_8370508047571984746[4] = 0;
   out_8370508047571984746[5] = 0;
   out_8370508047571984746[6] = 1;
   out_8370508047571984746[7] = 0;
   out_8370508047571984746[8] = 0;
}
void h_24(double *state, double *unused, double *out_8276081211538433864) {
   out_8276081211538433864[0] = state[4];
   out_8276081211538433864[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3500664219546048769) {
   out_3500664219546048769[0] = 0;
   out_3500664219546048769[1] = 0;
   out_3500664219546048769[2] = 0;
   out_3500664219546048769[3] = 0;
   out_3500664219546048769[4] = 1;
   out_3500664219546048769[5] = 0;
   out_3500664219546048769[6] = 0;
   out_3500664219546048769[7] = 0;
   out_3500664219546048769[8] = 0;
   out_3500664219546048769[9] = 0;
   out_3500664219546048769[10] = 0;
   out_3500664219546048769[11] = 0;
   out_3500664219546048769[12] = 0;
   out_3500664219546048769[13] = 0;
   out_3500664219546048769[14] = 1;
   out_3500664219546048769[15] = 0;
   out_3500664219546048769[16] = 0;
   out_3500664219546048769[17] = 0;
}
void h_30(double *state, double *unused, double *out_4191043076399716375) {
   out_4191043076399716375[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3159545684645950115) {
   out_3159545684645950115[0] = 0;
   out_3159545684645950115[1] = 0;
   out_3159545684645950115[2] = 0;
   out_3159545684645950115[3] = 0;
   out_3159545684645950115[4] = 1;
   out_3159545684645950115[5] = 0;
   out_3159545684645950115[6] = 0;
   out_3159545684645950115[7] = 0;
   out_3159545684645950115[8] = 0;
}
void h_26(double *state, double *unused, double *out_8770616965969002786) {
   out_8770616965969002786[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4629004728697928522) {
   out_4629004728697928522[0] = 0;
   out_4629004728697928522[1] = 0;
   out_4629004728697928522[2] = 0;
   out_4629004728697928522[3] = 0;
   out_4629004728697928522[4] = 0;
   out_4629004728697928522[5] = 0;
   out_4629004728697928522[6] = 0;
   out_4629004728697928522[7] = 1;
   out_4629004728697928522[8] = 0;
}
void h_27(double *state, double *unused, double *out_8652609572035626042) {
   out_8652609572035626042[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5334308996446375026) {
   out_5334308996446375026[0] = 0;
   out_5334308996446375026[1] = 0;
   out_5334308996446375026[2] = 0;
   out_5334308996446375026[3] = 1;
   out_5334308996446375026[4] = 0;
   out_5334308996446375026[5] = 0;
   out_5334308996446375026[6] = 0;
   out_5334308996446375026[7] = 0;
   out_5334308996446375026[8] = 0;
}
void h_29(double *state, double *unused, double *out_1154257024955383690) {
   out_1154257024955383690[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7047671723315926059) {
   out_7047671723315926059[0] = 0;
   out_7047671723315926059[1] = 1;
   out_7047671723315926059[2] = 0;
   out_7047671723315926059[3] = 0;
   out_7047671723315926059[4] = 0;
   out_7047671723315926059[5] = 0;
   out_7047671723315926059[6] = 0;
   out_7047671723315926059[7] = 0;
   out_7047671723315926059[8] = 0;
}
void h_28(double *state, double *unused, double *out_367933840404850400) {
   out_367933840404850400[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6316673333324094983) {
   out_6316673333324094983[0] = 1;
   out_6316673333324094983[1] = 0;
   out_6316673333324094983[2] = 0;
   out_6316673333324094983[3] = 0;
   out_6316673333324094983[4] = 0;
   out_6316673333324094983[5] = 0;
   out_6316673333324094983[6] = 0;
   out_6316673333324094983[7] = 0;
   out_6316673333324094983[8] = 0;
}
void h_31(double *state, double *unused, double *out_4073035682466339631) {
   out_4073035682466339631[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8401154009448945174) {
   out_8401154009448945174[0] = 0;
   out_8401154009448945174[1] = 0;
   out_8401154009448945174[2] = 0;
   out_8401154009448945174[3] = 0;
   out_8401154009448945174[4] = 0;
   out_8401154009448945174[5] = 0;
   out_8401154009448945174[6] = 0;
   out_8401154009448945174[7] = 0;
   out_8401154009448945174[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_7307813249485127470) {
  err_fun(nom_x, delta_x, out_7307813249485127470);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3794738096767726126) {
  inv_err_fun(nom_x, true_x, out_3794738096767726126);
}
void car_H_mod_fun(double *state, double *out_5274657662750201021) {
  H_mod_fun(state, out_5274657662750201021);
}
void car_f_fun(double *state, double dt, double *out_3406949635768903926) {
  f_fun(state,  dt, out_3406949635768903926);
}
void car_F_fun(double *state, double dt, double *out_8623400185762976873) {
  F_fun(state,  dt, out_8623400185762976873);
}
void car_h_25(double *state, double *unused, double *out_3690229574354332810) {
  h_25(state, unused, out_3690229574354332810);
}
void car_H_25(double *state, double *unused, double *out_8370508047571984746) {
  H_25(state, unused, out_8370508047571984746);
}
void car_h_24(double *state, double *unused, double *out_8276081211538433864) {
  h_24(state, unused, out_8276081211538433864);
}
void car_H_24(double *state, double *unused, double *out_3500664219546048769) {
  H_24(state, unused, out_3500664219546048769);
}
void car_h_30(double *state, double *unused, double *out_4191043076399716375) {
  h_30(state, unused, out_4191043076399716375);
}
void car_H_30(double *state, double *unused, double *out_3159545684645950115) {
  H_30(state, unused, out_3159545684645950115);
}
void car_h_26(double *state, double *unused, double *out_8770616965969002786) {
  h_26(state, unused, out_8770616965969002786);
}
void car_H_26(double *state, double *unused, double *out_4629004728697928522) {
  H_26(state, unused, out_4629004728697928522);
}
void car_h_27(double *state, double *unused, double *out_8652609572035626042) {
  h_27(state, unused, out_8652609572035626042);
}
void car_H_27(double *state, double *unused, double *out_5334308996446375026) {
  H_27(state, unused, out_5334308996446375026);
}
void car_h_29(double *state, double *unused, double *out_1154257024955383690) {
  h_29(state, unused, out_1154257024955383690);
}
void car_H_29(double *state, double *unused, double *out_7047671723315926059) {
  H_29(state, unused, out_7047671723315926059);
}
void car_h_28(double *state, double *unused, double *out_367933840404850400) {
  h_28(state, unused, out_367933840404850400);
}
void car_H_28(double *state, double *unused, double *out_6316673333324094983) {
  H_28(state, unused, out_6316673333324094983);
}
void car_h_31(double *state, double *unused, double *out_4073035682466339631) {
  h_31(state, unused, out_4073035682466339631);
}
void car_H_31(double *state, double *unused, double *out_8401154009448945174) {
  H_31(state, unused, out_8401154009448945174);
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
