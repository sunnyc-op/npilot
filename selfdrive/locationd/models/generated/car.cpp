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
void err_fun(double *nom_x, double *delta_x, double *out_1780734620249590456) {
   out_1780734620249590456[0] = delta_x[0] + nom_x[0];
   out_1780734620249590456[1] = delta_x[1] + nom_x[1];
   out_1780734620249590456[2] = delta_x[2] + nom_x[2];
   out_1780734620249590456[3] = delta_x[3] + nom_x[3];
   out_1780734620249590456[4] = delta_x[4] + nom_x[4];
   out_1780734620249590456[5] = delta_x[5] + nom_x[5];
   out_1780734620249590456[6] = delta_x[6] + nom_x[6];
   out_1780734620249590456[7] = delta_x[7] + nom_x[7];
   out_1780734620249590456[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4094607078236144920) {
   out_4094607078236144920[0] = -nom_x[0] + true_x[0];
   out_4094607078236144920[1] = -nom_x[1] + true_x[1];
   out_4094607078236144920[2] = -nom_x[2] + true_x[2];
   out_4094607078236144920[3] = -nom_x[3] + true_x[3];
   out_4094607078236144920[4] = -nom_x[4] + true_x[4];
   out_4094607078236144920[5] = -nom_x[5] + true_x[5];
   out_4094607078236144920[6] = -nom_x[6] + true_x[6];
   out_4094607078236144920[7] = -nom_x[7] + true_x[7];
   out_4094607078236144920[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3009298313997658244) {
   out_3009298313997658244[0] = 1.0;
   out_3009298313997658244[1] = 0;
   out_3009298313997658244[2] = 0;
   out_3009298313997658244[3] = 0;
   out_3009298313997658244[4] = 0;
   out_3009298313997658244[5] = 0;
   out_3009298313997658244[6] = 0;
   out_3009298313997658244[7] = 0;
   out_3009298313997658244[8] = 0;
   out_3009298313997658244[9] = 0;
   out_3009298313997658244[10] = 1.0;
   out_3009298313997658244[11] = 0;
   out_3009298313997658244[12] = 0;
   out_3009298313997658244[13] = 0;
   out_3009298313997658244[14] = 0;
   out_3009298313997658244[15] = 0;
   out_3009298313997658244[16] = 0;
   out_3009298313997658244[17] = 0;
   out_3009298313997658244[18] = 0;
   out_3009298313997658244[19] = 0;
   out_3009298313997658244[20] = 1.0;
   out_3009298313997658244[21] = 0;
   out_3009298313997658244[22] = 0;
   out_3009298313997658244[23] = 0;
   out_3009298313997658244[24] = 0;
   out_3009298313997658244[25] = 0;
   out_3009298313997658244[26] = 0;
   out_3009298313997658244[27] = 0;
   out_3009298313997658244[28] = 0;
   out_3009298313997658244[29] = 0;
   out_3009298313997658244[30] = 1.0;
   out_3009298313997658244[31] = 0;
   out_3009298313997658244[32] = 0;
   out_3009298313997658244[33] = 0;
   out_3009298313997658244[34] = 0;
   out_3009298313997658244[35] = 0;
   out_3009298313997658244[36] = 0;
   out_3009298313997658244[37] = 0;
   out_3009298313997658244[38] = 0;
   out_3009298313997658244[39] = 0;
   out_3009298313997658244[40] = 1.0;
   out_3009298313997658244[41] = 0;
   out_3009298313997658244[42] = 0;
   out_3009298313997658244[43] = 0;
   out_3009298313997658244[44] = 0;
   out_3009298313997658244[45] = 0;
   out_3009298313997658244[46] = 0;
   out_3009298313997658244[47] = 0;
   out_3009298313997658244[48] = 0;
   out_3009298313997658244[49] = 0;
   out_3009298313997658244[50] = 1.0;
   out_3009298313997658244[51] = 0;
   out_3009298313997658244[52] = 0;
   out_3009298313997658244[53] = 0;
   out_3009298313997658244[54] = 0;
   out_3009298313997658244[55] = 0;
   out_3009298313997658244[56] = 0;
   out_3009298313997658244[57] = 0;
   out_3009298313997658244[58] = 0;
   out_3009298313997658244[59] = 0;
   out_3009298313997658244[60] = 1.0;
   out_3009298313997658244[61] = 0;
   out_3009298313997658244[62] = 0;
   out_3009298313997658244[63] = 0;
   out_3009298313997658244[64] = 0;
   out_3009298313997658244[65] = 0;
   out_3009298313997658244[66] = 0;
   out_3009298313997658244[67] = 0;
   out_3009298313997658244[68] = 0;
   out_3009298313997658244[69] = 0;
   out_3009298313997658244[70] = 1.0;
   out_3009298313997658244[71] = 0;
   out_3009298313997658244[72] = 0;
   out_3009298313997658244[73] = 0;
   out_3009298313997658244[74] = 0;
   out_3009298313997658244[75] = 0;
   out_3009298313997658244[76] = 0;
   out_3009298313997658244[77] = 0;
   out_3009298313997658244[78] = 0;
   out_3009298313997658244[79] = 0;
   out_3009298313997658244[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7642406061189318393) {
   out_7642406061189318393[0] = state[0];
   out_7642406061189318393[1] = state[1];
   out_7642406061189318393[2] = state[2];
   out_7642406061189318393[3] = state[3];
   out_7642406061189318393[4] = state[4];
   out_7642406061189318393[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7642406061189318393[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7642406061189318393[7] = state[7];
   out_7642406061189318393[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8250113623205205041) {
   out_8250113623205205041[0] = 1;
   out_8250113623205205041[1] = 0;
   out_8250113623205205041[2] = 0;
   out_8250113623205205041[3] = 0;
   out_8250113623205205041[4] = 0;
   out_8250113623205205041[5] = 0;
   out_8250113623205205041[6] = 0;
   out_8250113623205205041[7] = 0;
   out_8250113623205205041[8] = 0;
   out_8250113623205205041[9] = 0;
   out_8250113623205205041[10] = 1;
   out_8250113623205205041[11] = 0;
   out_8250113623205205041[12] = 0;
   out_8250113623205205041[13] = 0;
   out_8250113623205205041[14] = 0;
   out_8250113623205205041[15] = 0;
   out_8250113623205205041[16] = 0;
   out_8250113623205205041[17] = 0;
   out_8250113623205205041[18] = 0;
   out_8250113623205205041[19] = 0;
   out_8250113623205205041[20] = 1;
   out_8250113623205205041[21] = 0;
   out_8250113623205205041[22] = 0;
   out_8250113623205205041[23] = 0;
   out_8250113623205205041[24] = 0;
   out_8250113623205205041[25] = 0;
   out_8250113623205205041[26] = 0;
   out_8250113623205205041[27] = 0;
   out_8250113623205205041[28] = 0;
   out_8250113623205205041[29] = 0;
   out_8250113623205205041[30] = 1;
   out_8250113623205205041[31] = 0;
   out_8250113623205205041[32] = 0;
   out_8250113623205205041[33] = 0;
   out_8250113623205205041[34] = 0;
   out_8250113623205205041[35] = 0;
   out_8250113623205205041[36] = 0;
   out_8250113623205205041[37] = 0;
   out_8250113623205205041[38] = 0;
   out_8250113623205205041[39] = 0;
   out_8250113623205205041[40] = 1;
   out_8250113623205205041[41] = 0;
   out_8250113623205205041[42] = 0;
   out_8250113623205205041[43] = 0;
   out_8250113623205205041[44] = 0;
   out_8250113623205205041[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8250113623205205041[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8250113623205205041[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8250113623205205041[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8250113623205205041[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8250113623205205041[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8250113623205205041[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8250113623205205041[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8250113623205205041[53] = -9.8000000000000007*dt;
   out_8250113623205205041[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8250113623205205041[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8250113623205205041[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8250113623205205041[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8250113623205205041[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8250113623205205041[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8250113623205205041[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8250113623205205041[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8250113623205205041[62] = 0;
   out_8250113623205205041[63] = 0;
   out_8250113623205205041[64] = 0;
   out_8250113623205205041[65] = 0;
   out_8250113623205205041[66] = 0;
   out_8250113623205205041[67] = 0;
   out_8250113623205205041[68] = 0;
   out_8250113623205205041[69] = 0;
   out_8250113623205205041[70] = 1;
   out_8250113623205205041[71] = 0;
   out_8250113623205205041[72] = 0;
   out_8250113623205205041[73] = 0;
   out_8250113623205205041[74] = 0;
   out_8250113623205205041[75] = 0;
   out_8250113623205205041[76] = 0;
   out_8250113623205205041[77] = 0;
   out_8250113623205205041[78] = 0;
   out_8250113623205205041[79] = 0;
   out_8250113623205205041[80] = 1;
}
void h_25(double *state, double *unused, double *out_5183177480751370237) {
   out_5183177480751370237[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4451369567180334730) {
   out_4451369567180334730[0] = 0;
   out_4451369567180334730[1] = 0;
   out_4451369567180334730[2] = 0;
   out_4451369567180334730[3] = 0;
   out_4451369567180334730[4] = 0;
   out_4451369567180334730[5] = 0;
   out_4451369567180334730[6] = 1;
   out_4451369567180334730[7] = 0;
   out_4451369567180334730[8] = 0;
}
void h_24(double *state, double *unused, double *out_5010304262199017215) {
   out_5010304262199017215[0] = state[4];
   out_5010304262199017215[1] = state[5];
}
void H_24(double *state, double *unused, double *out_469046415782566832) {
   out_469046415782566832[0] = 0;
   out_469046415782566832[1] = 0;
   out_469046415782566832[2] = 0;
   out_469046415782566832[3] = 0;
   out_469046415782566832[4] = 1;
   out_469046415782566832[5] = 0;
   out_469046415782566832[6] = 0;
   out_469046415782566832[7] = 0;
   out_469046415782566832[8] = 0;
   out_469046415782566832[9] = 0;
   out_469046415782566832[10] = 0;
   out_469046415782566832[11] = 0;
   out_469046415782566832[12] = 0;
   out_469046415782566832[13] = 0;
   out_469046415782566832[14] = 1;
   out_469046415782566832[15] = 0;
   out_469046415782566832[16] = 0;
   out_469046415782566832[17] = 0;
}
void h_30(double *state, double *unused, double *out_4766905911694441387) {
   out_4766905911694441387[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6969702525687583357) {
   out_6969702525687583357[0] = 0;
   out_6969702525687583357[1] = 0;
   out_6969702525687583357[2] = 0;
   out_6969702525687583357[3] = 0;
   out_6969702525687583357[4] = 1;
   out_6969702525687583357[5] = 0;
   out_6969702525687583357[6] = 0;
   out_6969702525687583357[7] = 0;
   out_6969702525687583357[8] = 0;
}
void h_26(double *state, double *unused, double *out_875823959386515448) {
   out_875823959386515448[0] = state[7];
}
void H_26(double *state, double *unused, double *out_709866248306278506) {
   out_709866248306278506[0] = 0;
   out_709866248306278506[1] = 0;
   out_709866248306278506[2] = 0;
   out_709866248306278506[3] = 0;
   out_709866248306278506[4] = 0;
   out_709866248306278506[5] = 0;
   out_709866248306278506[6] = 0;
   out_709866248306278506[7] = 1;
   out_709866248306278506[8] = 0;
}
void h_27(double *state, double *unused, double *out_4425327335590551427) {
   out_4425327335590551427[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4794939213887158446) {
   out_4794939213887158446[0] = 0;
   out_4794939213887158446[1] = 0;
   out_4794939213887158446[2] = 0;
   out_4794939213887158446[3] = 1;
   out_4794939213887158446[4] = 0;
   out_4794939213887158446[5] = 0;
   out_4794939213887158446[6] = 0;
   out_4794939213887158446[7] = 0;
   out_4794939213887158446[8] = 0;
}
void h_29(double *state, double *unused, double *out_8678164894354264584) {
   out_8678164894354264584[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7479933870001975541) {
   out_7479933870001975541[0] = 0;
   out_7479933870001975541[1] = 1;
   out_7479933870001975541[2] = 0;
   out_7479933870001975541[3] = 0;
   out_7479933870001975541[4] = 0;
   out_7479933870001975541[5] = 0;
   out_7479933870001975541[6] = 0;
   out_7479933870001975541[7] = 0;
   out_7479933870001975541[8] = 0;
}
void h_28(double *state, double *unused, double *out_8679549502232969626) {
   out_8679549502232969626[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2397534852932444967) {
   out_2397534852932444967[0] = 1;
   out_2397534852932444967[1] = 0;
   out_2397534852932444967[2] = 0;
   out_2397534852932444967[3] = 0;
   out_2397534852932444967[4] = 0;
   out_2397534852932444967[5] = 0;
   out_2397534852932444967[6] = 0;
   out_2397534852932444967[7] = 0;
   out_2397534852932444967[8] = 0;
}
void h_31(double *state, double *unused, double *out_5382293942204132194) {
   out_5382293942204132194[0] = state[8];
}
void H_31(double *state, double *unused, double *out_83658146072927030) {
   out_83658146072927030[0] = 0;
   out_83658146072927030[1] = 0;
   out_83658146072927030[2] = 0;
   out_83658146072927030[3] = 0;
   out_83658146072927030[4] = 0;
   out_83658146072927030[5] = 0;
   out_83658146072927030[6] = 0;
   out_83658146072927030[7] = 0;
   out_83658146072927030[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_1780734620249590456) {
  err_fun(nom_x, delta_x, out_1780734620249590456);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4094607078236144920) {
  inv_err_fun(nom_x, true_x, out_4094607078236144920);
}
void car_H_mod_fun(double *state, double *out_3009298313997658244) {
  H_mod_fun(state, out_3009298313997658244);
}
void car_f_fun(double *state, double dt, double *out_7642406061189318393) {
  f_fun(state,  dt, out_7642406061189318393);
}
void car_F_fun(double *state, double dt, double *out_8250113623205205041) {
  F_fun(state,  dt, out_8250113623205205041);
}
void car_h_25(double *state, double *unused, double *out_5183177480751370237) {
  h_25(state, unused, out_5183177480751370237);
}
void car_H_25(double *state, double *unused, double *out_4451369567180334730) {
  H_25(state, unused, out_4451369567180334730);
}
void car_h_24(double *state, double *unused, double *out_5010304262199017215) {
  h_24(state, unused, out_5010304262199017215);
}
void car_H_24(double *state, double *unused, double *out_469046415782566832) {
  H_24(state, unused, out_469046415782566832);
}
void car_h_30(double *state, double *unused, double *out_4766905911694441387) {
  h_30(state, unused, out_4766905911694441387);
}
void car_H_30(double *state, double *unused, double *out_6969702525687583357) {
  H_30(state, unused, out_6969702525687583357);
}
void car_h_26(double *state, double *unused, double *out_875823959386515448) {
  h_26(state, unused, out_875823959386515448);
}
void car_H_26(double *state, double *unused, double *out_709866248306278506) {
  H_26(state, unused, out_709866248306278506);
}
void car_h_27(double *state, double *unused, double *out_4425327335590551427) {
  h_27(state, unused, out_4425327335590551427);
}
void car_H_27(double *state, double *unused, double *out_4794939213887158446) {
  H_27(state, unused, out_4794939213887158446);
}
void car_h_29(double *state, double *unused, double *out_8678164894354264584) {
  h_29(state, unused, out_8678164894354264584);
}
void car_H_29(double *state, double *unused, double *out_7479933870001975541) {
  H_29(state, unused, out_7479933870001975541);
}
void car_h_28(double *state, double *unused, double *out_8679549502232969626) {
  h_28(state, unused, out_8679549502232969626);
}
void car_H_28(double *state, double *unused, double *out_2397534852932444967) {
  H_28(state, unused, out_2397534852932444967);
}
void car_h_31(double *state, double *unused, double *out_5382293942204132194) {
  h_31(state, unused, out_5382293942204132194);
}
void car_H_31(double *state, double *unused, double *out_83658146072927030) {
  H_31(state, unused, out_83658146072927030);
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
