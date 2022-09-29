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
void err_fun(double *nom_x, double *delta_x, double *out_3243033182238193640) {
   out_3243033182238193640[0] = delta_x[0] + nom_x[0];
   out_3243033182238193640[1] = delta_x[1] + nom_x[1];
   out_3243033182238193640[2] = delta_x[2] + nom_x[2];
   out_3243033182238193640[3] = delta_x[3] + nom_x[3];
   out_3243033182238193640[4] = delta_x[4] + nom_x[4];
   out_3243033182238193640[5] = delta_x[5] + nom_x[5];
   out_3243033182238193640[6] = delta_x[6] + nom_x[6];
   out_3243033182238193640[7] = delta_x[7] + nom_x[7];
   out_3243033182238193640[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2111442011747270149) {
   out_2111442011747270149[0] = -nom_x[0] + true_x[0];
   out_2111442011747270149[1] = -nom_x[1] + true_x[1];
   out_2111442011747270149[2] = -nom_x[2] + true_x[2];
   out_2111442011747270149[3] = -nom_x[3] + true_x[3];
   out_2111442011747270149[4] = -nom_x[4] + true_x[4];
   out_2111442011747270149[5] = -nom_x[5] + true_x[5];
   out_2111442011747270149[6] = -nom_x[6] + true_x[6];
   out_2111442011747270149[7] = -nom_x[7] + true_x[7];
   out_2111442011747270149[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8408061644982863180) {
   out_8408061644982863180[0] = 1.0;
   out_8408061644982863180[1] = 0;
   out_8408061644982863180[2] = 0;
   out_8408061644982863180[3] = 0;
   out_8408061644982863180[4] = 0;
   out_8408061644982863180[5] = 0;
   out_8408061644982863180[6] = 0;
   out_8408061644982863180[7] = 0;
   out_8408061644982863180[8] = 0;
   out_8408061644982863180[9] = 0;
   out_8408061644982863180[10] = 1.0;
   out_8408061644982863180[11] = 0;
   out_8408061644982863180[12] = 0;
   out_8408061644982863180[13] = 0;
   out_8408061644982863180[14] = 0;
   out_8408061644982863180[15] = 0;
   out_8408061644982863180[16] = 0;
   out_8408061644982863180[17] = 0;
   out_8408061644982863180[18] = 0;
   out_8408061644982863180[19] = 0;
   out_8408061644982863180[20] = 1.0;
   out_8408061644982863180[21] = 0;
   out_8408061644982863180[22] = 0;
   out_8408061644982863180[23] = 0;
   out_8408061644982863180[24] = 0;
   out_8408061644982863180[25] = 0;
   out_8408061644982863180[26] = 0;
   out_8408061644982863180[27] = 0;
   out_8408061644982863180[28] = 0;
   out_8408061644982863180[29] = 0;
   out_8408061644982863180[30] = 1.0;
   out_8408061644982863180[31] = 0;
   out_8408061644982863180[32] = 0;
   out_8408061644982863180[33] = 0;
   out_8408061644982863180[34] = 0;
   out_8408061644982863180[35] = 0;
   out_8408061644982863180[36] = 0;
   out_8408061644982863180[37] = 0;
   out_8408061644982863180[38] = 0;
   out_8408061644982863180[39] = 0;
   out_8408061644982863180[40] = 1.0;
   out_8408061644982863180[41] = 0;
   out_8408061644982863180[42] = 0;
   out_8408061644982863180[43] = 0;
   out_8408061644982863180[44] = 0;
   out_8408061644982863180[45] = 0;
   out_8408061644982863180[46] = 0;
   out_8408061644982863180[47] = 0;
   out_8408061644982863180[48] = 0;
   out_8408061644982863180[49] = 0;
   out_8408061644982863180[50] = 1.0;
   out_8408061644982863180[51] = 0;
   out_8408061644982863180[52] = 0;
   out_8408061644982863180[53] = 0;
   out_8408061644982863180[54] = 0;
   out_8408061644982863180[55] = 0;
   out_8408061644982863180[56] = 0;
   out_8408061644982863180[57] = 0;
   out_8408061644982863180[58] = 0;
   out_8408061644982863180[59] = 0;
   out_8408061644982863180[60] = 1.0;
   out_8408061644982863180[61] = 0;
   out_8408061644982863180[62] = 0;
   out_8408061644982863180[63] = 0;
   out_8408061644982863180[64] = 0;
   out_8408061644982863180[65] = 0;
   out_8408061644982863180[66] = 0;
   out_8408061644982863180[67] = 0;
   out_8408061644982863180[68] = 0;
   out_8408061644982863180[69] = 0;
   out_8408061644982863180[70] = 1.0;
   out_8408061644982863180[71] = 0;
   out_8408061644982863180[72] = 0;
   out_8408061644982863180[73] = 0;
   out_8408061644982863180[74] = 0;
   out_8408061644982863180[75] = 0;
   out_8408061644982863180[76] = 0;
   out_8408061644982863180[77] = 0;
   out_8408061644982863180[78] = 0;
   out_8408061644982863180[79] = 0;
   out_8408061644982863180[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5002018357170392705) {
   out_5002018357170392705[0] = state[0];
   out_5002018357170392705[1] = state[1];
   out_5002018357170392705[2] = state[2];
   out_5002018357170392705[3] = state[3];
   out_5002018357170392705[4] = state[4];
   out_5002018357170392705[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5002018357170392705[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5002018357170392705[7] = state[7];
   out_5002018357170392705[8] = state[8];
}
void F_fun(double *state, double dt, double *out_123727185940828648) {
   out_123727185940828648[0] = 1;
   out_123727185940828648[1] = 0;
   out_123727185940828648[2] = 0;
   out_123727185940828648[3] = 0;
   out_123727185940828648[4] = 0;
   out_123727185940828648[5] = 0;
   out_123727185940828648[6] = 0;
   out_123727185940828648[7] = 0;
   out_123727185940828648[8] = 0;
   out_123727185940828648[9] = 0;
   out_123727185940828648[10] = 1;
   out_123727185940828648[11] = 0;
   out_123727185940828648[12] = 0;
   out_123727185940828648[13] = 0;
   out_123727185940828648[14] = 0;
   out_123727185940828648[15] = 0;
   out_123727185940828648[16] = 0;
   out_123727185940828648[17] = 0;
   out_123727185940828648[18] = 0;
   out_123727185940828648[19] = 0;
   out_123727185940828648[20] = 1;
   out_123727185940828648[21] = 0;
   out_123727185940828648[22] = 0;
   out_123727185940828648[23] = 0;
   out_123727185940828648[24] = 0;
   out_123727185940828648[25] = 0;
   out_123727185940828648[26] = 0;
   out_123727185940828648[27] = 0;
   out_123727185940828648[28] = 0;
   out_123727185940828648[29] = 0;
   out_123727185940828648[30] = 1;
   out_123727185940828648[31] = 0;
   out_123727185940828648[32] = 0;
   out_123727185940828648[33] = 0;
   out_123727185940828648[34] = 0;
   out_123727185940828648[35] = 0;
   out_123727185940828648[36] = 0;
   out_123727185940828648[37] = 0;
   out_123727185940828648[38] = 0;
   out_123727185940828648[39] = 0;
   out_123727185940828648[40] = 1;
   out_123727185940828648[41] = 0;
   out_123727185940828648[42] = 0;
   out_123727185940828648[43] = 0;
   out_123727185940828648[44] = 0;
   out_123727185940828648[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_123727185940828648[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_123727185940828648[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_123727185940828648[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_123727185940828648[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_123727185940828648[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_123727185940828648[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_123727185940828648[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_123727185940828648[53] = -9.8000000000000007*dt;
   out_123727185940828648[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_123727185940828648[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_123727185940828648[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_123727185940828648[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_123727185940828648[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_123727185940828648[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_123727185940828648[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_123727185940828648[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_123727185940828648[62] = 0;
   out_123727185940828648[63] = 0;
   out_123727185940828648[64] = 0;
   out_123727185940828648[65] = 0;
   out_123727185940828648[66] = 0;
   out_123727185940828648[67] = 0;
   out_123727185940828648[68] = 0;
   out_123727185940828648[69] = 0;
   out_123727185940828648[70] = 1;
   out_123727185940828648[71] = 0;
   out_123727185940828648[72] = 0;
   out_123727185940828648[73] = 0;
   out_123727185940828648[74] = 0;
   out_123727185940828648[75] = 0;
   out_123727185940828648[76] = 0;
   out_123727185940828648[77] = 0;
   out_123727185940828648[78] = 0;
   out_123727185940828648[79] = 0;
   out_123727185940828648[80] = 1;
}
void h_25(double *state, double *unused, double *out_4633127428766816095) {
   out_4633127428766816095[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5849167184823695555) {
   out_5849167184823695555[0] = 0;
   out_5849167184823695555[1] = 0;
   out_5849167184823695555[2] = 0;
   out_5849167184823695555[3] = 0;
   out_5849167184823695555[4] = 0;
   out_5849167184823695555[5] = 0;
   out_5849167184823695555[6] = 1;
   out_5849167184823695555[7] = 0;
   out_5849167184823695555[8] = 0;
}
void h_24(double *state, double *unused, double *out_4389809127990204203) {
   out_4389809127990204203[0] = state[4];
   out_4389809127990204203[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3676517585818195989) {
   out_3676517585818195989[0] = 0;
   out_3676517585818195989[1] = 0;
   out_3676517585818195989[2] = 0;
   out_3676517585818195989[3] = 0;
   out_3676517585818195989[4] = 1;
   out_3676517585818195989[5] = 0;
   out_3676517585818195989[6] = 0;
   out_3676517585818195989[7] = 0;
   out_3676517585818195989[8] = 0;
   out_3676517585818195989[9] = 0;
   out_3676517585818195989[10] = 0;
   out_3676517585818195989[11] = 0;
   out_3676517585818195989[12] = 0;
   out_3676517585818195989[13] = 0;
   out_3676517585818195989[14] = 1;
   out_3676517585818195989[15] = 0;
   out_3676517585818195989[16] = 0;
   out_3676517585818195989[17] = 0;
}
void h_30(double *state, double *unused, double *out_2827431335482072230) {
   out_2827431335482072230[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8367500143330944182) {
   out_8367500143330944182[0] = 0;
   out_8367500143330944182[1] = 0;
   out_8367500143330944182[2] = 0;
   out_8367500143330944182[3] = 0;
   out_8367500143330944182[4] = 1;
   out_8367500143330944182[5] = 0;
   out_8367500143330944182[6] = 0;
   out_8367500143330944182[7] = 0;
   out_8367500143330944182[8] = 0;
}
void h_26(double *state, double *unused, double *out_6598769325787002944) {
   out_6598769325787002944[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2107663865949639331) {
   out_2107663865949639331[0] = 0;
   out_2107663865949639331[1] = 0;
   out_2107663865949639331[2] = 0;
   out_2107663865949639331[3] = 0;
   out_2107663865949639331[4] = 0;
   out_2107663865949639331[5] = 0;
   out_2107663865949639331[6] = 0;
   out_2107663865949639331[7] = 1;
   out_2107663865949639331[8] = 0;
}
void h_27(double *state, double *unused, double *out_1622697293192231159) {
   out_1622697293192231159[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6192736831530519271) {
   out_6192736831530519271[0] = 0;
   out_6192736831530519271[1] = 0;
   out_6192736831530519271[2] = 0;
   out_6192736831530519271[3] = 1;
   out_6192736831530519271[4] = 0;
   out_6192736831530519271[5] = 0;
   out_6192736831530519271[6] = 0;
   out_6192736831530519271[7] = 0;
   out_6192736831530519271[8] = 0;
}
void h_29(double *state, double *unused, double *out_2238986183236270493) {
   out_2238986183236270493[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8877731487645336366) {
   out_8877731487645336366[0] = 0;
   out_8877731487645336366[1] = 1;
   out_8877731487645336366[2] = 0;
   out_8877731487645336366[3] = 0;
   out_8877731487645336366[4] = 0;
   out_8877731487645336366[5] = 0;
   out_8877731487645336366[6] = 0;
   out_8877731487645336366[7] = 0;
   out_8877731487645336366[8] = 0;
}
void h_28(double *state, double *unused, double *out_2709423941548695486) {
   out_2709423941548695486[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3795332470575805792) {
   out_3795332470575805792[0] = 1;
   out_3795332470575805792[1] = 0;
   out_3795332470575805792[2] = 0;
   out_3795332470575805792[3] = 0;
   out_3795332470575805792[4] = 0;
   out_3795332470575805792[5] = 0;
   out_3795332470575805792[6] = 0;
   out_3795332470575805792[7] = 0;
   out_3795332470575805792[8] = 0;
}
void h_31(double *state, double *unused, double *out_5173739168224193637) {
   out_5173739168224193637[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1481455763716287855) {
   out_1481455763716287855[0] = 0;
   out_1481455763716287855[1] = 0;
   out_1481455763716287855[2] = 0;
   out_1481455763716287855[3] = 0;
   out_1481455763716287855[4] = 0;
   out_1481455763716287855[5] = 0;
   out_1481455763716287855[6] = 0;
   out_1481455763716287855[7] = 0;
   out_1481455763716287855[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3243033182238193640) {
  err_fun(nom_x, delta_x, out_3243033182238193640);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2111442011747270149) {
  inv_err_fun(nom_x, true_x, out_2111442011747270149);
}
void car_H_mod_fun(double *state, double *out_8408061644982863180) {
  H_mod_fun(state, out_8408061644982863180);
}
void car_f_fun(double *state, double dt, double *out_5002018357170392705) {
  f_fun(state,  dt, out_5002018357170392705);
}
void car_F_fun(double *state, double dt, double *out_123727185940828648) {
  F_fun(state,  dt, out_123727185940828648);
}
void car_h_25(double *state, double *unused, double *out_4633127428766816095) {
  h_25(state, unused, out_4633127428766816095);
}
void car_H_25(double *state, double *unused, double *out_5849167184823695555) {
  H_25(state, unused, out_5849167184823695555);
}
void car_h_24(double *state, double *unused, double *out_4389809127990204203) {
  h_24(state, unused, out_4389809127990204203);
}
void car_H_24(double *state, double *unused, double *out_3676517585818195989) {
  H_24(state, unused, out_3676517585818195989);
}
void car_h_30(double *state, double *unused, double *out_2827431335482072230) {
  h_30(state, unused, out_2827431335482072230);
}
void car_H_30(double *state, double *unused, double *out_8367500143330944182) {
  H_30(state, unused, out_8367500143330944182);
}
void car_h_26(double *state, double *unused, double *out_6598769325787002944) {
  h_26(state, unused, out_6598769325787002944);
}
void car_H_26(double *state, double *unused, double *out_2107663865949639331) {
  H_26(state, unused, out_2107663865949639331);
}
void car_h_27(double *state, double *unused, double *out_1622697293192231159) {
  h_27(state, unused, out_1622697293192231159);
}
void car_H_27(double *state, double *unused, double *out_6192736831530519271) {
  H_27(state, unused, out_6192736831530519271);
}
void car_h_29(double *state, double *unused, double *out_2238986183236270493) {
  h_29(state, unused, out_2238986183236270493);
}
void car_H_29(double *state, double *unused, double *out_8877731487645336366) {
  H_29(state, unused, out_8877731487645336366);
}
void car_h_28(double *state, double *unused, double *out_2709423941548695486) {
  h_28(state, unused, out_2709423941548695486);
}
void car_H_28(double *state, double *unused, double *out_3795332470575805792) {
  H_28(state, unused, out_3795332470575805792);
}
void car_h_31(double *state, double *unused, double *out_5173739168224193637) {
  h_31(state, unused, out_5173739168224193637);
}
void car_H_31(double *state, double *unused, double *out_1481455763716287855) {
  H_31(state, unused, out_1481455763716287855);
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
