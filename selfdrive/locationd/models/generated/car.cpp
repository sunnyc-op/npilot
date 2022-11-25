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
void err_fun(double *nom_x, double *delta_x, double *out_6118062787728240548) {
   out_6118062787728240548[0] = delta_x[0] + nom_x[0];
   out_6118062787728240548[1] = delta_x[1] + nom_x[1];
   out_6118062787728240548[2] = delta_x[2] + nom_x[2];
   out_6118062787728240548[3] = delta_x[3] + nom_x[3];
   out_6118062787728240548[4] = delta_x[4] + nom_x[4];
   out_6118062787728240548[5] = delta_x[5] + nom_x[5];
   out_6118062787728240548[6] = delta_x[6] + nom_x[6];
   out_6118062787728240548[7] = delta_x[7] + nom_x[7];
   out_6118062787728240548[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9211263313597478507) {
   out_9211263313597478507[0] = -nom_x[0] + true_x[0];
   out_9211263313597478507[1] = -nom_x[1] + true_x[1];
   out_9211263313597478507[2] = -nom_x[2] + true_x[2];
   out_9211263313597478507[3] = -nom_x[3] + true_x[3];
   out_9211263313597478507[4] = -nom_x[4] + true_x[4];
   out_9211263313597478507[5] = -nom_x[5] + true_x[5];
   out_9211263313597478507[6] = -nom_x[6] + true_x[6];
   out_9211263313597478507[7] = -nom_x[7] + true_x[7];
   out_9211263313597478507[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4561024202480585870) {
   out_4561024202480585870[0] = 1.0;
   out_4561024202480585870[1] = 0;
   out_4561024202480585870[2] = 0;
   out_4561024202480585870[3] = 0;
   out_4561024202480585870[4] = 0;
   out_4561024202480585870[5] = 0;
   out_4561024202480585870[6] = 0;
   out_4561024202480585870[7] = 0;
   out_4561024202480585870[8] = 0;
   out_4561024202480585870[9] = 0;
   out_4561024202480585870[10] = 1.0;
   out_4561024202480585870[11] = 0;
   out_4561024202480585870[12] = 0;
   out_4561024202480585870[13] = 0;
   out_4561024202480585870[14] = 0;
   out_4561024202480585870[15] = 0;
   out_4561024202480585870[16] = 0;
   out_4561024202480585870[17] = 0;
   out_4561024202480585870[18] = 0;
   out_4561024202480585870[19] = 0;
   out_4561024202480585870[20] = 1.0;
   out_4561024202480585870[21] = 0;
   out_4561024202480585870[22] = 0;
   out_4561024202480585870[23] = 0;
   out_4561024202480585870[24] = 0;
   out_4561024202480585870[25] = 0;
   out_4561024202480585870[26] = 0;
   out_4561024202480585870[27] = 0;
   out_4561024202480585870[28] = 0;
   out_4561024202480585870[29] = 0;
   out_4561024202480585870[30] = 1.0;
   out_4561024202480585870[31] = 0;
   out_4561024202480585870[32] = 0;
   out_4561024202480585870[33] = 0;
   out_4561024202480585870[34] = 0;
   out_4561024202480585870[35] = 0;
   out_4561024202480585870[36] = 0;
   out_4561024202480585870[37] = 0;
   out_4561024202480585870[38] = 0;
   out_4561024202480585870[39] = 0;
   out_4561024202480585870[40] = 1.0;
   out_4561024202480585870[41] = 0;
   out_4561024202480585870[42] = 0;
   out_4561024202480585870[43] = 0;
   out_4561024202480585870[44] = 0;
   out_4561024202480585870[45] = 0;
   out_4561024202480585870[46] = 0;
   out_4561024202480585870[47] = 0;
   out_4561024202480585870[48] = 0;
   out_4561024202480585870[49] = 0;
   out_4561024202480585870[50] = 1.0;
   out_4561024202480585870[51] = 0;
   out_4561024202480585870[52] = 0;
   out_4561024202480585870[53] = 0;
   out_4561024202480585870[54] = 0;
   out_4561024202480585870[55] = 0;
   out_4561024202480585870[56] = 0;
   out_4561024202480585870[57] = 0;
   out_4561024202480585870[58] = 0;
   out_4561024202480585870[59] = 0;
   out_4561024202480585870[60] = 1.0;
   out_4561024202480585870[61] = 0;
   out_4561024202480585870[62] = 0;
   out_4561024202480585870[63] = 0;
   out_4561024202480585870[64] = 0;
   out_4561024202480585870[65] = 0;
   out_4561024202480585870[66] = 0;
   out_4561024202480585870[67] = 0;
   out_4561024202480585870[68] = 0;
   out_4561024202480585870[69] = 0;
   out_4561024202480585870[70] = 1.0;
   out_4561024202480585870[71] = 0;
   out_4561024202480585870[72] = 0;
   out_4561024202480585870[73] = 0;
   out_4561024202480585870[74] = 0;
   out_4561024202480585870[75] = 0;
   out_4561024202480585870[76] = 0;
   out_4561024202480585870[77] = 0;
   out_4561024202480585870[78] = 0;
   out_4561024202480585870[79] = 0;
   out_4561024202480585870[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6415819914950947790) {
   out_6415819914950947790[0] = state[0];
   out_6415819914950947790[1] = state[1];
   out_6415819914950947790[2] = state[2];
   out_6415819914950947790[3] = state[3];
   out_6415819914950947790[4] = state[4];
   out_6415819914950947790[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6415819914950947790[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6415819914950947790[7] = state[7];
   out_6415819914950947790[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5710333805771084184) {
   out_5710333805771084184[0] = 1;
   out_5710333805771084184[1] = 0;
   out_5710333805771084184[2] = 0;
   out_5710333805771084184[3] = 0;
   out_5710333805771084184[4] = 0;
   out_5710333805771084184[5] = 0;
   out_5710333805771084184[6] = 0;
   out_5710333805771084184[7] = 0;
   out_5710333805771084184[8] = 0;
   out_5710333805771084184[9] = 0;
   out_5710333805771084184[10] = 1;
   out_5710333805771084184[11] = 0;
   out_5710333805771084184[12] = 0;
   out_5710333805771084184[13] = 0;
   out_5710333805771084184[14] = 0;
   out_5710333805771084184[15] = 0;
   out_5710333805771084184[16] = 0;
   out_5710333805771084184[17] = 0;
   out_5710333805771084184[18] = 0;
   out_5710333805771084184[19] = 0;
   out_5710333805771084184[20] = 1;
   out_5710333805771084184[21] = 0;
   out_5710333805771084184[22] = 0;
   out_5710333805771084184[23] = 0;
   out_5710333805771084184[24] = 0;
   out_5710333805771084184[25] = 0;
   out_5710333805771084184[26] = 0;
   out_5710333805771084184[27] = 0;
   out_5710333805771084184[28] = 0;
   out_5710333805771084184[29] = 0;
   out_5710333805771084184[30] = 1;
   out_5710333805771084184[31] = 0;
   out_5710333805771084184[32] = 0;
   out_5710333805771084184[33] = 0;
   out_5710333805771084184[34] = 0;
   out_5710333805771084184[35] = 0;
   out_5710333805771084184[36] = 0;
   out_5710333805771084184[37] = 0;
   out_5710333805771084184[38] = 0;
   out_5710333805771084184[39] = 0;
   out_5710333805771084184[40] = 1;
   out_5710333805771084184[41] = 0;
   out_5710333805771084184[42] = 0;
   out_5710333805771084184[43] = 0;
   out_5710333805771084184[44] = 0;
   out_5710333805771084184[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5710333805771084184[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5710333805771084184[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5710333805771084184[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5710333805771084184[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5710333805771084184[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5710333805771084184[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5710333805771084184[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5710333805771084184[53] = -9.8000000000000007*dt;
   out_5710333805771084184[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5710333805771084184[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5710333805771084184[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5710333805771084184[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5710333805771084184[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5710333805771084184[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5710333805771084184[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5710333805771084184[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5710333805771084184[62] = 0;
   out_5710333805771084184[63] = 0;
   out_5710333805771084184[64] = 0;
   out_5710333805771084184[65] = 0;
   out_5710333805771084184[66] = 0;
   out_5710333805771084184[67] = 0;
   out_5710333805771084184[68] = 0;
   out_5710333805771084184[69] = 0;
   out_5710333805771084184[70] = 1;
   out_5710333805771084184[71] = 0;
   out_5710333805771084184[72] = 0;
   out_5710333805771084184[73] = 0;
   out_5710333805771084184[74] = 0;
   out_5710333805771084184[75] = 0;
   out_5710333805771084184[76] = 0;
   out_5710333805771084184[77] = 0;
   out_5710333805771084184[78] = 0;
   out_5710333805771084184[79] = 0;
   out_5710333805771084184[80] = 1;
}
void h_25(double *state, double *unused, double *out_5580444758777246286) {
   out_5580444758777246286[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5747614728045202509) {
   out_5747614728045202509[0] = 0;
   out_5747614728045202509[1] = 0;
   out_5747614728045202509[2] = 0;
   out_5747614728045202509[3] = 0;
   out_5747614728045202509[4] = 0;
   out_5747614728045202509[5] = 0;
   out_5747614728045202509[6] = 1;
   out_5747614728045202509[7] = 0;
   out_5747614728045202509[8] = 0;
}
void h_24(double *state, double *unused, double *out_5802005886931275559) {
   out_5802005886931275559[0] = state[4];
   out_5802005886931275559[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2189689129242879373) {
   out_2189689129242879373[0] = 0;
   out_2189689129242879373[1] = 0;
   out_2189689129242879373[2] = 0;
   out_2189689129242879373[3] = 0;
   out_2189689129242879373[4] = 1;
   out_2189689129242879373[5] = 0;
   out_2189689129242879373[6] = 0;
   out_2189689129242879373[7] = 0;
   out_2189689129242879373[8] = 0;
   out_2189689129242879373[9] = 0;
   out_2189689129242879373[10] = 0;
   out_2189689129242879373[11] = 0;
   out_2189689129242879373[12] = 0;
   out_2189689129242879373[13] = 0;
   out_2189689129242879373[14] = 1;
   out_2189689129242879373[15] = 0;
   out_2189689129242879373[16] = 0;
   out_2189689129242879373[17] = 0;
}
void h_30(double *state, double *unused, double *out_4642539516321470839) {
   out_4642539516321470839[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3229281769537953882) {
   out_3229281769537953882[0] = 0;
   out_3229281769537953882[1] = 0;
   out_3229281769537953882[2] = 0;
   out_3229281769537953882[3] = 0;
   out_3229281769537953882[4] = 1;
   out_3229281769537953882[5] = 0;
   out_3229281769537953882[6] = 0;
   out_3229281769537953882[7] = 0;
   out_3229281769537953882[8] = 0;
}
void h_26(double *state, double *unused, double *out_8699869409390652045) {
   out_8699869409390652045[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2443088758284401908) {
   out_2443088758284401908[0] = 0;
   out_2443088758284401908[1] = 0;
   out_2443088758284401908[2] = 0;
   out_2443088758284401908[3] = 0;
   out_2443088758284401908[4] = 0;
   out_2443088758284401908[5] = 0;
   out_2443088758284401908[6] = 0;
   out_2443088758284401908[7] = 1;
   out_2443088758284401908[8] = 0;
}
void h_27(double *state, double *unused, double *out_2031696063376884196) {
   out_2031696063376884196[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5404045081338378793) {
   out_5404045081338378793[0] = 0;
   out_5404045081338378793[1] = 0;
   out_5404045081338378793[2] = 0;
   out_5404045081338378793[3] = 1;
   out_5404045081338378793[4] = 0;
   out_5404045081338378793[5] = 0;
   out_5404045081338378793[6] = 0;
   out_5404045081338378793[7] = 0;
   out_5404045081338378793[8] = 0;
}
void h_29(double *state, double *unused, double *out_1115445397672267117) {
   out_1115445397672267117[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2719050425223561698) {
   out_2719050425223561698[0] = 0;
   out_2719050425223561698[1] = 1;
   out_2719050425223561698[2] = 0;
   out_2719050425223561698[3] = 0;
   out_2719050425223561698[4] = 0;
   out_2719050425223561698[5] = 0;
   out_2719050425223561698[6] = 0;
   out_2719050425223561698[7] = 0;
   out_2719050425223561698[8] = 0;
}
void h_28(double *state, double *unused, double *out_3199439864453025933) {
   out_3199439864453025933[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7801449442293092272) {
   out_7801449442293092272[0] = 1;
   out_7801449442293092272[1] = 0;
   out_7801449442293092272[2] = 0;
   out_7801449442293092272[3] = 0;
   out_7801449442293092272[4] = 0;
   out_7801449442293092272[5] = 0;
   out_7801449442293092272[6] = 0;
   out_7801449442293092272[7] = 0;
   out_7801449442293092272[8] = 0;
}
void h_31(double *state, double *unused, double *out_6876182662686600696) {
   out_6876182662686600696[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3069296860517753384) {
   out_3069296860517753384[0] = 0;
   out_3069296860517753384[1] = 0;
   out_3069296860517753384[2] = 0;
   out_3069296860517753384[3] = 0;
   out_3069296860517753384[4] = 0;
   out_3069296860517753384[5] = 0;
   out_3069296860517753384[6] = 0;
   out_3069296860517753384[7] = 0;
   out_3069296860517753384[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6118062787728240548) {
  err_fun(nom_x, delta_x, out_6118062787728240548);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9211263313597478507) {
  inv_err_fun(nom_x, true_x, out_9211263313597478507);
}
void car_H_mod_fun(double *state, double *out_4561024202480585870) {
  H_mod_fun(state, out_4561024202480585870);
}
void car_f_fun(double *state, double dt, double *out_6415819914950947790) {
  f_fun(state,  dt, out_6415819914950947790);
}
void car_F_fun(double *state, double dt, double *out_5710333805771084184) {
  F_fun(state,  dt, out_5710333805771084184);
}
void car_h_25(double *state, double *unused, double *out_5580444758777246286) {
  h_25(state, unused, out_5580444758777246286);
}
void car_H_25(double *state, double *unused, double *out_5747614728045202509) {
  H_25(state, unused, out_5747614728045202509);
}
void car_h_24(double *state, double *unused, double *out_5802005886931275559) {
  h_24(state, unused, out_5802005886931275559);
}
void car_H_24(double *state, double *unused, double *out_2189689129242879373) {
  H_24(state, unused, out_2189689129242879373);
}
void car_h_30(double *state, double *unused, double *out_4642539516321470839) {
  h_30(state, unused, out_4642539516321470839);
}
void car_H_30(double *state, double *unused, double *out_3229281769537953882) {
  H_30(state, unused, out_3229281769537953882);
}
void car_h_26(double *state, double *unused, double *out_8699869409390652045) {
  h_26(state, unused, out_8699869409390652045);
}
void car_H_26(double *state, double *unused, double *out_2443088758284401908) {
  H_26(state, unused, out_2443088758284401908);
}
void car_h_27(double *state, double *unused, double *out_2031696063376884196) {
  h_27(state, unused, out_2031696063376884196);
}
void car_H_27(double *state, double *unused, double *out_5404045081338378793) {
  H_27(state, unused, out_5404045081338378793);
}
void car_h_29(double *state, double *unused, double *out_1115445397672267117) {
  h_29(state, unused, out_1115445397672267117);
}
void car_H_29(double *state, double *unused, double *out_2719050425223561698) {
  H_29(state, unused, out_2719050425223561698);
}
void car_h_28(double *state, double *unused, double *out_3199439864453025933) {
  h_28(state, unused, out_3199439864453025933);
}
void car_H_28(double *state, double *unused, double *out_7801449442293092272) {
  H_28(state, unused, out_7801449442293092272);
}
void car_h_31(double *state, double *unused, double *out_6876182662686600696) {
  h_31(state, unused, out_6876182662686600696);
}
void car_H_31(double *state, double *unused, double *out_3069296860517753384) {
  H_31(state, unused, out_3069296860517753384);
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
