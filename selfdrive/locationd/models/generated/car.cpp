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
void err_fun(double *nom_x, double *delta_x, double *out_4989488229479570161) {
   out_4989488229479570161[0] = delta_x[0] + nom_x[0];
   out_4989488229479570161[1] = delta_x[1] + nom_x[1];
   out_4989488229479570161[2] = delta_x[2] + nom_x[2];
   out_4989488229479570161[3] = delta_x[3] + nom_x[3];
   out_4989488229479570161[4] = delta_x[4] + nom_x[4];
   out_4989488229479570161[5] = delta_x[5] + nom_x[5];
   out_4989488229479570161[6] = delta_x[6] + nom_x[6];
   out_4989488229479570161[7] = delta_x[7] + nom_x[7];
   out_4989488229479570161[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4379738640735284012) {
   out_4379738640735284012[0] = -nom_x[0] + true_x[0];
   out_4379738640735284012[1] = -nom_x[1] + true_x[1];
   out_4379738640735284012[2] = -nom_x[2] + true_x[2];
   out_4379738640735284012[3] = -nom_x[3] + true_x[3];
   out_4379738640735284012[4] = -nom_x[4] + true_x[4];
   out_4379738640735284012[5] = -nom_x[5] + true_x[5];
   out_4379738640735284012[6] = -nom_x[6] + true_x[6];
   out_4379738640735284012[7] = -nom_x[7] + true_x[7];
   out_4379738640735284012[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2445474491913214453) {
   out_2445474491913214453[0] = 1.0;
   out_2445474491913214453[1] = 0;
   out_2445474491913214453[2] = 0;
   out_2445474491913214453[3] = 0;
   out_2445474491913214453[4] = 0;
   out_2445474491913214453[5] = 0;
   out_2445474491913214453[6] = 0;
   out_2445474491913214453[7] = 0;
   out_2445474491913214453[8] = 0;
   out_2445474491913214453[9] = 0;
   out_2445474491913214453[10] = 1.0;
   out_2445474491913214453[11] = 0;
   out_2445474491913214453[12] = 0;
   out_2445474491913214453[13] = 0;
   out_2445474491913214453[14] = 0;
   out_2445474491913214453[15] = 0;
   out_2445474491913214453[16] = 0;
   out_2445474491913214453[17] = 0;
   out_2445474491913214453[18] = 0;
   out_2445474491913214453[19] = 0;
   out_2445474491913214453[20] = 1.0;
   out_2445474491913214453[21] = 0;
   out_2445474491913214453[22] = 0;
   out_2445474491913214453[23] = 0;
   out_2445474491913214453[24] = 0;
   out_2445474491913214453[25] = 0;
   out_2445474491913214453[26] = 0;
   out_2445474491913214453[27] = 0;
   out_2445474491913214453[28] = 0;
   out_2445474491913214453[29] = 0;
   out_2445474491913214453[30] = 1.0;
   out_2445474491913214453[31] = 0;
   out_2445474491913214453[32] = 0;
   out_2445474491913214453[33] = 0;
   out_2445474491913214453[34] = 0;
   out_2445474491913214453[35] = 0;
   out_2445474491913214453[36] = 0;
   out_2445474491913214453[37] = 0;
   out_2445474491913214453[38] = 0;
   out_2445474491913214453[39] = 0;
   out_2445474491913214453[40] = 1.0;
   out_2445474491913214453[41] = 0;
   out_2445474491913214453[42] = 0;
   out_2445474491913214453[43] = 0;
   out_2445474491913214453[44] = 0;
   out_2445474491913214453[45] = 0;
   out_2445474491913214453[46] = 0;
   out_2445474491913214453[47] = 0;
   out_2445474491913214453[48] = 0;
   out_2445474491913214453[49] = 0;
   out_2445474491913214453[50] = 1.0;
   out_2445474491913214453[51] = 0;
   out_2445474491913214453[52] = 0;
   out_2445474491913214453[53] = 0;
   out_2445474491913214453[54] = 0;
   out_2445474491913214453[55] = 0;
   out_2445474491913214453[56] = 0;
   out_2445474491913214453[57] = 0;
   out_2445474491913214453[58] = 0;
   out_2445474491913214453[59] = 0;
   out_2445474491913214453[60] = 1.0;
   out_2445474491913214453[61] = 0;
   out_2445474491913214453[62] = 0;
   out_2445474491913214453[63] = 0;
   out_2445474491913214453[64] = 0;
   out_2445474491913214453[65] = 0;
   out_2445474491913214453[66] = 0;
   out_2445474491913214453[67] = 0;
   out_2445474491913214453[68] = 0;
   out_2445474491913214453[69] = 0;
   out_2445474491913214453[70] = 1.0;
   out_2445474491913214453[71] = 0;
   out_2445474491913214453[72] = 0;
   out_2445474491913214453[73] = 0;
   out_2445474491913214453[74] = 0;
   out_2445474491913214453[75] = 0;
   out_2445474491913214453[76] = 0;
   out_2445474491913214453[77] = 0;
   out_2445474491913214453[78] = 0;
   out_2445474491913214453[79] = 0;
   out_2445474491913214453[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6392909334638572965) {
   out_6392909334638572965[0] = state[0];
   out_6392909334638572965[1] = state[1];
   out_6392909334638572965[2] = state[2];
   out_6392909334638572965[3] = state[3];
   out_6392909334638572965[4] = state[4];
   out_6392909334638572965[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6392909334638572965[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6392909334638572965[7] = state[7];
   out_6392909334638572965[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5263796904319004870) {
   out_5263796904319004870[0] = 1;
   out_5263796904319004870[1] = 0;
   out_5263796904319004870[2] = 0;
   out_5263796904319004870[3] = 0;
   out_5263796904319004870[4] = 0;
   out_5263796904319004870[5] = 0;
   out_5263796904319004870[6] = 0;
   out_5263796904319004870[7] = 0;
   out_5263796904319004870[8] = 0;
   out_5263796904319004870[9] = 0;
   out_5263796904319004870[10] = 1;
   out_5263796904319004870[11] = 0;
   out_5263796904319004870[12] = 0;
   out_5263796904319004870[13] = 0;
   out_5263796904319004870[14] = 0;
   out_5263796904319004870[15] = 0;
   out_5263796904319004870[16] = 0;
   out_5263796904319004870[17] = 0;
   out_5263796904319004870[18] = 0;
   out_5263796904319004870[19] = 0;
   out_5263796904319004870[20] = 1;
   out_5263796904319004870[21] = 0;
   out_5263796904319004870[22] = 0;
   out_5263796904319004870[23] = 0;
   out_5263796904319004870[24] = 0;
   out_5263796904319004870[25] = 0;
   out_5263796904319004870[26] = 0;
   out_5263796904319004870[27] = 0;
   out_5263796904319004870[28] = 0;
   out_5263796904319004870[29] = 0;
   out_5263796904319004870[30] = 1;
   out_5263796904319004870[31] = 0;
   out_5263796904319004870[32] = 0;
   out_5263796904319004870[33] = 0;
   out_5263796904319004870[34] = 0;
   out_5263796904319004870[35] = 0;
   out_5263796904319004870[36] = 0;
   out_5263796904319004870[37] = 0;
   out_5263796904319004870[38] = 0;
   out_5263796904319004870[39] = 0;
   out_5263796904319004870[40] = 1;
   out_5263796904319004870[41] = 0;
   out_5263796904319004870[42] = 0;
   out_5263796904319004870[43] = 0;
   out_5263796904319004870[44] = 0;
   out_5263796904319004870[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5263796904319004870[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5263796904319004870[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5263796904319004870[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5263796904319004870[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5263796904319004870[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5263796904319004870[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5263796904319004870[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5263796904319004870[53] = -9.8000000000000007*dt;
   out_5263796904319004870[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5263796904319004870[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5263796904319004870[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5263796904319004870[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5263796904319004870[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5263796904319004870[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5263796904319004870[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5263796904319004870[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5263796904319004870[62] = 0;
   out_5263796904319004870[63] = 0;
   out_5263796904319004870[64] = 0;
   out_5263796904319004870[65] = 0;
   out_5263796904319004870[66] = 0;
   out_5263796904319004870[67] = 0;
   out_5263796904319004870[68] = 0;
   out_5263796904319004870[69] = 0;
   out_5263796904319004870[70] = 1;
   out_5263796904319004870[71] = 0;
   out_5263796904319004870[72] = 0;
   out_5263796904319004870[73] = 0;
   out_5263796904319004870[74] = 0;
   out_5263796904319004870[75] = 0;
   out_5263796904319004870[76] = 0;
   out_5263796904319004870[77] = 0;
   out_5263796904319004870[78] = 0;
   out_5263796904319004870[79] = 0;
   out_5263796904319004870[80] = 1;
}
void h_25(double *state, double *unused, double *out_5333733597242535689) {
   out_5333733597242535689[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1984779620623110074) {
   out_1984779620623110074[0] = 0;
   out_1984779620623110074[1] = 0;
   out_1984779620623110074[2] = 0;
   out_1984779620623110074[3] = 0;
   out_1984779620623110074[4] = 0;
   out_1984779620623110074[5] = 0;
   out_1984779620623110074[6] = 1;
   out_1984779620623110074[7] = 0;
   out_1984779620623110074[8] = 0;
}
void h_24(double *state, double *unused, double *out_7261268887423112988) {
   out_7261268887423112988[0] = state[4];
   out_7261268887423112988[1] = state[5];
}
void H_24(double *state, double *unused, double *out_5542705219425433210) {
   out_5542705219425433210[0] = 0;
   out_5542705219425433210[1] = 0;
   out_5542705219425433210[2] = 0;
   out_5542705219425433210[3] = 0;
   out_5542705219425433210[4] = 1;
   out_5542705219425433210[5] = 0;
   out_5542705219425433210[6] = 0;
   out_5542705219425433210[7] = 0;
   out_5542705219425433210[8] = 0;
   out_5542705219425433210[9] = 0;
   out_5542705219425433210[10] = 0;
   out_5542705219425433210[11] = 0;
   out_5542705219425433210[12] = 0;
   out_5542705219425433210[13] = 0;
   out_5542705219425433210[14] = 1;
   out_5542705219425433210[15] = 0;
   out_5542705219425433210[16] = 0;
   out_5542705219425433210[17] = 0;
}
void h_30(double *state, double *unused, double *out_1254094329233759474) {
   out_1254094329233759474[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2542916709504498124) {
   out_2542916709504498124[0] = 0;
   out_2542916709504498124[1] = 0;
   out_2542916709504498124[2] = 0;
   out_2542916709504498124[3] = 0;
   out_2542916709504498124[4] = 1;
   out_2542916709504498124[5] = 0;
   out_2542916709504498124[6] = 0;
   out_2542916709504498124[7] = 0;
   out_2542916709504498124[8] = 0;
}
void h_26(double *state, double *unused, double *out_3858499650042457125) {
   out_3858499650042457125[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1756723698250946150) {
   out_1756723698250946150[0] = 0;
   out_1756723698250946150[1] = 0;
   out_1756723698250946150[2] = 0;
   out_1756723698250946150[3] = 0;
   out_1756723698250946150[4] = 0;
   out_1756723698250946150[5] = 0;
   out_1756723698250946150[6] = 0;
   out_1756723698250946150[7] = 1;
   out_1756723698250946150[8] = 0;
}
void h_27(double *state, double *unused, double *out_7771143240580985364) {
   out_7771143240580985364[0] = state[3];
}
void H_27(double *state, double *unused, double *out_319322638320554907) {
   out_319322638320554907[0] = 0;
   out_319322638320554907[1] = 0;
   out_319322638320554907[2] = 0;
   out_319322638320554907[3] = 1;
   out_319322638320554907[4] = 0;
   out_319322638320554907[5] = 0;
   out_319322638320554907[6] = 0;
   out_319322638320554907[7] = 0;
   out_319322638320554907[8] = 0;
}
void h_29(double *state, double *unused, double *out_8190268924592470331) {
   out_8190268924592470331[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2032685365190105940) {
   out_2032685365190105940[0] = 0;
   out_2032685365190105940[1] = 1;
   out_2032685365190105940[2] = 0;
   out_2032685365190105940[3] = 0;
   out_2032685365190105940[4] = 0;
   out_2032685365190105940[5] = 0;
   out_2032685365190105940[6] = 0;
   out_2032685365190105940[7] = 0;
   out_2032685365190105940[8] = 0;
}
void h_28(double *state, double *unused, double *out_5673927565467720607) {
   out_5673927565467720607[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7115084382259636514) {
   out_7115084382259636514[0] = 1;
   out_7115084382259636514[1] = 0;
   out_7115084382259636514[2] = 0;
   out_7115084382259636514[3] = 0;
   out_7115084382259636514[4] = 0;
   out_7115084382259636514[5] = 0;
   out_7115084382259636514[6] = 0;
   out_7115084382259636514[7] = 0;
   out_7115084382259636514[8] = 0;
}
void h_31(double *state, double *unused, double *out_8330126522076446849) {
   out_8330126522076446849[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2382931800484297626) {
   out_2382931800484297626[0] = 0;
   out_2382931800484297626[1] = 0;
   out_2382931800484297626[2] = 0;
   out_2382931800484297626[3] = 0;
   out_2382931800484297626[4] = 0;
   out_2382931800484297626[5] = 0;
   out_2382931800484297626[6] = 0;
   out_2382931800484297626[7] = 0;
   out_2382931800484297626[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4989488229479570161) {
  err_fun(nom_x, delta_x, out_4989488229479570161);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4379738640735284012) {
  inv_err_fun(nom_x, true_x, out_4379738640735284012);
}
void car_H_mod_fun(double *state, double *out_2445474491913214453) {
  H_mod_fun(state, out_2445474491913214453);
}
void car_f_fun(double *state, double dt, double *out_6392909334638572965) {
  f_fun(state,  dt, out_6392909334638572965);
}
void car_F_fun(double *state, double dt, double *out_5263796904319004870) {
  F_fun(state,  dt, out_5263796904319004870);
}
void car_h_25(double *state, double *unused, double *out_5333733597242535689) {
  h_25(state, unused, out_5333733597242535689);
}
void car_H_25(double *state, double *unused, double *out_1984779620623110074) {
  H_25(state, unused, out_1984779620623110074);
}
void car_h_24(double *state, double *unused, double *out_7261268887423112988) {
  h_24(state, unused, out_7261268887423112988);
}
void car_H_24(double *state, double *unused, double *out_5542705219425433210) {
  H_24(state, unused, out_5542705219425433210);
}
void car_h_30(double *state, double *unused, double *out_1254094329233759474) {
  h_30(state, unused, out_1254094329233759474);
}
void car_H_30(double *state, double *unused, double *out_2542916709504498124) {
  H_30(state, unused, out_2542916709504498124);
}
void car_h_26(double *state, double *unused, double *out_3858499650042457125) {
  h_26(state, unused, out_3858499650042457125);
}
void car_H_26(double *state, double *unused, double *out_1756723698250946150) {
  H_26(state, unused, out_1756723698250946150);
}
void car_h_27(double *state, double *unused, double *out_7771143240580985364) {
  h_27(state, unused, out_7771143240580985364);
}
void car_H_27(double *state, double *unused, double *out_319322638320554907) {
  H_27(state, unused, out_319322638320554907);
}
void car_h_29(double *state, double *unused, double *out_8190268924592470331) {
  h_29(state, unused, out_8190268924592470331);
}
void car_H_29(double *state, double *unused, double *out_2032685365190105940) {
  H_29(state, unused, out_2032685365190105940);
}
void car_h_28(double *state, double *unused, double *out_5673927565467720607) {
  h_28(state, unused, out_5673927565467720607);
}
void car_H_28(double *state, double *unused, double *out_7115084382259636514) {
  H_28(state, unused, out_7115084382259636514);
}
void car_h_31(double *state, double *unused, double *out_8330126522076446849) {
  h_31(state, unused, out_8330126522076446849);
}
void car_H_31(double *state, double *unused, double *out_2382931800484297626) {
  H_31(state, unused, out_2382931800484297626);
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
