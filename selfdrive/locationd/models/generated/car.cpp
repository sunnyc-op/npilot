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
void err_fun(double *nom_x, double *delta_x, double *out_4877302666818620112) {
   out_4877302666818620112[0] = delta_x[0] + nom_x[0];
   out_4877302666818620112[1] = delta_x[1] + nom_x[1];
   out_4877302666818620112[2] = delta_x[2] + nom_x[2];
   out_4877302666818620112[3] = delta_x[3] + nom_x[3];
   out_4877302666818620112[4] = delta_x[4] + nom_x[4];
   out_4877302666818620112[5] = delta_x[5] + nom_x[5];
   out_4877302666818620112[6] = delta_x[6] + nom_x[6];
   out_4877302666818620112[7] = delta_x[7] + nom_x[7];
   out_4877302666818620112[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1202084109018656834) {
   out_1202084109018656834[0] = -nom_x[0] + true_x[0];
   out_1202084109018656834[1] = -nom_x[1] + true_x[1];
   out_1202084109018656834[2] = -nom_x[2] + true_x[2];
   out_1202084109018656834[3] = -nom_x[3] + true_x[3];
   out_1202084109018656834[4] = -nom_x[4] + true_x[4];
   out_1202084109018656834[5] = -nom_x[5] + true_x[5];
   out_1202084109018656834[6] = -nom_x[6] + true_x[6];
   out_1202084109018656834[7] = -nom_x[7] + true_x[7];
   out_1202084109018656834[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_359485901428299906) {
   out_359485901428299906[0] = 1.0;
   out_359485901428299906[1] = 0;
   out_359485901428299906[2] = 0;
   out_359485901428299906[3] = 0;
   out_359485901428299906[4] = 0;
   out_359485901428299906[5] = 0;
   out_359485901428299906[6] = 0;
   out_359485901428299906[7] = 0;
   out_359485901428299906[8] = 0;
   out_359485901428299906[9] = 0;
   out_359485901428299906[10] = 1.0;
   out_359485901428299906[11] = 0;
   out_359485901428299906[12] = 0;
   out_359485901428299906[13] = 0;
   out_359485901428299906[14] = 0;
   out_359485901428299906[15] = 0;
   out_359485901428299906[16] = 0;
   out_359485901428299906[17] = 0;
   out_359485901428299906[18] = 0;
   out_359485901428299906[19] = 0;
   out_359485901428299906[20] = 1.0;
   out_359485901428299906[21] = 0;
   out_359485901428299906[22] = 0;
   out_359485901428299906[23] = 0;
   out_359485901428299906[24] = 0;
   out_359485901428299906[25] = 0;
   out_359485901428299906[26] = 0;
   out_359485901428299906[27] = 0;
   out_359485901428299906[28] = 0;
   out_359485901428299906[29] = 0;
   out_359485901428299906[30] = 1.0;
   out_359485901428299906[31] = 0;
   out_359485901428299906[32] = 0;
   out_359485901428299906[33] = 0;
   out_359485901428299906[34] = 0;
   out_359485901428299906[35] = 0;
   out_359485901428299906[36] = 0;
   out_359485901428299906[37] = 0;
   out_359485901428299906[38] = 0;
   out_359485901428299906[39] = 0;
   out_359485901428299906[40] = 1.0;
   out_359485901428299906[41] = 0;
   out_359485901428299906[42] = 0;
   out_359485901428299906[43] = 0;
   out_359485901428299906[44] = 0;
   out_359485901428299906[45] = 0;
   out_359485901428299906[46] = 0;
   out_359485901428299906[47] = 0;
   out_359485901428299906[48] = 0;
   out_359485901428299906[49] = 0;
   out_359485901428299906[50] = 1.0;
   out_359485901428299906[51] = 0;
   out_359485901428299906[52] = 0;
   out_359485901428299906[53] = 0;
   out_359485901428299906[54] = 0;
   out_359485901428299906[55] = 0;
   out_359485901428299906[56] = 0;
   out_359485901428299906[57] = 0;
   out_359485901428299906[58] = 0;
   out_359485901428299906[59] = 0;
   out_359485901428299906[60] = 1.0;
   out_359485901428299906[61] = 0;
   out_359485901428299906[62] = 0;
   out_359485901428299906[63] = 0;
   out_359485901428299906[64] = 0;
   out_359485901428299906[65] = 0;
   out_359485901428299906[66] = 0;
   out_359485901428299906[67] = 0;
   out_359485901428299906[68] = 0;
   out_359485901428299906[69] = 0;
   out_359485901428299906[70] = 1.0;
   out_359485901428299906[71] = 0;
   out_359485901428299906[72] = 0;
   out_359485901428299906[73] = 0;
   out_359485901428299906[74] = 0;
   out_359485901428299906[75] = 0;
   out_359485901428299906[76] = 0;
   out_359485901428299906[77] = 0;
   out_359485901428299906[78] = 0;
   out_359485901428299906[79] = 0;
   out_359485901428299906[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1034882919924552313) {
   out_1034882919924552313[0] = state[0];
   out_1034882919924552313[1] = state[1];
   out_1034882919924552313[2] = state[2];
   out_1034882919924552313[3] = state[3];
   out_1034882919924552313[4] = state[4];
   out_1034882919924552313[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1034882919924552313[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1034882919924552313[7] = state[7];
   out_1034882919924552313[8] = state[8];
}
void F_fun(double *state, double dt, double *out_9047421667853216185) {
   out_9047421667853216185[0] = 1;
   out_9047421667853216185[1] = 0;
   out_9047421667853216185[2] = 0;
   out_9047421667853216185[3] = 0;
   out_9047421667853216185[4] = 0;
   out_9047421667853216185[5] = 0;
   out_9047421667853216185[6] = 0;
   out_9047421667853216185[7] = 0;
   out_9047421667853216185[8] = 0;
   out_9047421667853216185[9] = 0;
   out_9047421667853216185[10] = 1;
   out_9047421667853216185[11] = 0;
   out_9047421667853216185[12] = 0;
   out_9047421667853216185[13] = 0;
   out_9047421667853216185[14] = 0;
   out_9047421667853216185[15] = 0;
   out_9047421667853216185[16] = 0;
   out_9047421667853216185[17] = 0;
   out_9047421667853216185[18] = 0;
   out_9047421667853216185[19] = 0;
   out_9047421667853216185[20] = 1;
   out_9047421667853216185[21] = 0;
   out_9047421667853216185[22] = 0;
   out_9047421667853216185[23] = 0;
   out_9047421667853216185[24] = 0;
   out_9047421667853216185[25] = 0;
   out_9047421667853216185[26] = 0;
   out_9047421667853216185[27] = 0;
   out_9047421667853216185[28] = 0;
   out_9047421667853216185[29] = 0;
   out_9047421667853216185[30] = 1;
   out_9047421667853216185[31] = 0;
   out_9047421667853216185[32] = 0;
   out_9047421667853216185[33] = 0;
   out_9047421667853216185[34] = 0;
   out_9047421667853216185[35] = 0;
   out_9047421667853216185[36] = 0;
   out_9047421667853216185[37] = 0;
   out_9047421667853216185[38] = 0;
   out_9047421667853216185[39] = 0;
   out_9047421667853216185[40] = 1;
   out_9047421667853216185[41] = 0;
   out_9047421667853216185[42] = 0;
   out_9047421667853216185[43] = 0;
   out_9047421667853216185[44] = 0;
   out_9047421667853216185[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_9047421667853216185[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_9047421667853216185[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9047421667853216185[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_9047421667853216185[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_9047421667853216185[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_9047421667853216185[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_9047421667853216185[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_9047421667853216185[53] = -9.8000000000000007*dt;
   out_9047421667853216185[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_9047421667853216185[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_9047421667853216185[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9047421667853216185[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9047421667853216185[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_9047421667853216185[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_9047421667853216185[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_9047421667853216185[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_9047421667853216185[62] = 0;
   out_9047421667853216185[63] = 0;
   out_9047421667853216185[64] = 0;
   out_9047421667853216185[65] = 0;
   out_9047421667853216185[66] = 0;
   out_9047421667853216185[67] = 0;
   out_9047421667853216185[68] = 0;
   out_9047421667853216185[69] = 0;
   out_9047421667853216185[70] = 1;
   out_9047421667853216185[71] = 0;
   out_9047421667853216185[72] = 0;
   out_9047421667853216185[73] = 0;
   out_9047421667853216185[74] = 0;
   out_9047421667853216185[75] = 0;
   out_9047421667853216185[76] = 0;
   out_9047421667853216185[77] = 0;
   out_9047421667853216185[78] = 0;
   out_9047421667853216185[79] = 0;
   out_9047421667853216185[80] = 1;
}
void h_25(double *state, double *unused, double *out_1301771313225191319) {
   out_1301771313225191319[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2116846282827690996) {
   out_2116846282827690996[0] = 0;
   out_2116846282827690996[1] = 0;
   out_2116846282827690996[2] = 0;
   out_2116846282827690996[3] = 0;
   out_2116846282827690996[4] = 0;
   out_2116846282827690996[5] = 0;
   out_2116846282827690996[6] = 1;
   out_2116846282827690996[7] = 0;
   out_2116846282827690996[8] = 0;
}
void h_24(double *state, double *unused, double *out_1965321636609380214) {
   out_1965321636609380214[0] = state[4];
   out_1965321636609380214[1] = state[5];
}
void H_24(double *state, double *unused, double *out_60368140779458977) {
   out_60368140779458977[0] = 0;
   out_60368140779458977[1] = 0;
   out_60368140779458977[2] = 0;
   out_60368140779458977[3] = 0;
   out_60368140779458977[4] = 1;
   out_60368140779458977[5] = 0;
   out_60368140779458977[6] = 0;
   out_60368140779458977[7] = 0;
   out_60368140779458977[8] = 0;
   out_60368140779458977[9] = 0;
   out_60368140779458977[10] = 0;
   out_60368140779458977[11] = 0;
   out_60368140779458977[12] = 0;
   out_60368140779458977[13] = 0;
   out_60368140779458977[14] = 1;
   out_60368140779458977[15] = 0;
   out_60368140779458977[16] = 0;
   out_60368140779458977[17] = 0;
}
void h_30(double *state, double *unused, double *out_7665365291527007180) {
   out_7665365291527007180[0] = state[4];
}
void H_30(double *state, double *unused, double *out_401486675679557631) {
   out_401486675679557631[0] = 0;
   out_401486675679557631[1] = 0;
   out_401486675679557631[2] = 0;
   out_401486675679557631[3] = 0;
   out_401486675679557631[4] = 1;
   out_401486675679557631[5] = 0;
   out_401486675679557631[6] = 0;
   out_401486675679557631[7] = 0;
   out_401486675679557631[8] = 0;
}
void h_26(double *state, double *unused, double *out_8833109092603148917) {
   out_8833109092603148917[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5858349601701747220) {
   out_5858349601701747220[0] = 0;
   out_5858349601701747220[1] = 0;
   out_5858349601701747220[2] = 0;
   out_5858349601701747220[3] = 0;
   out_5858349601701747220[4] = 0;
   out_5858349601701747220[5] = 0;
   out_5858349601701747220[6] = 0;
   out_5858349601701747220[7] = 1;
   out_5858349601701747220[8] = 0;
}
void h_27(double *state, double *unused, double *out_5700991390567874460) {
   out_5700991390567874460[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1773276636120867280) {
   out_1773276636120867280[0] = 0;
   out_1773276636120867280[1] = 0;
   out_1773276636120867280[2] = 0;
   out_1773276636120867280[3] = 1;
   out_1773276636120867280[4] = 0;
   out_1773276636120867280[5] = 0;
   out_1773276636120867280[6] = 0;
   out_1773276636120867280[7] = 0;
   out_1773276636120867280[8] = 0;
}
void h_29(double *state, double *unused, double *out_5555471566347219489) {
   out_5555471566347219489[0] = state[1];
}
void H_29(double *state, double *unused, double *out_911718019993949815) {
   out_911718019993949815[0] = 0;
   out_911718019993949815[1] = 1;
   out_911718019993949815[2] = 0;
   out_911718019993949815[3] = 0;
   out_911718019993949815[4] = 0;
   out_911718019993949815[5] = 0;
   out_911718019993949815[6] = 0;
   out_911718019993949815[7] = 0;
   out_911718019993949815[8] = 0;
}
void h_28(double *state, double *unused, double *out_699318122907028815) {
   out_699318122907028815[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4170680997075580759) {
   out_4170680997075580759[0] = 1;
   out_4170680997075580759[1] = 0;
   out_4170680997075580759[2] = 0;
   out_4170680997075580759[3] = 0;
   out_4170680997075580759[4] = 0;
   out_4170680997075580759[5] = 0;
   out_4170680997075580759[6] = 0;
   out_4170680997075580759[7] = 0;
   out_4170680997075580759[8] = 0;
}
void h_31(double *state, double *unused, double *out_1458957981576320464) {
   out_1458957981576320464[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6484557703935098696) {
   out_6484557703935098696[0] = 0;
   out_6484557703935098696[1] = 0;
   out_6484557703935098696[2] = 0;
   out_6484557703935098696[3] = 0;
   out_6484557703935098696[4] = 0;
   out_6484557703935098696[5] = 0;
   out_6484557703935098696[6] = 0;
   out_6484557703935098696[7] = 0;
   out_6484557703935098696[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_4877302666818620112) {
  err_fun(nom_x, delta_x, out_4877302666818620112);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1202084109018656834) {
  inv_err_fun(nom_x, true_x, out_1202084109018656834);
}
void car_H_mod_fun(double *state, double *out_359485901428299906) {
  H_mod_fun(state, out_359485901428299906);
}
void car_f_fun(double *state, double dt, double *out_1034882919924552313) {
  f_fun(state,  dt, out_1034882919924552313);
}
void car_F_fun(double *state, double dt, double *out_9047421667853216185) {
  F_fun(state,  dt, out_9047421667853216185);
}
void car_h_25(double *state, double *unused, double *out_1301771313225191319) {
  h_25(state, unused, out_1301771313225191319);
}
void car_H_25(double *state, double *unused, double *out_2116846282827690996) {
  H_25(state, unused, out_2116846282827690996);
}
void car_h_24(double *state, double *unused, double *out_1965321636609380214) {
  h_24(state, unused, out_1965321636609380214);
}
void car_H_24(double *state, double *unused, double *out_60368140779458977) {
  H_24(state, unused, out_60368140779458977);
}
void car_h_30(double *state, double *unused, double *out_7665365291527007180) {
  h_30(state, unused, out_7665365291527007180);
}
void car_H_30(double *state, double *unused, double *out_401486675679557631) {
  H_30(state, unused, out_401486675679557631);
}
void car_h_26(double *state, double *unused, double *out_8833109092603148917) {
  h_26(state, unused, out_8833109092603148917);
}
void car_H_26(double *state, double *unused, double *out_5858349601701747220) {
  H_26(state, unused, out_5858349601701747220);
}
void car_h_27(double *state, double *unused, double *out_5700991390567874460) {
  h_27(state, unused, out_5700991390567874460);
}
void car_H_27(double *state, double *unused, double *out_1773276636120867280) {
  H_27(state, unused, out_1773276636120867280);
}
void car_h_29(double *state, double *unused, double *out_5555471566347219489) {
  h_29(state, unused, out_5555471566347219489);
}
void car_H_29(double *state, double *unused, double *out_911718019993949815) {
  H_29(state, unused, out_911718019993949815);
}
void car_h_28(double *state, double *unused, double *out_699318122907028815) {
  h_28(state, unused, out_699318122907028815);
}
void car_H_28(double *state, double *unused, double *out_4170680997075580759) {
  H_28(state, unused, out_4170680997075580759);
}
void car_h_31(double *state, double *unused, double *out_1458957981576320464) {
  h_31(state, unused, out_1458957981576320464);
}
void car_H_31(double *state, double *unused, double *out_6484557703935098696) {
  H_31(state, unused, out_6484557703935098696);
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
