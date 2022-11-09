#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2248172355834184046);
void live_err_fun(double *nom_x, double *delta_x, double *out_4352425929272528905);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3324796908486365865);
void live_H_mod_fun(double *state, double *out_7260159374251765835);
void live_f_fun(double *state, double dt, double *out_7179379146869147553);
void live_F_fun(double *state, double dt, double *out_4484335645246076076);
void live_h_4(double *state, double *unused, double *out_5933419603447310505);
void live_H_4(double *state, double *unused, double *out_1376987890976625277);
void live_h_9(double *state, double *unused, double *out_8989048969653774573);
void live_H_9(double *state, double *unused, double *out_1135798244347034632);
void live_h_10(double *state, double *unused, double *out_4050349310392222314);
void live_H_10(double *state, double *unused, double *out_1333477270818431092);
void live_h_12(double *state, double *unused, double *out_6044490304134937817);
void live_H_12(double *state, double *unused, double *out_3642468517055336518);
void live_h_31(double *state, double *unused, double *out_4494584311679769787);
void live_H_31(double *state, double *unused, double *out_1989674166395982099);
void live_h_32(double *state, double *unused, double *out_7173413103019206882);
void live_H_32(double *state, double *unused, double *out_2229025071275926971);
void live_h_13(double *state, double *unused, double *out_5739820833516471718);
void live_H_13(double *state, double *unused, double *out_7408568998853253593);
void live_h_14(double *state, double *unused, double *out_8989048969653774573);
void live_H_14(double *state, double *unused, double *out_1135798244347034632);
void live_h_33(double *state, double *unused, double *out_9195618204988076440);
void live_H_33(double *state, double *unused, double *out_5140231171034839703);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}