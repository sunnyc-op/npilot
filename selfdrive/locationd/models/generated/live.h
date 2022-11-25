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
void live_H(double *in_vec, double *out_3796868102103770979);
void live_err_fun(double *nom_x, double *delta_x, double *out_7507187861718128714);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5892106134665915380);
void live_H_mod_fun(double *state, double *out_4192746810752475091);
void live_f_fun(double *state, double dt, double *out_1205702479476172964);
void live_F_fun(double *state, double dt, double *out_1691520411734880419);
void live_h_4(double *state, double *unused, double *out_4767559188458182025);
void live_H_4(double *state, double *unused, double *out_7174878267494420528);
void live_h_9(double *state, double *unused, double *out_1897727502726321434);
void live_H_9(double *state, double *unused, double *out_3017710531139643045);
void live_h_10(double *state, double *unused, double *out_3147874293051680291);
void live_H_10(double *state, double *unused, double *out_4966352340262754741);
void live_h_12(double *state, double *unused, double *out_8895808911183884641);
void live_H_12(double *state, double *unused, double *out_7795977292542014195);
void live_h_31(double *state, double *unused, double *out_2639856322092056917);
void live_H_31(double *state, double *unused, double *out_7905203748842523712);
void live_h_32(double *state, double *unused, double *out_4440336405208144338);
void live_H_32(double *state, double *unused, double *out_8896700815156128302);
void live_h_13(double *state, double *unused, double *out_1287230079996720546);
void live_H_13(double *state, double *unused, double *out_469682175391566911);
void live_h_14(double *state, double *unused, double *out_1897727502726321434);
void live_H_14(double *state, double *unused, double *out_3017710531139643045);
void live_h_33(double *state, double *unused, double *out_8667330531082858134);
void live_H_33(double *state, double *unused, double *out_4754646744203666108);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}