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
void live_H(double *in_vec, double *out_7619092523166866453);
void live_err_fun(double *nom_x, double *delta_x, double *out_998547385383123003);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2382509876506872837);
void live_H_mod_fun(double *state, double *out_7212472148138734480);
void live_f_fun(double *state, double dt, double *out_6372746636710684500);
void live_F_fun(double *state, double dt, double *out_4542744388432953779);
void live_h_4(double *state, double *unused, double *out_2090931560907131531);
void live_H_4(double *state, double *unused, double *out_3256738396430010607);
void live_h_9(double *state, double *unused, double *out_2206083063208067569);
void live_H_9(double *state, double *unused, double *out_3497928043059601252);
void live_h_10(double *state, double *unused, double *out_4392644004237345603);
void live_H_10(double *state, double *unused, double *out_3849577756465794672);
void live_h_12(double *state, double *unused, double *out_3415239196926908075);
void live_H_12(double *state, double *unused, double *out_8276194804461972402);
void live_h_31(double *state, double *unused, double *out_1210493774321623088);
void live_H_31(double *state, double *unused, double *out_6623400453802617983);
void live_h_32(double *state, double *unused, double *out_4638131286751065894);
void live_H_32(double *state, double *unused, double *out_291464049891498373);
void live_h_13(double *state, double *unused, double *out_9036931581555587088);
void live_H_13(double *state, double *unused, double *out_5065496777539568443);
void live_h_14(double *state, double *unused, double *out_2206083063208067569);
void live_H_14(double *state, double *unused, double *out_3497928043059601252);
void live_h_33(double *state, double *unused, double *out_2399728465088226552);
void live_H_33(double *state, double *unused, double *out_8672786615268076029);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}