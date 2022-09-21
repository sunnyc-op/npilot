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
void live_H(double *in_vec, double *out_4100496910655232877);
void live_err_fun(double *nom_x, double *delta_x, double *out_5392627368994945959);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2141277271085920178);
void live_H_mod_fun(double *state, double *out_5237530958733850901);
void live_f_fun(double *state, double dt, double *out_3661643488610007850);
void live_F_fun(double *state, double dt, double *out_1859944115140833756);
void live_h_4(double *state, double *unused, double *out_477847467584938311);
void live_H_4(double *state, double *unused, double *out_1389179959631934882);
void live_h_9(double *state, double *unused, double *out_5537830471293940262);
void live_H_9(double *state, double *unused, double *out_1147990313002344237);
void live_h_10(double *state, double *unused, double *out_7345311412480798263);
void live_H_10(double *state, double *unused, double *out_1795520567236383254);
void live_h_12(double *state, double *unused, double *out_5793995701614192869);
void live_H_12(double *state, double *unused, double *out_3630276448400026913);
void live_h_31(double *state, double *unused, double *out_2788617611365882815);
void live_H_31(double *state, double *unused, double *out_1977482097740672494);
void live_h_32(double *state, double *unused, double *out_5885104912807721734);
void live_H_32(double *state, double *unused, double *out_7778786469948285052);
void live_h_13(double *state, double *unused, double *out_7010832056161989348);
void live_H_13(double *state, double *unused, double *out_6130077919333823603);
void live_h_14(double *state, double *unused, double *out_5537830471293940262);
void live_H_14(double *state, double *unused, double *out_1147990313002344237);
void live_h_33(double *state, double *unused, double *out_3088870080015929401);
void live_H_33(double *state, double *unused, double *out_5128039102379530098);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}