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
void live_H(double *in_vec, double *out_1456494305438152637);
void live_err_fun(double *nom_x, double *delta_x, double *out_3733468540891634108);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5118104397419630567);
void live_H_mod_fun(double *state, double *out_4375928136598789824);
void live_f_fun(double *state, double dt, double *out_394294405565155546);
void live_F_fun(double *state, double dt, double *out_3869301692626177806);
void live_h_4(double *state, double *unused, double *out_8196831371611339857);
void live_H_4(double *state, double *unused, double *out_8146711423965938697);
void live_h_9(double *state, double *unused, double *out_5250036663816455848);
void live_H_9(double *state, double *unused, double *out_3012813714479165449);
void live_h_10(double *state, double *unused, double *out_5295172777092500790);
void live_H_10(double *state, double *unused, double *out_2674330437399666525);
void live_h_12(double *state, double *unused, double *out_628823918747259819);
void live_H_12(double *state, double *unused, double *out_2632904336061162427);
void live_h_31(double *state, double *unused, double *out_1340437049358684502);
void live_H_31(double *state, double *unused, double *out_112658696263851282);
void live_h_32(double *state, double *unused, double *out_7994667005165214277);
void live_H_32(double *state, double *unused, double *out_5913963068471463840);
void live_h_13(double *state, double *unused, double *out_260821041099461451);
void live_H_13(double *state, double *unused, double *out_1876957704875076489);
void live_h_14(double *state, double *unused, double *out_5250036663816455848);
void live_H_14(double *state, double *unused, double *out_3012813714479165449);
void live_h_33(double *state, double *unused, double *out_6533104562095675023);
void live_H_33(double *state, double *unused, double *out_3263215700902708886);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}