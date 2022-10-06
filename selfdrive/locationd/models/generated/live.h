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
void live_H(double *in_vec, double *out_8046515251328236826);
void live_err_fun(double *nom_x, double *delta_x, double *out_4842925533229794788);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8183795172776349593);
void live_H_mod_fun(double *state, double *out_3099639700326057884);
void live_f_fun(double *state, double dt, double *out_4133204539928242426);
void live_F_fun(double *state, double dt, double *out_3819195157595720780);
void live_h_4(double *state, double *unused, double *out_9156380342399205220);
void live_H_4(double *state, double *unused, double *out_5201801911868158922);
void live_h_9(double *state, double *unused, double *out_4869497960647677992);
void live_H_9(double *state, double *unused, double *out_5442991558497749567);
void live_h_10(double *state, double *unused, double *out_2938596904775568349);
void live_H_10(double *state, double *unused, double *out_6909870786769237186);
void live_h_12(double *state, double *unused, double *out_1480307932043820451);
void live_H_12(double *state, double *unused, double *out_8225485753809430899);
void live_h_31(double *state, double *unused, double *out_4066929957199277186);
void live_H_31(double *state, double *unused, double *out_5479922721484417190);
void live_h_32(double *state, double *unused, double *out_5870711691875296376);
void live_H_32(double *state, double *unused, double *out_4349764731568857228);
void live_h_13(double *state, double *unused, double *out_2253315789354612603);
void live_H_13(double *state, double *unused, double *out_7875262776952249890);
void live_h_14(double *state, double *unused, double *out_4869497960647677992);
void live_H_14(double *state, double *unused, double *out_5442991558497749567);
void live_h_33(double *state, double *unused, double *out_8770641920430350887);
void live_H_33(double *state, double *unused, double *out_2329365716845559586);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}