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
void live_H(double *in_vec, double *out_7314429967761925320);
void live_err_fun(double *nom_x, double *delta_x, double *out_5063828239748852282);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3766884446287441611);
void live_H_mod_fun(double *state, double *out_5299362412229614450);
void live_f_fun(double *state, double dt, double *out_6438217386767210752);
void live_F_fun(double *state, double dt, double *out_8711821868833026606);
void live_h_4(double *state, double *unused, double *out_6699434746503508921);
void live_H_4(double *state, double *unused, double *out_4765815366198060142);
void live_h_9(double *state, double *unused, double *out_2938772362047964784);
void live_H_9(double *state, double *unused, double *out_2521403569066387328);
void live_h_10(double *state, double *unused, double *out_572592492373180910);
void live_H_10(double *state, double *unused, double *out_5831623508457838178);
void live_h_12(double *state, double *unused, double *out_6087461880107305338);
void live_H_12(double *state, double *unused, double *out_253641041833901653);
void live_h_31(double *state, double *unused, double *out_6848381938966983359);
void live_H_31(double *state, double *unused, double *out_1399153308825452766);
void live_h_32(double *state, double *unused, double *out_5512723683214713035);
void live_H_32(double *state, double *unused, double *out_1267988523884712297);
void live_h_13(double *state, double *unused, double *out_8440708348654386059);
void live_H_13(double *state, double *unused, double *out_5914274422995110640);
void live_h_14(double *state, double *unused, double *out_2938772362047964784);
void live_H_14(double *state, double *unused, double *out_2521403569066387328);
void live_h_33(double *state, double *unused, double *out_3995606346095382009);
void live_H_33(double *state, double *unused, double *out_1751403695813404838);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}