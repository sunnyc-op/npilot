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
void live_H(double *in_vec, double *out_654241702107958367);
void live_err_fun(double *nom_x, double *delta_x, double *out_5125877990615348777);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6267094538257137714);
void live_H_mod_fun(double *state, double *out_9122635722341517273);
void live_f_fun(double *state, double dt, double *out_8022326375520724887);
void live_F_fun(double *state, double dt, double *out_2606084813432559444);
void live_h_4(double *state, double *unused, double *out_3884732784060602165);
void live_H_4(double *state, double *unused, double *out_7998511476979429796);
void live_h_9(double *state, double *unused, double *out_4025118706756764653);
void live_H_9(double *state, double *unused, double *out_7757321830349839151);
void live_h_10(double *state, double *unused, double *out_7435728696533416274);
void live_H_10(double *state, double *unused, double *out_8190984688142623416);
void live_h_12(double *state, double *unused, double *out_328531660217095654);
void live_H_12(double *state, double *unused, double *out_7377412451931836129);
void live_h_31(double *state, double *unused, double *out_7725675484320907111);
void live_H_31(double *state, double *unused, double *out_4631849419606822420);
void live_h_32(double *state, double *unused, double *out_5963873259645966842);
void live_H_32(double *state, double *unused, double *out_1169454952600790138);
void live_h_13(double *state, double *unused, double *out_2168386746467312527);
void live_H_13(double *state, double *unused, double *out_200233473131994255);
void live_h_14(double *state, double *unused, double *out_4025118706756764653);
void live_H_14(double *state, double *unused, double *out_7757321830349839151);
void live_h_33(double *state, double *unused, double *out_7631564866611376192);
void live_H_33(double *state, double *unused, double *out_1481292414967964816);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}