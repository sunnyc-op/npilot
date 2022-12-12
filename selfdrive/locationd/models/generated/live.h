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
void live_H(double *in_vec, double *out_4177389968757708308);
void live_err_fun(double *nom_x, double *delta_x, double *out_8273844871437731763);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8003987836581394650);
void live_H_mod_fun(double *state, double *out_6364386299083317310);
void live_f_fun(double *state, double dt, double *out_9097766932839021284);
void live_F_fun(double *state, double dt, double *out_8990022318288498037);
void live_h_4(double *state, double *unused, double *out_8623575298982420939);
void live_H_4(double *state, double *unused, double *out_5629170196859374975);
void live_h_9(double *state, double *unused, double *out_653461898791692623);
void live_H_9(double *state, double *unused, double *out_5387980550229784330);
void live_h_10(double *state, double *unused, double *out_6181034955312552132);
void live_H_10(double *state, double *unused, double *out_1525706263490996231);
void live_h_12(double *state, double *unused, double *out_6610386752139946599);
void live_H_12(double *state, double *unused, double *out_609713788827413180);
void live_h_31(double *state, double *unused, double *out_7270034002460865856);
void live_H_31(double *state, double *unused, double *out_2262508139486767599);
void live_h_32(double *state, double *unused, double *out_5218588835983853765);
void live_H_32(double *state, double *unused, double *out_9177372643180883955);
void live_h_13(double *state, double *unused, double *out_4399227080735883910);
void live_H_13(double *state, double *unused, double *out_3530497425735196069);
void live_h_14(double *state, double *unused, double *out_653461898791692623);
void live_H_14(double *state, double *unused, double *out_5387980550229784330);
void live_h_33(double *state, double *unused, double *out_7696770941766349025);
void live_H_33(double *state, double *unused, double *out_888048865152090005);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}