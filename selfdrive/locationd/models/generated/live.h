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
void live_H(double *in_vec, double *out_6784604416272540050);
void live_err_fun(double *nom_x, double *delta_x, double *out_2655675818042385017);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2963903028631488861);
void live_H_mod_fun(double *state, double *out_6140507083378850689);
void live_f_fun(double *state, double dt, double *out_8342962912891581995);
void live_F_fun(double *state, double dt, double *out_1526442805071530398);
void live_h_4(double *state, double *unused, double *out_2670771569647076777);
void live_H_4(double *state, double *unused, double *out_4218719645024462601);
void live_h_9(double *state, double *unused, double *out_6675251998404262248);
void live_H_9(double *state, double *unused, double *out_3977529998394871956);
void live_h_10(double *state, double *unused, double *out_1276204232902478899);
void live_H_10(double *state, double *unused, double *out_6757950970174612806);
void live_h_12(double *state, double *unused, double *out_2978561226524827777);
void live_H_12(double *state, double *unused, double *out_800736763007499194);
void live_h_31(double *state, double *unused, double *out_2413691193150815936);
void live_H_31(double *state, double *unused, double *out_3546299795332512903);
void live_h_32(double *state, double *unused, double *out_3243039148850874459);
void live_H_32(double *state, double *unused, double *out_4520284407358662164);
void live_h_13(double *state, double *unused, double *out_4786736807995769626);
void live_H_13(double *state, double *unused, double *out_6288316727689772411);
void live_h_14(double *state, double *unused, double *out_6675251998404262248);
void live_H_14(double *state, double *unused, double *out_3977529998394871956);
void live_h_33(double *state, double *unused, double *out_7896467299687702405);
void live_H_33(double *state, double *unused, double *out_6696856799971370507);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}