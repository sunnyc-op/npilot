#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4550771329507489288);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9207675421062885601);
void car_H_mod_fun(double *state, double *out_1712302744524275838);
void car_f_fun(double *state, double dt, double *out_8383254522148583744);
void car_F_fun(double *state, double dt, double *out_4688408953840746502);
void car_h_25(double *state, double *unused, double *out_2365408889928126150);
void car_H_25(double *state, double *unused, double *out_2624847431539050735);
void car_h_24(double *state, double *unused, double *out_1455146287796013519);
void car_H_24(double *state, double *unused, double *out_933078167263272401);
void car_h_30(double *state, double *unused, double *out_2522595558279255295);
void car_H_30(double *state, double *unused, double *out_106514473031802108);
void car_h_26(double *state, double *unused, double *out_399766992907939301);
void car_H_26(double *state, double *unused, double *out_679678538221749866);
void car_h_27(double *state, double *unused, double *out_7327788887609419382);
void car_H_27(double *state, double *unused, double *out_2117079598152141109);
void car_h_29(double *state, double *unused, double *out_2479077709381590350);
void car_H_29(double *state, double *unused, double *out_403716871282590076);
void car_h_28(double *state, double *unused, double *out_6345704794274121270);
void car_H_28(double *state, double *unused, double *out_4678682145786940498);
void car_h_31(double *state, double *unused, double *out_4405426336422224786);
void car_H_31(double *state, double *unused, double *out_4451827818972766518);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}