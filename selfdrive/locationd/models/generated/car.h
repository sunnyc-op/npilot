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
void car_err_fun(double *nom_x, double *delta_x, double *out_3405411849062960531);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5631930522123645329);
void car_H_mod_fun(double *state, double *out_2249019235334351941);
void car_f_fun(double *state, double dt, double *out_1970153544911791805);
void car_F_fun(double *state, double dt, double *out_117042200783792997);
void car_h_25(double *state, double *unused, double *out_2527274791798040583);
void car_H_25(double *state, double *unused, double *out_4939095172763981300);
void car_h_24(double *state, double *unused, double *out_3993072292709402740);
void car_H_24(double *state, double *unused, double *out_2761880749156831327);
void car_h_30(double *state, double *unused, double *out_6164724320052062933);
void car_H_30(double *state, double *unused, double *out_2420762214256732673);
void car_h_26(double *state, double *unused, double *out_1963597352928310702);
void car_H_26(double *state, double *unused, double *out_8680598491638037524);
void car_h_27(double *state, double *unused, double *out_2081604746861687446);
void car_H_27(double *state, double *unused, double *out_4595525526057157584);
void car_h_29(double *state, double *unused, double *out_7056112537822163736);
void car_H_29(double *state, double *unused, double *out_1910530869942340489);
void car_h_28(double *state, double *unused, double *out_4112674769809816212);
void car_H_28(double *state, double *unused, double *out_6992929887011871063);
void car_h_31(double *state, double *unused, double *out_4440040372882991911);
void car_H_31(double *state, double *unused, double *out_9139937479838162616);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}