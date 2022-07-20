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
void car_err_fun(double *nom_x, double *delta_x, double *out_1587320949155634031);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7610236178695361162);
void car_H_mod_fun(double *state, double *out_2153325158078460112);
void car_f_fun(double *state, double dt, double *out_1127301623673412374);
void car_F_fun(double *state, double dt, double *out_4211123534076608715);
void car_h_25(double *state, double *unused, double *out_5804680800098537329);
void car_H_25(double *state, double *unused, double *out_8042566109691692167);
void car_h_24(double *state, double *unused, double *out_5424263938900647146);
void car_H_24(double *state, double *unused, double *out_6916074274185325760);
void car_h_30(double *state, double *unused, double *out_6054235347134719124);
void car_H_30(double *state, double *unused, double *out_5876481633890251251);
void car_h_26(double *state, double *unused, double *out_7166601510739507545);
void car_H_26(double *state, double *unused, double *out_6662674645143803225);
void car_h_27(double *state, double *unused, double *out_1198081903694528450);
void car_H_27(double *state, double *unused, double *out_3701718322089826340);
void car_h_29(double *state, double *unused, double *out_9091021398579051809);
void car_H_29(double *state, double *unused, double *out_6386712978204643435);
void car_h_28(double *state, double *unused, double *out_2141591756596190885);
void car_H_28(double *state, double *unused, double *out_1304313961135112861);
void car_h_31(double *state, double *unused, double *out_5438847316625028317);
void car_H_31(double *state, double *unused, double *out_6036466542910451749);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}