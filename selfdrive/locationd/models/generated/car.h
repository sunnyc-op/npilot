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
void car_err_fun(double *nom_x, double *delta_x, double *out_6777767383690293166);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5397592667997122704);
void car_H_mod_fun(double *state, double *out_1600455802523382261);
void car_f_fun(double *state, double dt, double *out_5233723299563261311);
void car_F_fun(double *state, double dt, double *out_7948139696811054819);
void car_h_25(double *state, double *unused, double *out_2103105793419728366);
void car_H_25(double *state, double *unused, double *out_1472224050795332230);
void car_h_24(double *state, double *unused, double *out_6612164286682862269);
void car_H_24(double *state, double *unused, double *out_6341038915823039082);
void car_h_30(double *state, double *unused, double *out_2378299855704234255);
void car_H_30(double *state, double *unused, double *out_5999920380922940428);
void car_h_26(double *state, double *unused, double *out_2623373411956611390);
void car_H_26(double *state, double *unused, double *out_5213727369669388454);
void car_h_27(double *state, double *unused, double *out_4295935723994120780);
void car_H_27(double *state, double *unused, double *out_8174683692723365339);
void car_h_29(double *state, double *unused, double *out_658486195740098430);
void car_H_29(double *state, double *unused, double *out_5489689036608548244);
void car_h_28(double *state, double *unused, double *out_102016357287508232);
void car_H_28(double *state, double *unused, double *out_3526058765043221993);
void car_h_31(double *state, double *unused, double *out_6201409091699100230);
void car_H_31(double *state, double *unused, double *out_5839935471902739930);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}