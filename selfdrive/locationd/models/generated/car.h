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
void car_err_fun(double *nom_x, double *delta_x, double *out_7307813249485127470);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3794738096767726126);
void car_H_mod_fun(double *state, double *out_5274657662750201021);
void car_f_fun(double *state, double dt, double *out_3406949635768903926);
void car_F_fun(double *state, double dt, double *out_8623400185762976873);
void car_h_25(double *state, double *unused, double *out_3690229574354332810);
void car_H_25(double *state, double *unused, double *out_8370508047571984746);
void car_h_24(double *state, double *unused, double *out_8276081211538433864);
void car_H_24(double *state, double *unused, double *out_3500664219546048769);
void car_h_30(double *state, double *unused, double *out_4191043076399716375);
void car_H_30(double *state, double *unused, double *out_3159545684645950115);
void car_h_26(double *state, double *unused, double *out_8770616965969002786);
void car_H_26(double *state, double *unused, double *out_4629004728697928522);
void car_h_27(double *state, double *unused, double *out_8652609572035626042);
void car_H_27(double *state, double *unused, double *out_5334308996446375026);
void car_h_29(double *state, double *unused, double *out_1154257024955383690);
void car_H_29(double *state, double *unused, double *out_7047671723315926059);
void car_h_28(double *state, double *unused, double *out_367933840404850400);
void car_H_28(double *state, double *unused, double *out_6316673333324094983);
void car_h_31(double *state, double *unused, double *out_4073035682466339631);
void car_H_31(double *state, double *unused, double *out_8401154009448945174);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}