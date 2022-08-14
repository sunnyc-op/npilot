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
void car_err_fun(double *nom_x, double *delta_x, double *out_1780734620249590456);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4094607078236144920);
void car_H_mod_fun(double *state, double *out_3009298313997658244);
void car_f_fun(double *state, double dt, double *out_7642406061189318393);
void car_F_fun(double *state, double dt, double *out_8250113623205205041);
void car_h_25(double *state, double *unused, double *out_5183177480751370237);
void car_H_25(double *state, double *unused, double *out_4451369567180334730);
void car_h_24(double *state, double *unused, double *out_5010304262199017215);
void car_H_24(double *state, double *unused, double *out_469046415782566832);
void car_h_30(double *state, double *unused, double *out_4766905911694441387);
void car_H_30(double *state, double *unused, double *out_6969702525687583357);
void car_h_26(double *state, double *unused, double *out_875823959386515448);
void car_H_26(double *state, double *unused, double *out_709866248306278506);
void car_h_27(double *state, double *unused, double *out_4425327335590551427);
void car_H_27(double *state, double *unused, double *out_4794939213887158446);
void car_h_29(double *state, double *unused, double *out_8678164894354264584);
void car_H_29(double *state, double *unused, double *out_7479933870001975541);
void car_h_28(double *state, double *unused, double *out_8679549502232969626);
void car_H_28(double *state, double *unused, double *out_2397534852932444967);
void car_h_31(double *state, double *unused, double *out_5382293942204132194);
void car_H_31(double *state, double *unused, double *out_83658146072927030);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}