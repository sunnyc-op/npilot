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
void car_err_fun(double *nom_x, double *delta_x, double *out_4877302666818620112);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1202084109018656834);
void car_H_mod_fun(double *state, double *out_359485901428299906);
void car_f_fun(double *state, double dt, double *out_1034882919924552313);
void car_F_fun(double *state, double dt, double *out_9047421667853216185);
void car_h_25(double *state, double *unused, double *out_1301771313225191319);
void car_H_25(double *state, double *unused, double *out_2116846282827690996);
void car_h_24(double *state, double *unused, double *out_1965321636609380214);
void car_H_24(double *state, double *unused, double *out_60368140779458977);
void car_h_30(double *state, double *unused, double *out_7665365291527007180);
void car_H_30(double *state, double *unused, double *out_401486675679557631);
void car_h_26(double *state, double *unused, double *out_8833109092603148917);
void car_H_26(double *state, double *unused, double *out_5858349601701747220);
void car_h_27(double *state, double *unused, double *out_5700991390567874460);
void car_H_27(double *state, double *unused, double *out_1773276636120867280);
void car_h_29(double *state, double *unused, double *out_5555471566347219489);
void car_H_29(double *state, double *unused, double *out_911718019993949815);
void car_h_28(double *state, double *unused, double *out_699318122907028815);
void car_H_28(double *state, double *unused, double *out_4170680997075580759);
void car_h_31(double *state, double *unused, double *out_1458957981576320464);
void car_H_31(double *state, double *unused, double *out_6484557703935098696);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}