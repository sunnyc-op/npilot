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
void car_err_fun(double *nom_x, double *delta_x, double *out_9076953633350933155);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6493249169715700303);
void car_H_mod_fun(double *state, double *out_6053903725589148651);
void car_f_fun(double *state, double dt, double *out_7510396694923395417);
void car_F_fun(double *state, double dt, double *out_4314618042520952482);
void car_h_25(double *state, double *unused, double *out_7579285510236742280);
void car_H_25(double *state, double *unused, double *out_5043007725208902189);
void car_h_24(double *state, double *unused, double *out_6868631688394465622);
void car_H_24(double *state, double *unused, double *out_1485082126406579053);
void car_h_30(double *state, double *unused, double *out_9029916902865625447);
void car_H_30(double *state, double *unused, double *out_1873682616282714566);
void car_h_26(double *state, double *unused, double *out_8086407049963963012);
void car_H_26(double *state, double *unused, double *out_8784511044082958413);
void car_h_27(double *state, double *unused, double *out_4173763459425434773);
void car_H_27(double *state, double *unused, double *out_7347109984152567170);
void car_h_29(double *state, double *unused, double *out_4028243635204779802);
void car_H_29(double *state, double *unused, double *out_9060472711022118203);
void car_h_28(double *state, double *unused, double *out_1052143756409890174);
void car_H_28(double *state, double *unused, double *out_7096842439456791952);
void car_h_31(double *state, double *unused, double *out_2986185912718760151);
void car_H_31(double *state, double *unused, double *out_5012361763331941761);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}