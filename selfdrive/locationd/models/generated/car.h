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
void car_err_fun(double *nom_x, double *delta_x, double *out_4989488229479570161);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4379738640735284012);
void car_H_mod_fun(double *state, double *out_2445474491913214453);
void car_f_fun(double *state, double dt, double *out_6392909334638572965);
void car_F_fun(double *state, double dt, double *out_5263796904319004870);
void car_h_25(double *state, double *unused, double *out_5333733597242535689);
void car_H_25(double *state, double *unused, double *out_1984779620623110074);
void car_h_24(double *state, double *unused, double *out_7261268887423112988);
void car_H_24(double *state, double *unused, double *out_5542705219425433210);
void car_h_30(double *state, double *unused, double *out_1254094329233759474);
void car_H_30(double *state, double *unused, double *out_2542916709504498124);
void car_h_26(double *state, double *unused, double *out_3858499650042457125);
void car_H_26(double *state, double *unused, double *out_1756723698250946150);
void car_h_27(double *state, double *unused, double *out_7771143240580985364);
void car_H_27(double *state, double *unused, double *out_319322638320554907);
void car_h_29(double *state, double *unused, double *out_8190268924592470331);
void car_H_29(double *state, double *unused, double *out_2032685365190105940);
void car_h_28(double *state, double *unused, double *out_5673927565467720607);
void car_H_28(double *state, double *unused, double *out_7115084382259636514);
void car_h_31(double *state, double *unused, double *out_8330126522076446849);
void car_H_31(double *state, double *unused, double *out_2382931800484297626);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}