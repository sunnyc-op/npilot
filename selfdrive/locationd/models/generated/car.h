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
void car_err_fun(double *nom_x, double *delta_x, double *out_3243033182238193640);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2111442011747270149);
void car_H_mod_fun(double *state, double *out_8408061644982863180);
void car_f_fun(double *state, double dt, double *out_5002018357170392705);
void car_F_fun(double *state, double dt, double *out_123727185940828648);
void car_h_25(double *state, double *unused, double *out_4633127428766816095);
void car_H_25(double *state, double *unused, double *out_5849167184823695555);
void car_h_24(double *state, double *unused, double *out_4389809127990204203);
void car_H_24(double *state, double *unused, double *out_3676517585818195989);
void car_h_30(double *state, double *unused, double *out_2827431335482072230);
void car_H_30(double *state, double *unused, double *out_8367500143330944182);
void car_h_26(double *state, double *unused, double *out_6598769325787002944);
void car_H_26(double *state, double *unused, double *out_2107663865949639331);
void car_h_27(double *state, double *unused, double *out_1622697293192231159);
void car_H_27(double *state, double *unused, double *out_6192736831530519271);
void car_h_29(double *state, double *unused, double *out_2238986183236270493);
void car_H_29(double *state, double *unused, double *out_8877731487645336366);
void car_h_28(double *state, double *unused, double *out_2709423941548695486);
void car_H_28(double *state, double *unused, double *out_3795332470575805792);
void car_h_31(double *state, double *unused, double *out_5173739168224193637);
void car_H_31(double *state, double *unused, double *out_1481455763716287855);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}