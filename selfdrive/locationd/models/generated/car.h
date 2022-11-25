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
void car_err_fun(double *nom_x, double *delta_x, double *out_6118062787728240548);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9211263313597478507);
void car_H_mod_fun(double *state, double *out_4561024202480585870);
void car_f_fun(double *state, double dt, double *out_6415819914950947790);
void car_F_fun(double *state, double dt, double *out_5710333805771084184);
void car_h_25(double *state, double *unused, double *out_5580444758777246286);
void car_H_25(double *state, double *unused, double *out_5747614728045202509);
void car_h_24(double *state, double *unused, double *out_5802005886931275559);
void car_H_24(double *state, double *unused, double *out_2189689129242879373);
void car_h_30(double *state, double *unused, double *out_4642539516321470839);
void car_H_30(double *state, double *unused, double *out_3229281769537953882);
void car_h_26(double *state, double *unused, double *out_8699869409390652045);
void car_H_26(double *state, double *unused, double *out_2443088758284401908);
void car_h_27(double *state, double *unused, double *out_2031696063376884196);
void car_H_27(double *state, double *unused, double *out_5404045081338378793);
void car_h_29(double *state, double *unused, double *out_1115445397672267117);
void car_H_29(double *state, double *unused, double *out_2719050425223561698);
void car_h_28(double *state, double *unused, double *out_3199439864453025933);
void car_H_28(double *state, double *unused, double *out_7801449442293092272);
void car_h_31(double *state, double *unused, double *out_6876182662686600696);
void car_H_31(double *state, double *unused, double *out_3069296860517753384);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}