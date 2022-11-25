#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_907718203128985335);
void live_err_fun(double *nom_x, double *delta_x, double *out_2437406129465532556);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2504976990150394946);
void live_H_mod_fun(double *state, double *out_2703325889272361258);
void live_f_fun(double *state, double dt, double *out_4440999693116400891);
void live_F_fun(double *state, double dt, double *out_463991190804664540);
void live_h_4(double *state, double *unused, double *out_6316878766118747446);
void live_H_4(double *state, double *unused, double *out_2974684771154449431);
void live_h_9(double *state, double *unused, double *out_630471256582841790);
void live_H_9(double *state, double *unused, double *out_2733495124524858786);
void live_h_10(double *state, double *unused, double *out_2123937926456274023);
void live_H_10(double *state, double *unused, double *out_8985010996966574143);
void live_h_12(double *state, double *unused, double *out_4281273698123610649);
void live_H_12(double *state, double *unused, double *out_2044771636877512364);
void live_h_31(double *state, double *unused, double *out_967733614636716488);
void live_H_31(double *state, double *unused, double *out_391977286218157945);
void live_h_32(double *state, double *unused, double *out_3527738957832712282);
void live_H_32(double *state, double *unused, double *out_7573992833620943666);
void live_h_13(double *state, double *unused, double *out_2230033776087315688);
void live_H_13(double *state, double *unused, double *out_8715528700733448605);
void live_h_14(double *state, double *unused, double *out_630471256582841790);
void live_H_14(double *state, double *unused, double *out_2733495124524858786);
void live_h_33(double *state, double *unused, double *out_6026981664250205993);
void live_H_33(double *state, double *unused, double *out_3542534290857015549);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}