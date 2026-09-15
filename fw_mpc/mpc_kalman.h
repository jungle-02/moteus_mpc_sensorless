#pragma once

#include <array>
#include <cstring>
#include <cstdint>
#include <cmath>

#include "fw_mpc/moteus_math.h"

// Bat toi uu hoa toc do cuc do, fast-math de dung phan cung FPU, ep unroll
#pragma GCC optimize ("O3", "fast-math", "unroll-loops")

namespace moteus {

class MPCKalmanEstimator {
public:
    static constexpr int Np = 5;  // Horizon du doan
    static constexpr int Nc = 3;  // Horizon dieu khien
    static constexpr float FREQ_MPC_HZ = 5000.0f; // Tan so goi MPC: 5kHz

    struct MotorParams {
        float poles = 14.0f;
        float inertia_kg_m2 = 2.94e-5f;
        float friction_Nms = 1e-4f;
        float flux_Wb = 0.0027f;
    };

    struct MPCConfig {
        float Qw = 15.0f;
        float Qth = 65.0f;
        float Ru = 0.001f;
        float Iq_max = 12.0f;

        int32_t max_iterations = 10;
        float tolerance = 1e-9f;

        float Tm_lpf_alpha = 0.01f;

        float max_err_w  = 2.0f;   // rad/s
        float max_err_th = 0.5f;   // rad

        // ========================================================
        // CO CAU CUNG
        // ========================================================
        float steady_state_torque_limit = 0.02f;
        float steady_state_enter_w_err = 3.0f;
        float steady_state_exit_w_err  = 10.0f;

        // ========================================================
        // THONG SO CHO VUNG HYBRID
        // ========================================================
        float transition_torque_limit = 0.01f;   // Gioi han khi tron pha
        float hybrid_error_squelch = 0.01f;      // Sai so gui vao QP
        float max_torque_rate = 30.0f;           // [Nm/s] Toc do Slew Rate co ban
    };

    struct KalmanConfig {
        float q_w = 0.01f;
        float q_theta = 0.01f;
        float q_torque = 10.0f;
        float r_w = 50.0f;
        float r_theta = 50.0f;
    };

    struct State {
        float omega_rad_s = 0.0f;
        float theta_rad = 0.0f;
        float torque_est_Nm = 0.0f;
    };

    struct Output {
        float iq_ref = 0.0f;
        float id_ref = 0.0f;
        float torque_cmd_Nm = 0.0f;
        State state;
    };

    MPCKalmanEstimator() {
        for(int i=0; i<3; i++) x_hat_[i] = 0.0f;
        for(int i=0; i<3; i++) {
            P_diag_[i] = 0.1f;
            P_offdiag_[i] = 0.0f;
        }
        for(int i=0; i<6; i++) lambda_[i] = 0.0f;

        Te_prev_ = 0.0f;
        Tm_hat_filt_ = 0.0f;

        mean_w_err_ = 100.0f;
        is_steady_state_ = false;

        cached_out_.iq_ref = 0.0f;
        cached_out_.id_ref = 0.0f;
        cached_out_.torque_cmd_Nm = 0.0f;
        cached_out_.state.omega_rad_s = 0.0f;
        cached_out_.state.theta_rad = 0.0f;
        cached_out_.state.torque_est_Nm = 0.0f;
    }

    void Init(float Ts_pwm) {
        Ts_pwm_ = Ts_pwm;

        float mpc_period = 1.0f / FREQ_MPC_HZ;
        decimation_factor_ = (int)(mpc_period / Ts_pwm_);
        if (decimation_factor_ < 1) decimation_factor_ = 1;

        tick_counter_ = 0;
        is_first_run_ = true;

        float p = motor_.poles / 2.0f;
        if (p < 1.0f) p = 7.0f;
        float flux = motor_.flux_Wb;
        if (flux < 1e-4f) flux = 0.0185f;
        Te_max_ = 1.5f * p * flux * mpc_config_.Iq_max;

        is_initialized_ = true;
    }

    __attribute__((always_inline))
    Output Update(float omega_meas, float theta_meas, float omega_ref, float theta_ref, float max_torque_cmd, bool is_velocity_only = false, bool is_hybrid_transition = false) MOTEUS_CCM_ATTRIBUTE {
        if (!is_initialized_) {
            return cached_out_;
        }

        tick_counter_++;

        if (tick_counter_ >= decimation_factor_ || is_first_run_) {
            tick_counter_ = 0;
            is_first_run_ = false;

            omega_meas = sanitize(omega_meas);
            theta_meas = sanitize(theta_meas);
            omega_ref = sanitize(omega_ref);
            theta_ref = sanitize(theta_ref);

            float current_max_Te = max_torque_cmd;
            if (current_max_Te > Te_max_) current_max_Te = Te_max_;

            // TRUYEN is_hybrid_transition VAO KALMAN DE KHOA NHIEU
            KalmanFilterStep(omega_meas, theta_meas, is_hybrid_transition);

            RunMPC(omega_ref, theta_ref, current_max_Te, is_velocity_only, is_hybrid_transition);

            cached_out_.state.omega_rad_s = x_hat_[0];
            cached_out_.state.theta_rad = x_hat_[1];
            cached_out_.state.torque_est_Nm = Tm_hat_filt_;

            cached_out_.torque_cmd_Nm = sanitize(cached_out_.torque_cmd_Nm);
            cached_out_.iq_ref = sanitize(cached_out_.iq_ref);
            cached_out_.id_ref = 0.0f;
        }

        return cached_out_;
    }

private:
    volatile bool is_initialized_ = false;
    float Ts_pwm_ = 0.0f;
    int decimation_factor_ = 1;
    int tick_counter_ = 0;
    bool is_first_run_ = true;

    MotorParams motor_;
    MPCConfig mpc_config_;
    KalmanConfig kf_config_;
    Output cached_out_;

    float Te_max_ = 0.0f;
    float Te_prev_ = 0.0f;
    float Tm_hat_filt_ = 0.0f;

    float mean_w_err_ = 100.0f;
    bool is_steady_state_ = false;

    std::array<float, 3> x_hat_;
    std::array<float, 3> P_diag_;
    std::array<float, 3> P_offdiag_;
    std::array<float, 6> lambda_;

    static constexpr float Akf_[9] = {0.999320f, 0.0f, -6.802721f, 0.0002f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    static constexpr float Bkf_[3] = {6.802721f, 0.0f, 0.0f};

    static constexpr float Qkf_[9] = {0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f, 0.0f, 0.0f, 0.001f};
    static constexpr float Rkf_[4] = {50.0f, 0.0f, 0.0f, 50.0f};

    static constexpr float Mx_[3][3] = {
        {166.452f, 10.512f, -54.731f},
        {102.311f,  6.221f, -28.514f},
        { 45.672f,  2.784f, - 9.182f}
    };

    static constexpr float M_Hinv_[6][3] = {
        { 0.1251f, -0.0521f,  0.0182f},
        { 0.1124f,  0.0812f, -0.0341f},
        { 0.0982f,  0.0651f,  0.0911f},
        {-0.1251f,  0.0521f, -0.0182f},
        {-0.1124f, -0.0812f,  0.0341f},
        {-0.0982f, -0.0651f, -0.0911f}
    };

    static constexpr float H_inv_[3][3] = {
        { 0.0211f, -0.0082f,  0.0012f},
        {-0.0082f,  0.0315f, -0.0051f},
        { 0.0012f, -0.0051f,  0.0421f}
    };

    static constexpr float W_[6][6] = {
        { 0.0125f,  0.0082f,  0.0041f, -0.0125f, -0.0082f, -0.0041f},
        { 0.0082f,  0.0214f,  0.0112f, -0.0082f, -0.0214f, -0.0112f},
        { 0.0041f,  0.0112f,  0.0318f, -0.0041f, -0.0112f, -0.0318f},
        {-0.0125f, -0.0082f, -0.0041f,  0.0125f,  0.0082f,  0.0041f},
        {-0.0082f, -0.0214f, -0.0112f,  0.0082f,  0.0214f,  0.0112f},
        {-0.0041f, -0.0112f, -0.0318f,  0.0041f,  0.0112f,  0.0318f}
    };

    static constexpr float W_inv_diag_[6] = {
        1.0f/0.0125f, 1.0f/0.0214f, 1.0f/0.0318f, 1.0f/0.0125f, 1.0f/0.0214f, 1.0f/0.0318f
    };

    inline float sanitize(float val) __attribute__((always_inline)) {
        union { float f; uint32_t i; } u;
        u.f = val;
        uint32_t exp = (u.i & 0x7F800000) >> 23;
        if (exp == 0xFF || exp == 0) return 0.0f;
        return u.f;
    }

    __attribute__((always_inline))
    void RunMPC(float omega_ref, float theta_ref, float current_max_Te, bool is_velocity_only, bool is_hybrid_transition) MOTEUS_CCM_ATTRIBUTE {
        Tm_hat_filt_ += mpc_config_.Tm_lpf_alpha * (x_hat_[2] - Tm_hat_filt_);

        if (Tm_hat_filt_ > Te_max_) Tm_hat_filt_ = Te_max_;
        if (Tm_hat_filt_ < -Te_max_) Tm_hat_filt_ = -Te_max_;

        float omega_err = x_hat_[0] - omega_ref;
        float theta_err = x_hat_[1] - theta_ref;

        // ======================================================================
        // QUEN DU LIEU KHI HYBRID
        // ======================================================================
        if (is_hybrid_transition) {
            // Squelch manh hon (chi giu 1% error)
            omega_err *= mpc_config_.hybrid_error_squelch;
            theta_err *= mpc_config_.hybrid_error_squelch;

            // Loai bo hoan toan feedforward bi sai do nhieu
            Tm_hat_filt_ *= 0.90f;
        }

        // Loc Slew Rate de loai bo nhieu Hybrid ra khoi he thong ra quyet dinh Hysteresis
        float filter_input = omega_err;
        if (filter_input > 15.0f) filter_input = 15.0f;
        if (filter_input < -15.0f) filter_input = -15.0f;

        mean_w_err_ += 0.005f * (filter_input - mean_w_err_);
        float abs_mean_err_w = __builtin_fabsf(mean_w_err_);

        if (!is_steady_state_) {
            if (abs_mean_err_w < mpc_config_.steady_state_enter_w_err) {
                is_steady_state_ = true;
            }
        } else {
            if (abs_mean_err_w > mpc_config_.steady_state_exit_w_err) {
                is_steady_state_ = false;
            }
        }

        float dynamic_max_Te = current_max_Te;

        if (is_steady_state_) {
            dynamic_max_Te = mpc_config_.steady_state_torque_limit;
        }

        if (is_hybrid_transition) {
            dynamic_max_Te = mpc_config_.transition_torque_limit;
        }

        // Phan tach Mode
        if (is_velocity_only) {
            theta_err = 0.0f;
        } else {
            if (theta_err > mpc_config_.max_err_th) theta_err = mpc_config_.max_err_th;
            if (theta_err < -mpc_config_.max_err_th) theta_err = -mpc_config_.max_err_th;
        }

        if (omega_err > mpc_config_.max_err_w) omega_err = mpc_config_.max_err_w;
        if (omega_err < -mpc_config_.max_err_w) omega_err = -mpc_config_.max_err_w;

        float Te_clip = Te_prev_;
        if (Te_clip > dynamic_max_Te) Te_clip = dynamic_max_Te;
        if (Te_clip < -dynamic_max_Te) Te_clip = -dynamic_max_Te;

        // u_curr chinh la ham chua Feedforward tai trong
        float u_curr = Te_clip - Tm_hat_filt_;

        float f0 = Mx_[0][0]*omega_err + Mx_[0][1]*theta_err + Mx_[0][2]*u_curr;
        float f1 = Mx_[1][0]*omega_err + Mx_[1][1]*theta_err + Mx_[1][2]*u_curr;
        float f2 = Mx_[2][0]*omega_err + Mx_[2][1]*theta_err + Mx_[2][2]*u_curr;

        float n_upper = dynamic_max_Te - Te_clip;
        float n_lower = dynamic_max_Te + Te_clip;

        float k0 = n_upper + M_Hinv_[0][0]*f0 + M_Hinv_[0][1]*f1 + M_Hinv_[0][2]*f2;
        float k1 = n_upper + M_Hinv_[1][0]*f0 + M_Hinv_[1][1]*f1 + M_Hinv_[1][2]*f2;
        float k2 = n_upper + M_Hinv_[2][0]*f0 + M_Hinv_[2][1]*f1 + M_Hinv_[2][2]*f2;
        float k3 = n_lower + M_Hinv_[3][0]*f0 + M_Hinv_[3][1]*f1 + M_Hinv_[3][2]*f2;
        float k4 = n_lower + M_Hinv_[4][0]*f0 + M_Hinv_[4][1]*f1 + M_Hinv_[4][2]*f2;
        float k5 = n_lower + M_Hinv_[5][0]*f0 + M_Hinv_[5][1]*f1 + M_Hinv_[5][2]*f2;

        // Giai QP
        for(int it=0; it < mpc_config_.max_iterations; it++) {
            float max_diff = 0.0f;
            float sum, wi, new_lambda, diff;

            sum = W_[0][1]*lambda_[1] + W_[0][2]*lambda_[2] + W_[0][3]*lambda_[3] + W_[0][4]*lambda_[4] + W_[0][5]*lambda_[5];
            wi = (-k0 - sum) * W_inv_diag_[0];
            new_lambda = (wi > 0.0f) ? wi : 0.0f;
            diff = __builtin_fabsf(new_lambda - lambda_[0]);
            if(diff > max_diff) max_diff = diff;
            lambda_[0] = new_lambda;

            sum = W_[1][0]*lambda_[0] + W_[1][2]*lambda_[2] + W_[1][3]*lambda_[3] + W_[1][4]*lambda_[4] + W_[1][5]*lambda_[5];
            wi = (-k1 - sum) * W_inv_diag_[1];
            new_lambda = (wi > 0.0f) ? wi : 0.0f;
            diff = __builtin_fabsf(new_lambda - lambda_[1]);
            if(diff > max_diff) max_diff = diff;
            lambda_[1] = new_lambda;

            sum = W_[2][0]*lambda_[0] + W_[2][1]*lambda_[1] + W_[2][3]*lambda_[3] + W_[2][4]*lambda_[4] + W_[2][5]*lambda_[5];
            wi = (-k2 - sum) * W_inv_diag_[2];
            new_lambda = (wi > 0.0f) ? wi : 0.0f;
            diff = __builtin_fabsf(new_lambda - lambda_[2]);
            if(diff > max_diff) max_diff = diff;
            lambda_[2] = new_lambda;

            sum = W_[3][0]*lambda_[0] + W_[3][1]*lambda_[1] + W_[3][2]*lambda_[2] + W_[3][4]*lambda_[4] + W_[3][5]*lambda_[5];
            wi = (-k3 - sum) * W_inv_diag_[3];
            new_lambda = (wi > 0.0f) ? wi : 0.0f;
            diff = __builtin_fabsf(new_lambda - lambda_[3]);
            if(diff > max_diff) max_diff = diff;
            lambda_[3] = new_lambda;

            sum = W_[4][0]*lambda_[0] + W_[4][1]*lambda_[1] + W_[4][2]*lambda_[2] + W_[4][3]*lambda_[3] + W_[4][5]*lambda_[5];
            wi = (-k4 - sum) * W_inv_diag_[4];
            new_lambda = (wi > 0.0f) ? wi : 0.0f;
            diff = __builtin_fabsf(new_lambda - lambda_[4]);
            if(diff > max_diff) max_diff = diff;
            lambda_[4] = new_lambda;

            sum = W_[5][0]*lambda_[0] + W_[5][1]*lambda_[1] + W_[5][2]*lambda_[2] + W_[5][3]*lambda_[3] + W_[5][4]*lambda_[4];
            wi = (-k5 - sum) * W_inv_diag_[5];
            new_lambda = (wi > 0.0f) ? wi : 0.0f;
            diff = __builtin_fabsf(new_lambda - lambda_[5]);
            if(diff > max_diff) max_diff = diff;
            lambda_[5] = new_lambda;

            if(max_diff < mpc_config_.tolerance) break;
        }

        float MT_lam0 = lambda_[0] + lambda_[1] + lambda_[2] - lambda_[3] - lambda_[4] - lambda_[5];
        float MT_lam1 = lambda_[1] + lambda_[2] - lambda_[4] - lambda_[5];
        float MT_lam2 = lambda_[2] - lambda_[5];

        float deltaU = -H_inv_[0][0]*(f0 + MT_lam0) - H_inv_[0][1]*(f1 + MT_lam1) - H_inv_[0][2]*(f2 + MT_lam2);

        float Te_next = Te_clip + deltaU;

        // ======================================================================
        // DYNAMIC SLEW RATE (KHOA CONG BANG VAT LY)
        // ======================================================================
        // Neu o vung binh thuong: cho phep tang max 30 Nm/s
        // Neu o vung Hybrid: Siet gat co hong, chi cho phep tang tu tu 1.0 Nm/s
        float current_rate = is_hybrid_transition ? 1.0f : mpc_config_.max_torque_rate;
        float max_delta_Te = current_rate / FREQ_MPC_HZ;

        float delta_Te = Te_next - Te_prev_;

        if (delta_Te > max_delta_Te) {
            Te_next = Te_prev_ + max_delta_Te;
        } else if (delta_Te < -max_delta_Te) {
            Te_next = Te_prev_ - max_delta_Te;
        }
        // ======================================================================

        if (Te_next > dynamic_max_Te) Te_next = dynamic_max_Te;
        if (Te_next < -dynamic_max_Te) Te_next = -dynamic_max_Te;

        Te_prev_ = sanitize(Te_next);

        float p = motor_.poles / 2.0f;
        if (p < 1.0f) p = 7.0f;
        float flux = motor_.flux_Wb;
        if (flux < 1e-4f) flux = 0.0185f;

        float iq_raw = (2.0f / (3.0f * p * flux)) * Te_next;
        if (iq_raw > mpc_config_.Iq_max) iq_raw = mpc_config_.Iq_max;
        if (iq_raw < -mpc_config_.Iq_max) iq_raw = -mpc_config_.Iq_max;

        cached_out_.torque_cmd_Nm = Te_next;
        cached_out_.iq_ref = sanitize(iq_raw);
        cached_out_.id_ref = 0.0f;
    }

    __attribute__((always_inline))
    void KalmanFilterStep(float w_meas, float th_meas, bool is_hybrid_transition) MOTEUS_CCM_ATTRIBUTE {
        float x0_p = Akf_[0]*x_hat_[0] + Akf_[2]*x_hat_[2] + Bkf_[0]*Te_prev_;
        float x1_p = Akf_[3]*x_hat_[0] + x_hat_[1]; // Akf_[4] = 1.0
        float x2_p = x_hat_[2]; // Akf_[8] = 1.0

        float P00 = P_diag_[0];
        float P11 = P_diag_[1];
        float P22 = P_diag_[2];
        float P01 = P_offdiag_[0];
        float P02 = P_offdiag_[1];
        float P12 = P_offdiag_[2];

        float A00 = Akf_[0];
        float A02 = Akf_[2];
        float A10 = Akf_[3];

        float AP00 = A00*P00 + A02*P02;
        float AP01 = A00*P01 + A02*P12;
        float AP02 = A00*P02 + A02*P22;
        float AP10 = A10*P00 + P01;
        float AP11 = A10*P01 + P11;
        float AP12 = A10*P02 + P12;

        float P0_new = AP00*A00 + AP02*A02 + Qkf_[0];
        float P1_new = AP00*A10 + AP01;
        float P2_new = AP02;
        float P3_new = AP10*A00 + AP12*A02;
        float P4_new = AP10*A10 + AP11 + Qkf_[4];
        float P5_new = AP12;
        float P6_new = P02*A00 + P22*A02;
        float P7_new = P02*A10 + P12;
        float P8_new = P22 + Qkf_[8];

        float S00 = P0_new + Rkf_[0];
        float S01 = P1_new;
        float S11 = P4_new + Rkf_[3];

        float det = S00*S11 - S01*S01;
        if (det > -1e-12f && det < 1e-12f) {
            det = (det < 0.0f) ? -1e-12f : 1e-12f;
        }
        float idet = 1.0f / det;

        float Si00 = S11 * idet;
        float Si01 = -S01 * idet;
        float Si11 = S00 * idet;

        float K0 = P0_new*Si00 + P1_new*Si01;
        float K1 = P0_new*Si01 + P1_new*Si11;
        float K2 = P3_new*Si00 + P4_new*Si01;
        float K3 = P3_new*Si01 + P4_new*Si11;
        float K4 = P6_new*Si00 + P7_new*Si01;
        float K5 = P6_new*Si01 + P7_new*Si11;

        float y_err0 = w_meas - x0_p;
        float y_err1 = th_meas - x1_p;

        x_hat_[0] = sanitize(x0_p + K0*y_err0 + K1*y_err1);
        x_hat_[1] = sanitize(x1_p + K2*y_err0 + K3*y_err1);

        // ======================================================================
        // LOC DU LIEU RAC
        // ======================================================================
        float delta_Tm = K4*y_err0 + K5*y_err1;
        float max_delta_Tm = 0.05f;

        if (is_hybrid_transition) {
            // Khi sensorless nhay pha (zero crossing), w_meas & th_meas hoan toan la rac.
            // Bat buoc bo qua luong cap nhat tai trong o chu ky nay.
            delta_Tm = 0.0f;

            // Xa tu tu nang luong uoc luong dang ton tai trong bien trang thai x_hat_[2]
            x2_p *= 0.99f;
        } else {
            if (delta_Tm > max_delta_Tm) delta_Tm = max_delta_Tm;
            if (delta_Tm < -max_delta_Tm) delta_Tm = -max_delta_Tm;
        }

        x_hat_[2] = sanitize(x2_p + delta_Tm);

        // Khoa an toan chong tran so
        if (x_hat_[2] > Te_max_) x_hat_[2] = Te_max_;
        if (x_hat_[2] < -Te_max_) x_hat_[2] = -Te_max_;

        float IK0 = 1.0f - K0;
        float IK1 = -K1;
        float IK2 = -K2;
        float IK3 = 1.0f - K3;
        float IK4 = -K4;
        float IK5 = -K5;

        float P0_up = IK0*P0_new + IK1*P3_new;
        float P1_up = IK0*P1_new + IK1*P4_new;
        float P2_up = IK0*P2_new + IK1*P5_new;
        float P3_up = IK2*P0_new + IK3*P3_new;
        float P4_up = IK2*P1_new + IK3*P4_new;
        float P5_up = IK2*P2_new + IK3*P5_new;
        float P6_up = IK4*P0_new + IK5*P3_new + P6_new;
        float P7_up = IK4*P1_new + IK5*P4_new + P7_new;
        float P8_up = IK4*P2_new + IK5*P5_new + P8_new;

        P_diag_[0] = (P0_up > 0.0f) ? P0_up : 0.0f;
        P_diag_[1] = (P4_up > 0.0f) ? P4_up : 0.0f;
        P_diag_[2] = (P8_up > 0.0f) ? P8_up : 0.0f;
        P_offdiag_[0] = (P1_up + P3_up) * 0.5f;
        P_offdiag_[1] = (P2_up + P6_up) * 0.5f;
        P_offdiag_[2] = (P5_up + P7_up) * 0.5f;
    }
};

} // namespace moteus
