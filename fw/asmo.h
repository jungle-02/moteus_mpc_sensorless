#pragma once

#include "fw/moteus_math.h"
#include <algorithm>
#include <cmath>

namespace moteus {

// Kep gia tri (Saturation)
inline float Clamp(float val, float min_val, float max_val) {
  if (val > max_val)
    return max_val;
  if (val < min_val)
    return min_val;
  return val;
}
// Ham Sigmoid xap xi tanh
inline float FastSigmoid(float x) { return Clamp(x, -1.0f, 1.0f); }

class AsmoObserver {
public:
  struct Config {
    // Thong so dong co (cap nhat tu bldc_servo.cc)
    float Rs = 0.0f;
    float Ls = 0.0f;
    float dt = 0.0f;

    // --- TUNING CHO PHAN CUNG THUC TE ---
    float K0 = 50.0f;
    float tau = 0.01f;
    float epsilon = 0.5f;
    float gamma = 10.0f;
    float max_k_derivative = 2000.0f; // Gioi han toc do tang gain

    float chi = 20.0f;
    float a = 5.0f;       // Do doc ham sigmoid
    float l_gain = 50.0f; // Gain quan sat BEMF

    int mode = 1;
  };

  struct State {
    float i_est_alpha = 0.0f, i_est_beta = 0.0f;
    float int_e_alpha = 0.0f, int_e_beta = 0.0f;
    float e_est_alpha = 0.0f, e_est_beta = 0.0f;
    float k_alpha = 0.1f, k_beta = 0.1f; // Khoi tao nho, tranh 0
    float phi_alpha = 0.0f, phi_beta = 0.0f;
    float K1_alpha = 1.0f, K1_beta = 1.0f;
    float we_hat = 0.0f, theta_hat = 0.0f;
  };

  void Update(const Config &c, float u_alpha, float u_beta, float i_alpha_meas,
              float i_beta_meas, float ref_speed_A) {

    // 1. Kiem tra NaN dau vao ngay lap tuc
    if (std::isnan(i_alpha_meas) || std::isnan(i_beta_meas))
      return;

    // 2.  Neu chua co thong so dong co (chia cho 0), thoat ngay
    if (c.Rs <= 0.0f || c.Ls <= 0.0f || c.dt <= 0.0f)
      return;

    float A = std::max(std::abs(ref_speed_A), 1.0f);

    // Tinh sai so dong dien
    float i_err_alpha = state_.i_est_alpha - i_alpha_meas;
    float i_err_beta = state_.i_est_beta - i_beta_meas;

    // Kep sai so dong dien (quan trong khi khoi dong)
    i_err_alpha = Clamp(i_err_alpha, -50.0f, 50.0f);
    i_err_beta = Clamp(i_err_beta, -50.0f, 50.0f);

    float term_RL = (c.chi * c.Ls - c.Rs);
    float e_err_alpha = term_RL * i_err_alpha;
    float e_err_beta = term_RL * i_err_beta;

    // Tich phan sai so
    state_.int_e_alpha += i_err_alpha * c.dt;
    state_.int_e_beta += i_err_beta * c.dt;

    // Anti-windup cho tich phan (BAT BUOC TREN PHAN CUNG)
    state_.int_e_alpha = Clamp(state_.int_e_alpha, -20.0f, 20.0f);
    state_.int_e_beta = Clamp(state_.int_e_beta, -20.0f, 20.0f);

    // Mat truot S
    float S_alpha = i_err_alpha + c.chi * state_.int_e_alpha;
    float S_beta = i_err_beta + c.chi * state_.int_e_beta;

    // Thay the tanh bang FastSigmoid (Saturation)
    float H_alpha = FastSigmoid(c.a * S_alpha);
    float H_beta = FastSigmoid(c.a * S_beta);

    // Adaptive Gain (co loc LPF thong qua tau)
    state_.phi_alpha += ((H_alpha - state_.phi_alpha) / c.tau) * c.dt;
    state_.phi_beta += ((H_beta - state_.phi_beta) / c.tau) * c.dt;

    // Ham cap nhat Gain thich nghi
    auto update_k = [&](float S, float &k, float &K1, float phi) {
      float dk = 0.0f;
      if (std::abs(S) > c.epsilon) {
        if (k <= 2000.0f) // Gioi han tran cho Gain
          dk = c.K0 * std::abs(S);
        K1 += dk * c.dt;
      } else {
        float target = K1 * std::sqrt(std::abs(phi));
        float raw_dk = c.gamma * (target - k);
        dk = Clamp(raw_dk, -c.max_k_derivative, c.max_k_derivative);
      }
      k += dk * c.dt;
      k = Clamp(k, 0.1f, 3000.0f); // Luon duong va khong qua lon
    };

    update_k(S_alpha, state_.k_alpha, state_.K1_alpha, state_.phi_alpha);
    update_k(S_beta, state_.k_beta, state_.K1_beta, state_.phi_beta);

    // Cap nhat dong dien uoc luong
    float di_est_alpha = (-c.Rs * state_.i_est_alpha + u_alpha -
                          state_.e_est_alpha - state_.k_alpha * H_alpha) /
                         c.Ls;
    float di_est_beta = (-c.Rs * state_.i_est_beta + u_beta -
                         state_.e_est_beta - state_.k_beta * H_beta) /
                        c.Ls;

    state_.i_est_alpha += di_est_alpha * c.dt;
    state_.i_est_beta += di_est_beta * c.dt;

    // Kep dong uoc luong (Bao ve)
    state_.i_est_alpha = Clamp(state_.i_est_alpha, -200.0f, 200.0f);
    state_.i_est_beta = Clamp(state_.i_est_beta, -200.0f, 200.0f);

    // --- PLL / Back-EMF Observer ---
    float gain_scale = (c.mode == 0) ? 2.0e4f : 5.0e4f;

    float gamma1 = (115.0f / A) * gain_scale;
    float gamma2 = (c.mode == 0) ? 0.0f : 2000.0f;
    float k_theta = (c.mode == 0) ? 0.97f : (6.25f / A);

    float eps_bemf =
        (e_err_alpha * state_.e_est_beta) - (e_err_beta * state_.e_est_alpha);

    // Kep sai so BEMF
    eps_bemf = Clamp(eps_bemf, -100.0f, 100.0f);

    // Cap nhat BEMF
    state_.e_est_alpha +=
        (-state_.we_hat * state_.e_est_beta - c.l_gain * e_err_alpha) * c.dt;
    state_.e_est_beta +=
        (state_.we_hat * state_.e_est_alpha - c.l_gain * e_err_beta) * c.dt;

    // Kep BEMF
    state_.e_est_alpha = Clamp(state_.e_est_alpha, -50.0f, 50.0f);
    state_.e_est_beta = Clamp(state_.e_est_beta, -50.0f, 50.0f);

    // Cap nhat Van toc & Vi tri
    state_.we_hat += gamma1 * eps_bemf * c.dt;

    // Kep van toc uoc luong
    state_.we_hat = Clamp(state_.we_hat, -5000.0f, 5000.0f);

    state_.theta_hat += k_theta * (state_.we_hat + gamma2 * eps_bemf) * c.dt;

    // Chuan hoa goc: Dung if thay vi while de an toan (tranh treo MCU)
    if (state_.theta_hat > kPi)
      state_.theta_hat -= k2Pi;
    else if (state_.theta_hat < -kPi)
      state_.theta_hat += k2Pi;
  }

  const State &state() const { return state_; }
  State *mutable_state() { return &state_; }

private:
  State state_;
};
} // namespace moteus
