#pragma once

#include <cmath>
#include <cstdint>

namespace moteus {

// BO UOC LUONG THAM SO DONG CO BANG RLS
class ParameterEstimatorRLS {
 public:
  ParameterEstimatorRLS() {
    // Khoi tao cac gia tri mac dinh ban dau cho bo uoc luong
    Reset(0.0575f, 21.77e-6f, 0.0026f, 25.0f);
  }

  // HAM RESET TOAN BO: dat lai tat ca tham so uoc luong ve gia tri khoi tao ban dau
  void Reset(float init_R, float init_L, float init_flux, float init_temp_C) {
    init_R_ = init_R; init_L_ = init_L; init_flux_ = init_flux; init_temp_C_ = init_temp_C;

    // THIET LAP GIOI HAN AN TOAN cho tung tham so de tranh uoc luong lech khoi vung vat ly
    kRMin_  = init_R_ * 0.4f; kRMax_  = init_R_ * 3.0f;
    kLMin_  = init_L_ * 0.3f; kLMax_  = init_L_ * 1.5f;
    kFxMin_ = init_flux_ * 0.4f; kFxMax_ = init_flux_ * 1.5f;

    params_.R = init_R_; params_.Ld = init_L_; params_.Lq = init_L_; params_.flux_m = init_flux_;

    target_ld_ = init_L_; target_lq_ = init_L_;

    // KHOI TAO MO HINH NHIET KEP cho stator va rotor
    stator_temp_lpf_ = init_temp_C;
    rotor_temp_lpf_ = init_temp_C;
    cached_dt_ = 0.0f;

    id_svf_ = iq_svf_ = ud_svf_ = uq_svf_ = we_svf_ = 0.0f;
    did_dt_svf_ = diq_dt_svf_ = 0.0f;

    we_prev_ = 0.0f;
    accel_filt_ = 0.0f;
    kinematic_timer_ = 0.0f;

    l_phi_d_filt_ = l_y_d_filt_ = l_phi_q_filt_ = l_y_q_filt_ = 0.0f;
    r_phi_d_filt_ = r_y_d_filt_ = r_phi_q_filt_ = r_y_q_filt_ = 0.0f;
    f_phi_filt_ = f_y_filt_ = 0.0f;

    ResetMatrixState();

    warmup_ctr_ = l_ctr_ = rls_decimation_ctr_ = ldq_turn_ = 0;
  }

  // AP DUNG THONG SO HIEU CHINH MOI: duoc goi khi co file calibration hoac lenh stop/start
  void ApplyNewCalibration(float new_R, float new_L, float new_flux, float current_temp_C) {
        // Kiem tra xem thong so co thuc su thay doi hay khong
        bool is_new_config = (AbsF(new_R - init_R_) > 1e-7f) ||
                             (AbsF(new_flux - init_flux_) > 1e-8f);

        if (is_new_config) {
            // TRUONG HOP 1: CO FILE CALIBRATION MOI - ghi de hoan toan cac tham so co ban
            init_R_ = new_R;
            init_L_ = new_L;
            init_flux_ = new_flux;
            init_temp_C_ = current_temp_C;

            kRMin_  = init_R_ * 0.4f; kRMax_  = init_R_ * 3.0f;
            kLMin_  = init_L_ * 0.3f; kLMax_  = init_L_ * 1.5f;
            kFxMin_ = init_flux_ * 0.4f; kFxMax_ = init_flux_ * 1.5f;

            params_.R = init_R_;
            params_.Ld = init_L_;
            params_.Lq = init_L_;
            params_.flux_m = init_flux_;

            target_ld_ = init_L_;
            target_lq_ = init_L_;

            stator_temp_lpf_ = current_temp_C;
            rotor_temp_lpf_ = current_temp_C;

            ResetMatrixState();
            warmup_ctr_ = 0; // Kich hoat lai qua trinh warmup cho RLS
        } else {
            // TRUONG HOP 2: CHI LA STOP ROI START LAI - giu nguyen thanh qua uoc luong, chi cap nhat neo
            stator_temp_lpf_ = current_temp_C;
            rotor_temp_lpf_ = current_temp_C;

            r_anchor_R_ = params_.R;
            r_anchor_T_ = current_temp_C;

            flux_anchor_flux_ = params_.flux_m;
            flux_anchor_T_ = current_temp_C;
        }

        // LUON LUON xoa trang thai bo loc SVF de tranh soc dao ham khi FET bat lai
        id_svf_ = iq_svf_ = ud_svf_ = uq_svf_ = we_svf_ = 0.0f;
        did_dt_svf_ = diq_dt_svf_ = 0.0f;
    }

  // CAP NHAT CHINH: ham thuc thi uoc luong tham so theo thoi gian thuc
  void Update(float id, float iq, float ud, float uq, float we, float dt, float temp_C) __attribute__((always_inline)) {
    if (dt <= 0.0f) return;

    // ================================================================
    // MO HINH NHIET KEP: stator thay doi nhanh, rotor thay doi cham
    // ================================================================
    stator_temp_lpf_ += kAlphaStatorTemp * (temp_C - stator_temp_lpf_);
    rotor_temp_lpf_ += kAlphaRotorTemp * (stator_temp_lpf_ - rotor_temp_lpf_);

    if (dt != cached_dt_) cached_dt_ = dt;
    const float inv_dt = 1.0f / dt;

    // Bu sap ap deadtime de hieu chinh dien ap dat vao dong co chinh xac hon
    const float v_drop_q = SmoothDrop(iq, 0.055f, 0.010f);
    const float v_drop_d = SmoothDrop(id, 0.055f, 0.010f);
    const float uq_comp = uq - v_drop_q;
    const float ud_comp = ud - v_drop_d;

    // ================================================================
    // BO LOC BIEN TRANG THAI (SVF) chay o tan so 30kHz
    // tinh dao ham dong dien va lam min cac tin hieu nhieu truoc khi dua vao RLS
    // ================================================================
    float did_dt_raw = (id - id_svf_) * (kAlphaSVF * inv_dt);
    float diq_dt_raw = (iq - iq_svf_) * (kAlphaSVF * inv_dt);

    did_dt_svf_ = did_dt_raw;
    diq_dt_svf_ = diq_dt_raw;

    id_svf_ += kAlphaSVF * (id - id_svf_);
    iq_svf_ += kAlphaSVF * (iq - iq_svf_);
    ud_svf_ += kAlphaSVF * (ud_comp - ud_svf_);
    uq_svf_ += kAlphaSVF * (uq_comp - uq_svf_);
    we_svf_ += kAlphaSVF * (we - we_svf_);

    // TINH TOAN GIA TOC VA KIEM TRA ON DINH DONG HOC
    // phat hien trang thai qua do de tranh uoc luong sai khi dong co dang thay doi toc do dot ngot
    float accel_raw = (we_svf_ - we_prev_) * inv_dt;
    we_prev_ = we_svf_;
    accel_filt_ += 0.005f * (accel_raw - accel_filt_);

    if (AbsF(accel_filt_) > 200.0f) kinematic_timer_ = 0.0f;
    else kinematic_timer_ += dt;

    // GIAI DOAN WARMUP: cho cac bo loc va tin hieu on dinh truoc khi bat dau uoc luong that su
    if (warmup_ctr_ < kWarmupCycles) {
    	++warmup_ctr_;
		  theta_R_ = params_.R;
		  theta_flux_ = params_.flux_m;
		  return;
    }

    // ================================================================
    // CAP NHAT DIEN CAM L, chay o tan so 5kHz
    // lan luot uoc luong Ld va Lq de tranh tuong ho cheo giua hai truc toa do
    // ================================================================
    if (++l_ctr_ >= kLDecim) {
        l_ctr_ = 0;

        if (ldq_turn_ == 0) {
            // Uoc luong Ld tu truc d dua tren dao ham dong dien id
            float phi_Ld_raw = did_dt_svf_;
            float y_Ld_raw = ud_svf_ - theta_R_ * id_svf_ + we_svf_ * theta_lq_ * iq_svf_;
            l_phi_d_filt_ += kAlphaMacro_3kHz * (phi_Ld_raw - l_phi_d_filt_);
            l_y_d_filt_ += kAlphaMacro_3kHz * (y_Ld_raw - l_y_d_filt_);

            UpdateLRobust(id_svf_, iq_svf_, ud_svf_, we_svf_,
                          theta_ld_, P_ld_, l_phi_d_filt_, l_y_d_filt_, target_ld_);
        } else {
            // Uoc luong Lq tu truc q dua tren dao ham dong dien iq
            float phi_Lq_raw = diq_dt_svf_;
            float y_Lq_raw = uq_svf_ - theta_R_ * iq_svf_ - we_svf_ * theta_ld_ * id_svf_ - we_svf_ * theta_flux_;
            l_phi_q_filt_ += kAlphaMacro_3kHz * (phi_Lq_raw - l_phi_q_filt_);
            l_y_q_filt_ += kAlphaMacro_3kHz * (y_Lq_raw - l_y_q_filt_);

            UpdateLRobust(id_svf_, iq_svf_, uq_svf_, we_svf_,
                          theta_lq_, P_lq_, l_phi_q_filt_, l_y_q_filt_, target_lq_);
        }
        ldq_turn_ ^= 1;
    }

    // ================================================================
    // BO LOC CHO UOC LUONG R VA FLUX: chay nen tan so 30kHz de lay tin hieu sach truoc khi uoc luong
    // ================================================================
    // Bo loc cho dien tro R tren truc d
    float phi_Rd_raw = id_svf_;
    float y_Rd_raw = ud_svf_ + we_svf_ * theta_lq_ * iq_svf_ - theta_ld_ * did_dt_svf_;
    r_phi_d_filt_ += kAlphaFR * (phi_Rd_raw - r_phi_d_filt_);
    r_y_d_filt_ += kAlphaFR * (y_Rd_raw - r_y_d_filt_);

    // Bo loc cho dien tro R tren truc q
    float phi_Rq_raw = iq_svf_;
    float y_Rq_raw = uq_svf_ - we_svf_ * theta_ld_ * id_svf_ - we_svf_ * theta_flux_ - theta_lq_ * diq_dt_svf_;
    r_phi_q_filt_ += kAlphaFR * (phi_Rq_raw - r_phi_q_filt_);
    r_y_q_filt_ += kAlphaFR * (y_Rq_raw - r_y_q_filt_);

    // Bo loc cho tu thong flux_m
    float phi_F_raw = we_svf_;
    float y_F_raw = uq_svf_ - theta_R_ * iq_svf_ - we_svf_ * theta_ld_ * id_svf_ - theta_lq_ * diq_dt_svf_;
    f_phi_filt_ += kAlphaFR * (phi_F_raw - f_phi_filt_);
    f_y_filt_ += kAlphaFR * (y_F_raw - f_y_filt_);

    // ================================================================
    // CAP NHAT R VA FLUX, chay o tan so 1kHz
    // ================================================================
    if (++rls_decimation_ctr_ >= kRlsDecimation) {
      rls_decimation_ctr_ = 0;

      float i_mag_sq = id_svf_ * id_svf_ + iq_svf_ * iq_svf_;
      bool has_current = (i_mag_sq > 0.25f);
      bool is_kinematic_safe = (kinematic_timer_ > 0.1f);

      bool is_steady_R = (AbsF(did_dt_svf_) < 150.0f && AbsF(diq_dt_svf_) < 150.0f);
      bool is_steady_Flux = (AbsF(did_dt_svf_) < 50.0f && AbsF(diq_dt_svf_) < 50.0f);

      if (is_kinematic_safe && has_current) {

          // DIEN TRO R -> chi cap nhat khi dao dam dong dien o muc vua phai
          if (is_steady_R) {
              bool updated_R = false;
              if (AbsF(r_phi_d_filt_) > 1.0f) {
                  RunScalarRLS(r_phi_d_filt_, r_y_d_filt_, theta_R_, P_R_, kLambdaR, kRMin_, kRMax_, 2.0f, 1e-8f, 1e-5f);
                  updated_R = true;
              } else if (AbsF(r_phi_q_filt_) > 1.0f) {
                  RunScalarRLS(r_phi_q_filt_, r_y_q_filt_, theta_R_, P_R_, kLambdaR, kRMin_, kRMax_, 2.0f, 1e-8f, 1e-5f);
                  updated_R = true;
              }
              // Cap nhat lai Moi neo khi RLS uoc luong duoc tin hieu tot
              if (updated_R) {
                  r_anchor_R_ = theta_R_;
                  r_anchor_T_ = stator_temp_lpf_;
              }
          }

          // TU THONG Phi_m -> yeu cau dong co quay voi toc do du lon de co succ dien dong cam ung
          if (is_steady_Flux && AbsF(we_svf_) > 20.0f) {
              RunScalarRLS(f_phi_filt_, f_y_filt_, theta_flux_, P_flux_, kLambdaFlux, kFxMin_, kFxMax_, 2.0f, 1e-10f, 5e-8f);
              flux_anchor_flux_ = theta_flux_;
              flux_anchor_T_ = rotor_temp_lpf_;
          }
      }
    }

    // ================================================================
    // XUAT KET QUA CHO FOC: chuyen doi tham so uoc luong thanh gia tri su dung cho bo dieu khien
    // ================================================================

    // TINH TOAN DAU RA Rs: thay doi nhanh theo nhiet do stator va co gioi han toc do thay doi tranh dao dong
    const float max_r_step = 0.05f * dt;
    float target_r = r_anchor_R_ * (1.0f + kAlphaCu * (stator_temp_lpf_ - r_anchor_T_));
    target_r = ClampF(target_r, kRMin_, kRMax_);

    if (target_r - params_.R > max_r_step) params_.R += max_r_step;
    else if (target_r - params_.R < -max_r_step) params_.R -= max_r_step;
    else params_.R = target_r;

    // TINH TOAN DAU RA Phi_m: phu thuoc hoan toan vao tre nhiet cua rotor, thay doi cham hon
    const float max_flux_step = 0.00001f * dt;
    float target_flux = flux_anchor_flux_ * (1.0f + kAlphaFlux * (rotor_temp_lpf_ - flux_anchor_T_));
    target_flux = ClampF(target_flux, kFxMin_, kFxMax_);

    if (target_flux - params_.flux_m > max_flux_step) params_.flux_m += max_flux_step;
    else if (target_flux - params_.flux_m < -max_flux_step) params_.flux_m -= max_flux_step;
    else params_.flux_m = target_flux;

    // TINH TOAN DAU RA Ld, Lq: loc anti-chatter de tranh dao dong khi dieu khien vi L thay doi dot ngot
    params_.Ld += 20.0f * dt * (ClampF(target_ld_, kLMin_, kLMax_) - params_.Ld);
    params_.Lq += 20.0f * dt * (ClampF(target_lq_, kLMin_, kLMax_) - params_.Lq);
  }

  // CAP NHAT CHE DO STANDY: duoc goi khi dong co dung yen, chi cap nhat nhiet va tham so theo mo hinh
  void UpdateStandby(float dt, float temp_C) {
    if (dt <= 0.0f) return;

    // Cap nhat mo hinh nhiet kep ngay ca khi dung yen de theo doi su nguoi dan cua dong co
    stator_temp_lpf_ += kAlphaStatorTemp * (temp_C - stator_temp_lpf_);
    rotor_temp_lpf_ += kAlphaRotorTemp * (stator_temp_lpf_ - rotor_temp_lpf_);

    const float max_r_step = 0.005f * dt;
    float target_r = ClampF(r_anchor_R_ * (1.0f + kAlphaCu * (stator_temp_lpf_ - r_anchor_T_)), kRMin_, kRMax_);
    if (target_r - params_.R > max_r_step) params_.R += max_r_step;
    else if (target_r - params_.R < -max_r_step) params_.R -= max_r_step;
    else params_.R = target_r;

    const float max_flux_step = 0.00001f * dt;
    float target_flux = ClampF(flux_anchor_flux_ * (1.0f + kAlphaFlux * (rotor_temp_lpf_ - flux_anchor_T_)), kFxMin_, kFxMax_);
    if (target_flux - params_.flux_m > max_flux_step) params_.flux_m += max_flux_step;
    else if (target_flux - params_.flux_m < -max_flux_step) params_.flux_m -= max_flux_step;
    else params_.flux_m = target_flux;
  }

  // CAC HAM LAY THAM SO DA UOC LUONG DE PHUC VU CHO BO DIEU KHIEN FOC
  float GetR()   const { return params_.R; }
  float GetFai() const { return params_.flux_m; }
  float GetLd()  const { return params_.Ld; }
  float GetLq()  const { return params_.Lq; }

 private:
  static constexpr int kWarmupCycles = 2000;

  static constexpr float kAlphaStatorTemp = 0.001f;
  static constexpr float kAlphaRotorTemp  = 0.00001f;

  static constexpr float kAlphaSVF = 0.02f;
  static constexpr float kAlphaFR  = 0.05f;
  static constexpr float kAlphaCu = 0.00393f;
  static constexpr float kAlphaFlux = -0.001f;

  // CAC HANG SO CHO UOC LUONG L
  static constexpr int kLDecim = 6;
  static constexpr float kAlphaMacro_3kHz = 0.1f;
  static constexpr float kMacroDiDtThreshold = 50.0f;
  static constexpr float kWeDecoupleThreshold = 15.0f;
  static constexpr float kLOutAlpha = 0.1f;
  static constexpr float kLDesatAlpha = 0.01f;
  const float kLambdaL = 0.99f;

  // CAC HANG SO CHO UOC LUONG R VA FLUX
  static constexpr int kRlsDecimation = 30;
  const float kLambdaR = 0.999f;
  const float kLambdaFlux = 0.999995f;

  float init_R_, init_L_, init_flux_, init_temp_C_;
  float kLMin_, kLMax_, kRMin_, kRMax_, kFxMin_, kFxMax_;
  struct EstimatedParams { float R, Ld, Lq, flux_m; } params_;

  float stator_temp_lpf_, rotor_temp_lpf_, cached_dt_;

  float id_svf_, iq_svf_, ud_svf_, uq_svf_, we_svf_;
  float did_dt_svf_, diq_dt_svf_;

  float we_prev_, accel_filt_, kinematic_timer_;

  float theta_ld_, P_ld_, theta_lq_, P_lq_;
  float theta_R_, P_R_, r_anchor_R_, r_anchor_T_;
  float theta_flux_, P_flux_, flux_anchor_flux_, flux_anchor_T_;

  float l_phi_d_filt_, l_y_d_filt_, l_phi_q_filt_, l_y_q_filt_;
  float r_phi_d_filt_, r_y_d_filt_, r_phi_q_filt_, r_y_q_filt_;
  float f_phi_filt_, f_y_filt_;

  float target_ld_, target_lq_;

  int warmup_ctr_, l_ctr_, rls_decimation_ctr_, ldq_turn_;

  // KHOI TAO LAI TRANG THAI MA TRAN: dat lai cac ma tran P va cac bo loc cho RLS
  void ResetMatrixState() {
    theta_R_ = params_.R;
    theta_flux_ = params_.flux_m;
    theta_ld_ = init_L_;
    theta_lq_ = init_L_;
    r_anchor_R_ = params_.R;
    flux_anchor_flux_ = params_.flux_m;
    r_anchor_T_ = stator_temp_lpf_;
    flux_anchor_T_ = rotor_temp_lpf_;

    P_R_ = 1e-5f;
    P_flux_ = 1e-7f;
    P_ld_ = 1e-8f;
    P_lq_ = 1e-8f;

    l_phi_d_filt_ = l_y_d_filt_ = 0.0f;
    l_phi_q_filt_ = l_y_q_filt_ = 0.0f;
    r_phi_d_filt_ = r_y_d_filt_ = 0.0f;
    r_phi_q_filt_ = r_y_q_filt_ = 0.0f;
    f_phi_filt_   = f_y_filt_   = 0.0f;
  }

  // HAM BU SUT AP DEADTIME: sut ap do FET va dien tro day dan
  static inline float SmoothDrop(float current, float deadtime_v, float resistive_v_per_a) {
    const float threshold = 0.05f; float v_dt = 0.0f;
    if (current > threshold) v_dt = deadtime_v;
    else if (current < -threshold) v_dt = -deadtime_v;
    else v_dt = (current / threshold) * deadtime_v;
    return v_dt + current * resistive_v_per_a;
  }

  // CAP NHAT L THEO PHUONG PHAP ROBUST
  void UpdateLRobust(float id_sync, float iq_sync, float u_sync, float we_sync,
                     float& theta, float& P, float phi_filt, float y_filt,
                     float& target_L) {
    float abs_phi = AbsF(phi_filt);
    float i_mag_sq = id_sync * id_sync + iq_sync * iq_sync;

    // Chi cap nhat khi di/dt du lon de dam bao tin hieu phi co y nghia thong ke
    if (abs_phi > kMacroDiDtThreshold) {
        float we_penalty = 1.0f;
        float abs_we = AbsF(we_sync);
        if (abs_we > kWeDecoupleThreshold) {
            we_penalty = kWeDecoupleThreshold / abs_we;
        }

        float clip = 0.5f * (AbsF(u_sync) + 1.0f);

        if (phi_filt > 1e-9f || phi_filt < -1e-9f) {
            const float phiP = phi_filt * P;
            const float denom = kLambdaL + phi_filt * phiP;

            if (denom > 1e-12f) {
                const float K = phiP / denom;
                float err = y_filt - theta * phi_filt;

                if (err > clip) err = clip; else if (err < -clip) err = -clip;

                theta += K * err * we_penalty;
                theta = ClampF(theta, kLMin_, kLMax_);

                P = (P - K * phiP) / kLambdaL;

                if (P < 1e-12f) P = 1e-12f;
                else if (P > 5e-8f) P = 5e-8f;
            }
        }
        target_L += kLOutAlpha * (theta - target_L);
    }
    else {
        // Chi ho tro xa bao hoa tu nhien (tro ve Unmagnetized State) NEU dong dien tut xuong rat thap (< 0.25A)
        // de tranh uoc luong bi lech khi khong du kich thich
        if (i_mag_sq < 0.25f) {
            target_L += kLDesatAlpha * (init_L_ - target_L);
            theta = target_L;
        }
    }
  }

  // BO UOC LUONG RLS VO HUONG: thuat toan core recursive least squares de uoc luong tung tham so rieng biet
  static void RunScalarRLS(float phi, float y, float& theta, float& P, float lambda, float theta_min, float theta_max, float clip, float p_min, float p_max) __attribute__((always_inline)) {
    if (phi < 1e-9f && phi > -1e-9f) return;
    float P_phi = P * phi;
    float denom = lambda + phi * P_phi;
    if (denom < 1e-12f) return;

    float K = P_phi / denom;
    float err = y - theta * phi;

    if (err > clip) err = clip;
    else if (err < -clip) err = -clip;

    float new_theta = theta + K * err;
    if (!std::isnan(new_theta) && !std::isinf(new_theta)) {
        theta = ClampF(new_theta, theta_min, theta_max);

        P = (P - K * phi * P) / lambda;
        if (P < p_min) P = p_min;
        else if (P > p_max) P = p_max;
    }
  }

  static inline float AbsF(float x) { return x < 0.0f ? -x : x; }
  static inline float ClampF(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};

} // namespace moteus
