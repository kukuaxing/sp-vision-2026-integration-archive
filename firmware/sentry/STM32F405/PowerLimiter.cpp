#include "PowerLimiter.h"
#include "supercap.h"
#include <cmath>

PowerLimiter power_limiter;

namespace {
constexpr float K_M3508_GEAR_RATIO = 19.2f;  // M3508 减速比（输出轴转速 = 电机轴转速 / 该值）
constexpr float K_C620_CURRENT_PER_LSB = 20.0f / 16384.0f;  // C620 电流指令单位换算：每 1 LSB 对应电流(A)
// The datasheet torque constant is for the geared output shaft.
// This project uses motor-side RPM for curspeed/setspeed, so the power model
// must convert the output torque constant back to the motor side.
constexpr float K_M3508_OUTPUT_TORQUE_PER_AMP = 0.30f;  // M3508 输出轴扭矩常数(N·m/A)
constexpr float K_TORQUE =
    (K_M3508_OUTPUT_TORQUE_PER_AMP / K_M3508_GEAR_RATIO) * K_C620_CURRENT_PER_LSB;  // 将电流指令 LSB 转为电机轴侧扭矩系数(N·m/LSB)
constexpr bool K_ENABLE_RLS = false;  // 是否启用 RLS 在线辨识更新 k1/k2（调试初期建议 false）
constexpr float K_MODEL_SAFETY_RATIO = 1.15f;  // 预测功率安全系数（>1 更保守，越大越不易超功率）
constexpr uint16_t K_SPIN_STARTUP_FRAMES = 90;  // 自旋起步保护持续帧数（2ms 周期下约 180ms）
constexpr float K_SPIN_STARTUP_LIMIT_START_RATIO = 0.72f;  // 起步首帧有效功率上限比例（相对 limit_power）
constexpr float K_SPIN_STARTUP_LIMIT_CAP_BONUS = 0.10f;  // 电容电量对起步功率上限比例的额外放宽
constexpr float K_SPIN_STARTUP_SCALE_START = 0.62f;  // 起步首帧电流缩放上限（scale_target 最大值）
constexpr float K_SPIN_STARTUP_SCALE_CAP_BONUS = 0.10f;  // 电容电量对起步 scale 上限的额外放宽
constexpr float K_SPIN_STARTUP_ENTER_RPM = 450.0f;  // 判定“请求起步”目标转速阈值（|setspeed| 超过触发）
constexpr float K_SPIN_STARTUP_EXIT_RPM = 150.0f;  // 退出起步判定的目标转速阈值（|setspeed| 低于清零）
constexpr float K_SPIN_STARTUP_ACTUAL_RPM = 180.0f;  // 仅在当前实际转速较低时才允许进入起步保护
constexpr float K_CAP_READY_ENERGY = 600.0f;  // 电容“可用”能量下限（低于该值不提供额外增益）
constexpr float K_CAP_FULL_ENERGY = 1764.0f;  // 电容“满电”能量标定值（用于归一化 cap_energy_ratio）
constexpr float K_CAP_POWER_BONUS_MAX = 0.0f;  // 常规状态电容可提供的最大额外功率(W)
constexpr float K_CAP_STARTUP_POWER_BONUS_MAX = 20.0f;  // 起步阶段电容可提供的最大额外功率(W)

// K1 uses motor-side angular speed as well, so it must be scaled by gear ratio.
constexpr float K_BASE_K1 = 0.015f;  // 功率模型线性速度项系数：k1 * Σ|w|（粘滞/铁耗近似）
constexpr float K_BASE_K2 = 7.6e-7f;  // 功率模型电流平方项系数：k2 * ΣI²（铜耗近似）
constexpr float K_BASE_K3 = 2.50f;  // 功率模型常数项偏置
constexpr float K_FEEDBACK_FILTER_ALPHA = 0.35f;  // 实测功率一阶低通滤波系数（大=响应快，小=更平滑）
constexpr float K_FEEDBACK_TRACK_ALPHA = 0.12f;  // 无新鲜反馈时，滤波值向预测值跟踪的系数
constexpr float K_RELEASE_THRESHOLD_RATIO = 0.96f;  // 低于该比例才放松功率修正（防止来回抖动）
constexpr float K_RELEASE_GAIN = 0.012f;  // 放松修正增益（反馈明显低于阈值时提升可用功率）
constexpr float K_TIGHTEN_GAIN = 0.24f;  // 收紧修正增益（反馈超限时快速降低可用功率）
constexpr float K_POWER_CORRECTION_MIN = -0.36f;  // 动态功率修正下限（最严格可降至 64% limit）
constexpr float K_POWER_CORRECTION_MAX = 0.02f;  // 动态功率修正上限（最多小幅放宽 2%）
constexpr float K_STATIC_POWER_MARGIN_W = 3.0f;  // 固定安全余量(W)，从有效功率上限中直接扣除
constexpr float K_LOW_SPEED_LIMIT_BOOST = 1.00f;  // 低速区有效功率倍率（1.0 表示不额外放宽）
constexpr float K_HARD_BRAKE_TRIGGER_RATIO = 1.02f;  // 硬刹车触发阈值（反馈功率超过 limit 的比例）
constexpr float K_HARD_BRAKE_GAIN = 1.05f;  // 硬刹车强度增益（越大压制越猛）
constexpr float K_HARD_BRAKE_MIN_FACTOR = 0.12f;  // 硬刹车后 scale 最小保底系数
constexpr float K_FEEDBACK_CLAMP_MARGIN = 0.88f;  // 反馈夹紧裕量（超限时按 limit*该值回推 scale）
constexpr float K_K1_MIN = K_BASE_K1;  // RLS 更新后 k1 的下限
constexpr float K_K1_MAX = 0.050f;  // RLS 更新后 k1 的上限
constexpr float K_K2_MIN = K_BASE_K2;  // RLS 更新后 k2 的下限
constexpr float K_K2_MAX = 8.0e-7f;  // RLS 更新后 k2 的上限
}

PowerLimiter::PowerLimiter() : rls(200.0f, 0.985f) {
    k1 = K_BASE_K1;
    k2 = K_BASE_K2;
    k3 = K_BASE_K3;
    last_scale = 1.0f;
    power_correction_ = 0.0f;
    last_limit_power_ = 0.0f;
    filtered_power_feedback_ = 0.0f;
    has_filtered_power_feedback_ = false;
    last_avg_setspeed_rpm_ = 0.0f;
    spin_start_frames_ = 0;
    last_predicted_power_ = 0.0f;
    last_model_debug_ = {};
}

void PowerLimiter::Init() {
    k1 = K_BASE_K1;
    k2 = K_BASE_K2;
    k3 = K_BASE_K3;
    rls.reset();
    rls.set_params(k1, k2);
    last_scale = 1.0f;
    power_correction_ = 0.0f;
    last_limit_power_ = 0.0f;
    filtered_power_feedback_ = 0.0f;
    has_filtered_power_feedback_ = false;
    last_avg_setspeed_rpm_ = 0.0f;
    spin_start_frames_ = 0;
    last_predicted_power_ = 0.0f;
    last_model_debug_ = {};
}

float PowerLimiter::GetPredictedPower() const {
    return last_predicted_power_;
}

PowerLimiter::ModelDebugData PowerLimiter::GetModelDebugData() const {
    return last_model_debug_;
}

float PowerLimiter::clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

void PowerLimiter::Process(Motor* chassis_motors[4], float real_power_feedback, float limit_power,
    bool feedback_is_fresh) {
    if (limit_power < 1.0f) {
        limit_power = 1.0f;
    }
    float cap_energy_ratio = 0.0f;
    if (supercap.connect) {
        cap_energy_ratio = clamp(
            (supercap.Rxsuper.cap_energy - K_CAP_READY_ENERGY) /
            (K_CAP_FULL_ENERGY - K_CAP_READY_ENERGY),
            0.0f, 1.0f);
    }
    const float cap_power_bonus = K_CAP_POWER_BONUS_MAX * cap_energy_ratio;

    float predicted_power = 0.0f;
    float sum_torque_power = 0.0f;

    Math::Matrix<2, 1> phi;
    float sum_abs_w = 0.0f;
    float sum_current_sq = 0.0f;
    float max_wheel_speed = 0.0f;
    float sum_setspeed_rpm = 0.0f;
    float sum_curspeed_rpm = 0.0f;
    int active_motor_count = 0;

    for (int i = 0; i < 4; i++) {
        Motor* m = chassis_motors[i];
        if (m == nullptr) continue;
        active_motor_count++;

        float w = static_cast<float>(m->curspeed) * (3.14159f / 30.0f);
        if (std::abs(w) > max_wheel_speed) {
            max_wheel_speed = std::abs(w);
        }

        float raw_current = static_cast<float>(m->current);
        float p_mech = (raw_current * K_TORQUE) * w;

        if (p_mech > 0.0f) {
            sum_torque_power += p_mech;
        }

        sum_abs_w += std::abs(w);
        sum_current_sq += raw_current * raw_current;
        sum_setspeed_rpm += static_cast<float>(m->setspeed);
        sum_curspeed_rpm += static_cast<float>(m->curspeed);
    }

    predicted_power = sum_torque_power + k1 * sum_abs_w + k2 * sum_current_sq + k3;
    const float k1_sum_abs_w = k1 * sum_abs_w;
    const float k2_sum_current_sq = k2 * sum_current_sq;
    const float predicted_power_raw = sum_torque_power + k1_sum_abs_w + k2_sum_current_sq + k3;
    predicted_power = predicted_power_raw * K_MODEL_SAFETY_RATIO;
    last_predicted_power_ = predicted_power;
    last_model_debug_.sum_torque_power = sum_torque_power;
    last_model_debug_.k1_sum_abs_w = k1_sum_abs_w;
    last_model_debug_.k2_sum_current_sq = k2_sum_current_sq;
    last_model_debug_.k3 = k3;
    last_model_debug_.predicted_power_raw = predicted_power_raw;
    last_model_debug_.predicted_power_with_safety = predicted_power;
    last_model_debug_.real_power_feedback = real_power_feedback;

    float avg_setspeed_rpm = 0.0f;
    float avg_curspeed_rpm = 0.0f;
    if (active_motor_count > 0) {
        avg_setspeed_rpm = sum_setspeed_rpm / static_cast<float>(active_motor_count);
        avg_curspeed_rpm = sum_curspeed_rpm / static_cast<float>(active_motor_count);
    }

    const bool entering_spin_request =
        std::abs(avg_setspeed_rpm) > K_SPIN_STARTUP_ENTER_RPM &&
        std::abs(last_avg_setspeed_rpm_) < K_SPIN_STARTUP_EXIT_RPM &&
        std::abs(avg_curspeed_rpm) < K_SPIN_STARTUP_ACTUAL_RPM;

    if (entering_spin_request) {
        spin_start_frames_ = K_SPIN_STARTUP_FRAMES;
    } else if (std::abs(avg_setspeed_rpm) < K_SPIN_STARTUP_EXIT_RPM) {
        spin_start_frames_ = 0;
    }
    last_avg_setspeed_rpm_ = avg_setspeed_rpm;

    const bool has_feedback = std::abs(real_power_feedback) > 3.0f;
    if (feedback_is_fresh && has_feedback) {
        if (!has_filtered_power_feedback_) {
            filtered_power_feedback_ = real_power_feedback;
            has_filtered_power_feedback_ = true;
        } else {
            filtered_power_feedback_ +=
                K_FEEDBACK_FILTER_ALPHA * (real_power_feedback - filtered_power_feedback_);
        }
    } else if (has_filtered_power_feedback_) {
        filtered_power_feedback_ +=
            K_FEEDBACK_TRACK_ALPHA * (predicted_power - filtered_power_feedback_);
    }

    const bool use_feedback = has_filtered_power_feedback_;
    const bool use_fresh_feedback = feedback_is_fresh && has_filtered_power_feedback_;
    const float feedback_power = has_filtered_power_feedback_
        ? filtered_power_feedback_
        : real_power_feedback;

    if (K_ENABLE_RLS && use_fresh_feedback && predicted_power > 3.0f) {
        float y = feedback_power - sum_torque_power - k3;

        if (std::abs(y) < 200.0f) {
            phi.data[0][0] = sum_abs_w;
            phi.data[1][0] = sum_current_sq;

            rls.update(phi, y);

            float new_k1 = rls.get_k1();
            float new_k2 = rls.get_k2();

            k1 = clamp(new_k1, K_K1_MIN, K_K1_MAX);
            k2 = clamp(new_k2, K_K2_MIN, K_K2_MAX);
        }
    }

    if (last_limit_power_ > 0.0f && limit_power < last_limit_power_ * 0.85f) {
        power_correction_ = 0.0f;
    }
    last_limit_power_ = limit_power;

    if (use_fresh_feedback) {
        const float lower_threshold = limit_power * K_RELEASE_THRESHOLD_RATIO;

        if (feedback_power > limit_power) {
            power_correction_ -= K_TIGHTEN_GAIN * (feedback_power - limit_power) / limit_power;
        } else if (feedback_power < lower_threshold) {
            power_correction_ += K_RELEASE_GAIN * (lower_threshold - feedback_power) / limit_power;
        }
        power_correction_ = clamp(power_correction_, K_POWER_CORRECTION_MIN, K_POWER_CORRECTION_MAX);
    }

    float effective_limit =
        limit_power * (1.0f + power_correction_) + cap_power_bonus - K_STATIC_POWER_MARGIN_W;
    if (effective_limit < 1.0f) {
        effective_limit = 1.0f;
    }

    const bool spin_start_active = spin_start_frames_ > 0;
    float spin_start_progress = 0.0f;
    if (spin_start_active) {
        spin_start_progress = 1.0f -
            static_cast<float>(spin_start_frames_) / static_cast<float>(K_SPIN_STARTUP_FRAMES);
        const float startup_limit_start_ratio =
            K_SPIN_STARTUP_LIMIT_START_RATIO + K_SPIN_STARTUP_LIMIT_CAP_BONUS * cap_energy_ratio;
        float startup_limit_ratio = startup_limit_start_ratio +
            (1.0f - startup_limit_start_ratio) * spin_start_progress;
        effective_limit *= startup_limit_ratio;
        const float startup_cap_bonus =
            K_CAP_STARTUP_POWER_BONUS_MAX * cap_energy_ratio * (1.0f - spin_start_progress);
        effective_limit += startup_cap_bonus;
    }

    if (max_wheel_speed < 10.0f) {
        effective_limit *= K_LOW_SPEED_LIMIT_BOOST;
    }

    float scale_target = 1.0f;

    if (predicted_power > effective_limit) {
        float a = k2 * sum_current_sq;
        float b = sum_torque_power;
        float c = k1 * sum_abs_w + k3 - effective_limit;

        if (std::abs(a) < 1.0e-9f) {
            scale_target = (std::abs(b) > 1.0e-9f)
                ? clamp((effective_limit - (k1 * sum_abs_w + k3)) / b, 0.0f, 1.0f)
                : 0.0f;
        } else {
            float delta = b * b - 4.0f * a * c;
            if (delta >= 0.0f) {
                float sqrt_delta = std::sqrt(delta);
                float x1 = (-b + sqrt_delta) / (2.0f * a);
                float x2 = (-b - sqrt_delta) / (2.0f * a);

                float best = -1.0f;
                if (x1 > 0.0f && x1 <= 1.0f) best = x1;
                if (x2 > 0.0f && x2 <= 1.0f && x2 > best) best = x2;

                if (best > 0.0f) {
                    scale_target = best;
                } else {
                    float fallback = (x1 > 0.0f) ? x1 : ((x2 > 0.0f) ? x2 : 0.0f);
                    scale_target = clamp(fallback, 0.0f, 1.0f);
                }
            } else {
                scale_target = 0.0f;
            }
        }
    }

    if (use_feedback && feedback_power > limit_power) {
        float feedback_scale = clamp((limit_power * K_FEEDBACK_CLAMP_MARGIN) / feedback_power,
            0.0f, 1.0f);
        if (feedback_scale < scale_target) {
            scale_target = feedback_scale;
        }
    }

    if (spin_start_active) {
        const float startup_scale_start =
            K_SPIN_STARTUP_SCALE_START + K_SPIN_STARTUP_SCALE_CAP_BONUS * cap_energy_ratio;
        float startup_scale_cap = startup_scale_start +
            (1.0f - startup_scale_start) * spin_start_progress;
        if (scale_target > startup_scale_cap) {
            scale_target = startup_scale_cap;
        }
        spin_start_frames_--;
    }

    scale_target = clamp(scale_target, 0.05f, 1.0f);
    if (scale_target < last_scale) {
        last_scale = last_scale * 0.2f + scale_target * 0.8f;
    } else {
        last_scale = last_scale * 0.75f + scale_target * 0.25f;
    }

    if (use_feedback &&
        feedback_power > limit_power * K_HARD_BRAKE_TRIGGER_RATIO) {
        float excess_ratio = (feedback_power - limit_power) / limit_power;
        last_scale *= clamp(1.0f - excess_ratio * K_HARD_BRAKE_GAIN,
            K_HARD_BRAKE_MIN_FACTOR, 1.0f);
    }
    last_scale = clamp(last_scale, 0.05f, 1.0f);

    for (int i = 0; i < 4; i++) {
        if (chassis_motors[i] != nullptr) {
            chassis_motors[i]->current =
                static_cast<int32_t>(chassis_motors[i]->current * last_scale);
        }
    }
}
