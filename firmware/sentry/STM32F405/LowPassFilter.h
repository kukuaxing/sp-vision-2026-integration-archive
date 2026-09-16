#pragma once
class LowPassFilter {
public:
    // 构造函数
    // alpha 越小，滤波效果越强（越平滑，但延迟越高）
    // alpha 越大，滤波效果越弱（越跟随，但抖动越大）
    LowPassFilter(float alpha) : alpha_(alpha), last_output_(0.0f), initialized_(false) {}

    // 更新滤波器并返回滤波后的值
    float update(float input) {
        if (!initialized_) {
            // 第一次调用时，直接用输入值初始化，防止跳变
            last_output_ = input;
            initialized_ = true;
        }
        else {
            // 滤波公式：
            // 当前输出 = alpha * 当前输入 + (1 - alpha) * 上一次输出
            last_output_ = alpha_ * input + (1.0f - alpha_) * last_output_;
        }
        return last_output_;
    }

private:
    float alpha_;         // 滤波系数 (0.0 < alpha < 1.0)
    float last_output_;   // 上一次的输出值
    bool initialized_;    // 是否已初始化
};