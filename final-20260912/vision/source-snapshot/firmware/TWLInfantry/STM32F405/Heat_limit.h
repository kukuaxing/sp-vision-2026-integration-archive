#pragma once
#include <cstdint>

class HeatLimiter
{
public:
    void Init(float safetyMargin = 0.90f, float resumeThreshold = 0.50f);

    // 每个控制周期调用，从裁判系统同步热量数据并更新状态
    void Update();

    // 是否允许开火
    bool CanFire() const { return m_canFire && m_refereeValid; }

    // 射击速度比例 (0.0~1.0)，用于缩放拨盘电机转速
    float GetFireSpeedScale() const { return m_fireSpeedScale; }

    // 当前热量 / 热量上限
    float GetHeatRatio() const;
    float GetHeat() const { return m_estimatedHeat; }
    float GetHeatLimit() const { return m_heatLimit; }
    float GetCoolingRate() const { return m_coolingRate; }

    void SetSafetyMargin(float margin);

private:
    float m_estimatedHeat = 0.0f;     // 当前热量 Q1
    float m_heatLimit = 100.0f;       // 热量上限 Q0
    float m_coolingRate = 10.0f;      // 每秒冷却值

    float m_safetyMargin = 0.90f;     // 安全阈值，超过此比例禁射
    float m_resumeThreshold = 0.50f;  // 恢复阈值，冷却到此比例以下恢复满速

    float m_fireSpeedScale = 1.0f;
    bool m_canFire = true;
    bool m_refereeValid = false;

    static constexpr float kMinFireSpeedScale = 0.30f;
    static constexpr float kDefaultHeatLimit = 100.0f;
    static constexpr float kDefaultCoolingRate = 10.0f;
};

extern HeatLimiter heatLimiter;
