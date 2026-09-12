#include "Heat_limit.h"
#include "judgement.h"

HeatLimiter heatLimiter;

namespace
{
constexpr float kHeatPerBullet = 10.0f;
}

void HeatLimiter::Init(float safetyMargin, float resumeThreshold)
{
    m_safetyMargin = safetyMargin;
    m_resumeThreshold = resumeThreshold;
    m_heatLimit = kDefaultHeatLimit;
    m_coolingRate = kDefaultCoolingRate;
    m_estimatedHeat = 0.0f;
    m_fireSpeedScale = 1.0f;
    m_canFire = true;
    m_refereeValid = false;
}

void HeatLimiter::Update()
{
    // 从裁判系统同步数据
    if (judgement.powerheatready && judgement.judgementready)
    {
        m_refereeValid = true;

        m_heatLimit = static_cast<float>(
            judgement.data.robot_status_t.shooter_barrel_heat_limit);
        m_coolingRate = static_cast<float>(
            judgement.data.robot_status_t.shooter_barrel_cooling_value);

        // 裁判系统的热量值是权威数据，直接同步
        m_estimatedHeat = static_cast<float>(
            judgement.data.power_heat_data_t.shooter_17mm_1_barrel_heat);
    }
    else
    {
        m_refereeValid = false;
        m_canFire = false;
        m_fireSpeedScale = 0.0f;
        return;
    }

    // 保护：避免零值导致除零
    if (m_heatLimit <= 0.0f)
        m_heatLimit = kDefaultHeatLimit;
    if (m_coolingRate <= 0.0f)
        m_coolingRate = kDefaultCoolingRate;

    const float heatRatio = m_estimatedHeat / m_heatLimit;

    // 每发弹丸增加 10 热量，安全阈值预留至少一发弹丸的余量
    const float effectiveSafety = m_safetyMargin - kHeatPerBullet / m_heatLimit;

    if (m_estimatedHeat >= m_heatLimit * effectiveSafety)
    {
        // 热量接近上限，禁止开火，等待冷却
        m_canFire = false;
        m_fireSpeedScale = 0.0f;
    }
    else if (m_estimatedHeat <= m_heatLimit * m_resumeThreshold)
    {
        // 热量已充分冷却，恢复满速
        m_canFire = true;
        m_fireSpeedScale = 1.0f;
    }
    else
    {
        // 中间区间：允许开火但按比例降速
        // 保持 canFire 上次状态（滞回），避免临界振荡
        const float range = effectiveSafety - m_resumeThreshold;
        if (range > 0.01f)
        {
            const float position = (heatRatio - m_resumeThreshold) / range;
            m_fireSpeedScale = 1.0f - position * (1.0f - kMinFireSpeedScale);
            if (m_fireSpeedScale < kMinFireSpeedScale)
                m_fireSpeedScale = kMinFireSpeedScale;
            if (m_fireSpeedScale > 1.0f)
                m_fireSpeedScale = 1.0f;
        }
        else
        {
            m_fireSpeedScale = kMinFireSpeedScale;
        }
    }
}

float HeatLimiter::GetHeatRatio() const
{
    if (m_heatLimit > 0.0f)
        return m_estimatedHeat / m_heatLimit;
    return 0.0f;
}

void HeatLimiter::SetSafetyMargin(float margin)
{
    if (margin > 0.0f && margin < 1.0f)
        m_safetyMargin = margin;
}
