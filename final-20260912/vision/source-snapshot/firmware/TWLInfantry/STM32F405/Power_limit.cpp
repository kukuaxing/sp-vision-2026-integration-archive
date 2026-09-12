#include "Power_limit.h"

#include "can.h"
#include "judgement.h"
#include "supercap.h"

PowerLimiter powerLimiter;

namespace
{
constexpr float kMinConfiguredPower = 15.0f;
constexpr float kJudgeFallbackPower = 45.0f;
constexpr float kMaxCapPowerOut = 220.0f;
constexpr float kCapEnergyFullTarget = 1200.0f;
constexpr float kCapEnergyBaseTarget = 600.0f;
constexpr float kRefereeBufferFullTarget = 60.0f;
constexpr float kRefereeBufferBaseTarget = 50.0f;
constexpr float kMinRlsPower = 5.0f;
constexpr float kMinRlsSampleOmega = 0.5f;
constexpr float kMinRlsSampleTorque = 0.01f;
constexpr float kIdleOmegaThreshold = 0.5f;
constexpr float kIdleTorqueThreshold = 0.05f;
constexpr float kStaticLossAlpha = 0.005f;
constexpr float kParamLowerBound = 1e-5f;
}

float PowerLimiter::CurrentToTorque(float current)
{
    const float ampere = current * M3508_CURRENT_TO_AMPERE;
    return ampere * M3508_TORQUE_CONSTANT;
}

float PowerLimiter::TorqueToCurrent(float torque)
{
    const float ampere = torque / M3508_TORQUE_CONSTANT;
    return ampere / M3508_CURRENT_TO_AMPERE;
}

float PowerLimiter::Clamp(float value, float minVal, float maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

float PowerLimiter::CalcMotorPower(float torque, float av) const
{
    return torque * av + m_k1 * fabsf(av) + m_k2 * torque * torque + m_k3 / 4.0f;
}

bool PowerLimiter::IsMotorConnected(int index) const
{
    return m_motors[index] != nullptr && m_motors[index]->getStatus() == FINE;
}

void PowerLimiter::Init(Motor* motors[4], float maxPower, float k1, float k2, float k3)
{
    for (int i = 0; i < 4; ++i)
        m_motors[i] = motors[i];

    m_userMaxPower = maxPower;
    m_maxPower = maxPower;
    m_lastJudgePowerLimit = maxPower;

    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;

    m_theta1 = k1;
    m_theta2 = k2;
    m_p11 = 1000.0f;
    m_p12 = 0.0f;
    m_p21 = 0.0f;
    m_p22 = 1000.0f;
    m_lambda = 0.999f;

    m_estimatedPower = 0.0f;
    m_measuredPower = 0.0f;
    m_rlsEnabled = true;
    m_userOverrideEnabled = false;
    m_initialized = true;
}

void PowerLimiter::SetMaxPower(float maxPower)
{
    m_userMaxPower = maxPower;
    m_userOverrideEnabled = true;
}

void PowerLimiter::SetModelParams(float k1, float k2, float k3)
{
    m_k1 = k1;
    m_k2 = k2;
    m_k3 = k3;
    m_theta1 = k1;
    m_theta2 = k2;
}

float PowerLimiter::ReadMeasuredPower() const
{
    if (supercap.connect)
    {
        const float capPower = supercap.Rxsuper.U * supercap.Rxsuper.I;
        if (capPower > 0.0f)
            return capPower;
    }

    if (judgement.powerheatready && judgement.data.power_heat_data_t.reserved2 > 0.0f)
        return judgement.data.power_heat_data_t.reserved2;

    return m_estimatedPower;
}

float PowerLimiter::ReadJudgePowerLimit() const
{
    if (judgement.judgementready && judgement.data.robot_status_t.chassis_power_limit > 0U)
        return static_cast<float>(judgement.data.robot_status_t.chassis_power_limit);

    return m_lastJudgePowerLimit;
}

void PowerLimiter::UpdatePowerLimitFromFeedback()
{
    const float judgeLimit = ReadJudgePowerLimit();
    if (judgeLimit > 0.0f)
        m_lastJudgePowerLimit = judgeLimit;

    m_refereePowerLimit = m_lastJudgePowerLimit > 0.0f ? m_lastJudgePowerLimit : kJudgeFallbackPower;

    float fullTarget = kRefereeBufferFullTarget;
    float baseTarget = kRefereeBufferBaseTarget;
    float energyFeedback = 0.0f;
    float maxExtraPower = 0.0f;

    if (supercap.connect && supercap.Rxsuper.cap_energy > 0.0f)
    {
        energyFeedback = supercap.Rxsuper.cap_energy;
        fullTarget = kCapEnergyFullTarget;
        baseTarget = kCapEnergyBaseTarget;
        maxExtraPower = kMaxCapPowerOut;
    }
    else if (judgement.powerheatready)
    {
        energyFeedback = static_cast<float>(judgement.data.power_heat_data_t.buffer_energy);
    }

    m_energyFeedback = energyFeedback;

    if (energyFeedback > 0.0f)
    {
        const float sqrtEnergy = sqrtf(fmaxf(energyFeedback, 0.0f));
        const float fullError = sqrtf(fullTarget) - sqrtEnergy;
        const float baseError = sqrtf(baseTarget) - sqrtEnergy;
        const float limitCeiling = m_refereePowerLimit + maxExtraPower;

        m_fullMaxPower = Clamp(m_refereePowerLimit - m_energyLoopKp * fullError, kMinConfiguredPower, limitCeiling);
        m_baseMaxPower = Clamp(m_refereePowerLimit - m_energyLoopKp * baseError, kMinConfiguredPower, limitCeiling);
    }
    else
    {
        m_fullMaxPower = m_refereePowerLimit;
        m_baseMaxPower = m_refereePowerLimit;
    }

    const float requestedPower = m_userOverrideEnabled ? m_userMaxPower : m_refereePowerLimit;
    const float dynamicMinPower = fminf(m_fullMaxPower, m_baseMaxPower);
    const float dynamicMaxPower = fmaxf(m_fullMaxPower, m_baseMaxPower);
    m_maxPower = Clamp(requestedPower, dynamicMinPower, dynamicMaxPower);
}

void PowerLimiter::UpdateStaticLoss(float measuredPower, float sumAbsOmega, float sumTorqueSq)
{
    if (measuredPower <= 0.0f)
        return;

    if (sumAbsOmega < kIdleOmegaThreshold && sumTorqueSq < kIdleTorqueThreshold)
        m_k3 = (1.0f - kStaticLossAlpha) * m_k3 + kStaticLossAlpha * measuredPower;
}

void PowerLimiter::UpdateRLS(float measuredPower, float effectivePower, float sumAbsOmega, float sumTorqueSq)
{
    if (!m_rlsEnabled)
        return;

    if (measuredPower < kMinRlsPower)
        return;

    if (sumAbsOmega < kMinRlsSampleOmega && sumTorqueSq < kMinRlsSampleTorque)
        return;

    const float y = measuredPower - effectivePower - m_k3;
    const float x1 = sumAbsOmega;
    const float x2 = sumTorqueSq;

    const float px1 = m_p11 * x1 + m_p12 * x2;
    const float px2 = m_p21 * x1 + m_p22 * x2;
    const float denom = m_lambda + x1 * px1 + x2 * px2;
    if (fabsf(denom) < 1e-6f)
        return;

    const float k1Gain = px1 / denom;
    const float k2Gain = px2 / denom;
    const float estimate = m_theta1 * x1 + m_theta2 * x2;
    const float error = y - estimate;

    m_theta1 += k1Gain * error;
    m_theta2 += k2Gain * error;

    const float oldP11 = m_p11;
    const float oldP12 = m_p12;
    const float oldP21 = m_p21;
    const float oldP22 = m_p22;

    m_p11 = (oldP11 - k1Gain * (x1 * oldP11 + x2 * oldP21)) / m_lambda;
    m_p12 = (oldP12 - k1Gain * (x1 * oldP12 + x2 * oldP22)) / m_lambda;
    m_p21 = (oldP21 - k2Gain * (x1 * oldP11 + x2 * oldP21)) / m_lambda;
    m_p22 = (oldP22 - k2Gain * (x1 * oldP12 + x2 * oldP22)) / m_lambda;

    m_k1 = fmaxf(m_theta1, kParamLowerBound);
    m_k2 = fmaxf(m_theta2, kParamLowerBound);
}

void PowerLimiter::ApplyToMotors(CAN& can)
{
    if (!m_initialized)
        return;

    float pidOutputs[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < 4; ++i)
    {
        if (m_motors[i] != nullptr)
            pidOutputs[i] = static_cast<float>(m_motors[i]->current);
    }

    float limitedOutputs[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    LimitOutput(pidOutputs, limitedOutputs);

    for (int i = 0; i < 4; ++i)
    {
        if (m_motors[i] == nullptr)
            continue;

        m_motors[i]->current = static_cast<int32_t>(limitedOutputs[i]);

        const uint32_t idx = m_motors[i]->ID - 0x205;
        if (idx < 4U)
        {
            can.temp_data[idx * 2] = static_cast<uint8_t>((m_motors[i]->current & 0xff00) >> 8);
            can.temp_data[idx * 2 + 1] = static_cast<uint8_t>(m_motors[i]->current & 0x00ff);
        }
    }
}

void PowerLimiter::LimitOutput(float pidOutputs[4], float limitedOutputs[4])
{
    if (!m_initialized)
    {
        for (int i = 0; i < 4; ++i)
            limitedOutputs[i] = pidOutputs[i];
        return;
    }

    UpdatePowerLimitFromFeedback();

    float cmdPower[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float curAv[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float setAv[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float error[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    float sumCmdPower = 0.0f;
    float sumError = 0.0f;

    float effectivePower = 0.0f;
    float sumAbsOmega = 0.0f;
    float sumTorqueSq = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        if (!IsMotorConnected(i))
        {
            limitedOutputs[i] = 0.0f;
            continue;
        }

        curAv[i] = MotorRpmToOutputAv(static_cast<float>(m_motors[i]->curspeed));
        setAv[i] = MotorRpmToOutputAv(static_cast<float>(m_motors[i]->setspeed));

        const float cmdTorque = CurrentToTorque(pidOutputs[i]);
        const float feedbackTorque = CurrentToTorque(static_cast<float>(m_motors[i]->torque_current));

        cmdPower[i] = CalcMotorPower(cmdTorque, curAv[i]);
        error[i] = fabsf(setAv[i] - curAv[i]);
        sumError += error[i];

        if (cmdPower[i] > 0.0f)
            sumCmdPower += cmdPower[i];

        effectivePower += feedbackTorque * curAv[i];
        sumAbsOmega += fabsf(curAv[i]);
        sumTorqueSq += feedbackTorque * feedbackTorque;
    }

    m_estimatedPower = effectivePower + m_k1 * sumAbsOmega + m_k2 * sumTorqueSq + m_k3;
    m_measuredPower = ReadMeasuredPower();

    UpdateStaticLoss(m_measuredPower, sumAbsOmega, sumTorqueSq);
    UpdateRLS(m_measuredPower, effectivePower, sumAbsOmega, sumTorqueSq);

    if (sumCmdPower <= m_maxPower)
    {
        for (int i = 0; i < 4; ++i)
        {
            if (IsMotorConnected(i))
            {
                limitedOutputs[i] = Clamp(pidOutputs[i],
                    -static_cast<float>(m_motors[i]->maxcurrent),
                    static_cast<float>(m_motors[i]->maxcurrent));
            }
            else
            {
                limitedOutputs[i] = 0.0f;
            }
        }
        return;
    }

    float errorConfidence = 0.0f;
    if (sumError > ERROR_DISTRIBUTION_UPPER)
    {
        errorConfidence = 1.0f;
    }
    else if (sumError > ERROR_DISTRIBUTION_LOWER)
    {
        errorConfidence = (sumError - ERROR_DISTRIBUTION_LOWER) /
            (ERROR_DISTRIBUTION_UPPER - ERROR_DISTRIBUTION_LOWER);
    }

    float allocatablePower = m_maxPower;
    float sumPowerRequired = 0.0f;

    for (int i = 0; i < 4; ++i)
    {
        if (!IsMotorConnected(i))
            continue;

        if (cmdPower[i] <= 0.0f)
            allocatablePower += -cmdPower[i];
        else
            sumPowerRequired += cmdPower[i];
    }

    for (int i = 0; i < 4; ++i)
    {
        if (!IsMotorConnected(i))
        {
            limitedOutputs[i] = 0.0f;
            continue;
        }

        if (cmdPower[i] <= 0.0f)
        {
            limitedOutputs[i] = Clamp(pidOutputs[i],
                -static_cast<float>(m_motors[i]->maxcurrent),
                static_cast<float>(m_motors[i]->maxcurrent));
            continue;
        }

        const float powerWeightError = (sumError > 1e-3f) ? (error[i] / sumError) : 0.25f;
        const float powerWeightProp = (sumPowerRequired > 1e-3f) ? (cmdPower[i] / sumPowerRequired) : 0.25f;
        const float powerWeight = errorConfidence * powerWeightError + (1.0f - errorConfidence) * powerWeightProp;
        const float allocatedPower = powerWeight * allocatablePower;

        const float A = m_k2;
        const float B = curAv[i];
        const float C = m_k1 * fabsf(curAv[i]) + m_k3 / 4.0f - allocatedPower;
        const float delta = B * B - 4.0f * A * C;

        float maxTorque = 0.0f;
        if (delta <= 0.0f || fabsf(A) < 1e-6f)
        {
            maxTorque = (fabsf(A) < 1e-6f) ? 0.0f : (-B / (2.0f * A));
        }
        else
        {
            const float sqrtDelta = sqrtf(delta);
            maxTorque = (pidOutputs[i] >= 0.0f) ? ((-B + sqrtDelta) / (2.0f * A))
                                                : ((-B - sqrtDelta) / (2.0f * A));
        }

        const float maxCurrent = fabsf(TorqueToCurrent(maxTorque));
        if (pidOutputs[i] >= 0.0f)
            limitedOutputs[i] = Clamp(pidOutputs[i], 0.0f, maxCurrent);
        else
            limitedOutputs[i] = Clamp(pidOutputs[i], -maxCurrent, 0.0f);

        limitedOutputs[i] = Clamp(limitedOutputs[i],
            -static_cast<float>(m_motors[i]->maxcurrent),
            static_cast<float>(m_motors[i]->maxcurrent));
    }
}
