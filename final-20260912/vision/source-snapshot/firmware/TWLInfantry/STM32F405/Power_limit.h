#pragma once

#include <cmath>
#include <cstdint>
#include "motor.h"

class CAN;

// M3508 output-side parameters used by the power model.
#define M3508_TORQUE_CONSTANT   0.3f
#define M3508_REDUCTION_RATIO   19.0f
#define M3508_CURRENT_TO_AMPERE (20.0f / 16384.0f)

#define ERROR_DISTRIBUTION_UPPER 20.0f
#define ERROR_DISTRIBUTION_LOWER 15.0f

struct PowerObj
{
    float pidOutput;
    float curAv;
    float setAv;
    float maxOutput;
};

class PowerLimiter
{
public:
    void Init(Motor* motors[4], float maxPower, float k1 = 0.22f, float k2 = 1.2f, float k3 = 2.78f);
    void LimitOutput(float pidOutputs[4], float limitedOutputs[4]);
    void ApplyToMotors(CAN& can);

    void SetMaxPower(float maxPower);
    float GetMaxPower() const { return m_maxPower; }
    float GetEstimatedPower() const { return m_estimatedPower; }
    float GetMeasuredPower() const { return m_measuredPower; }
    float GetK1() const { return m_k1; }
    float GetK2() const { return m_k2; }
    float GetK3() const { return m_k3; }

    void SetModelParams(float k1, float k2, float k3);
    void SetRLSEnabled(bool enabled) { m_rlsEnabled = enabled; }
    bool IsMotorConnected(int index) const;

private:
    static float Rpm2Av(float rpm) { return rpm * 3.14159265f / 30.0f; }
    static float MotorRpmToOutputAv(float motorRpm) { return (motorRpm / M3508_REDUCTION_RATIO) * 3.14159265f / 30.0f; }
    static float Clamp(float value, float minVal, float maxVal);
    static float CurrentToTorque(float current);
    static float TorqueToCurrent(float torque);

    float CalcMotorPower(float torque, float av) const;
    float ReadMeasuredPower() const;
    float ReadJudgePowerLimit() const;
    void UpdatePowerLimitFromFeedback();
    void UpdateStaticLoss(float measuredPower, float sumAbsOmega, float sumTorqueSq);
    void UpdateRLS(float measuredPower, float effectivePower, float sumAbsOmega, float sumTorqueSq);

private:
    Motor* m_motors[4] = { nullptr, nullptr, nullptr, nullptr };

    float m_userMaxPower = 60.0f;
    float m_maxPower = 60.0f;
    float m_lastJudgePowerLimit = 60.0f;
    float m_refereePowerLimit = 60.0f;
    float m_fullMaxPower = 60.0f;
    float m_baseMaxPower = 60.0f;
    float m_energyFeedback = 0.0f;
    float m_estimatedPower = 0.0f;
    float m_measuredPower = 0.0f;

    float m_k1 = 0.22f;
    float m_k2 = 1.2f;
    float m_k3 = 2.78f;

    bool m_initialized = false;
    bool m_rlsEnabled = true;
    bool m_userOverrideEnabled = false;

    float m_energyLoopKp = 6.0f;

    // 2-parameter RLS state: theta = [k1, k2]^T.
    float m_theta1 = 0.22f;
    float m_theta2 = 1.2f;
    float m_p11 = 1000.0f;
    float m_p12 = 0.0f;
    float m_p21 = 0.0f;
    float m_p22 = 1000.0f;
    float m_lambda = 0.999f;
};

extern PowerLimiter powerLimiter;
