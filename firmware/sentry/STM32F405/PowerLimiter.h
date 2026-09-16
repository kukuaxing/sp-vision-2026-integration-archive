/**
 * @file PowerLimiter.h
 * @brief RLS-based chassis power limiter.
 */
#pragma once

#include "stm32f4xx.h"
#include <cmath>
#include <cstring>
#include "motor.h"

namespace Math {
    template <unsigned int R, unsigned int C>
    struct Matrix {
        float data[R][C];

        Matrix() { memset(data, 0, sizeof(data)); }

        static Matrix zeros() { return Matrix(); }

        template <unsigned int C2>
        Matrix<R, C2> operator*(const Matrix<C, C2>& other) {
            Matrix<R, C2> res;
            for (unsigned int i = 0; i < R; i++) {
                for (unsigned int j = 0; j < C2; j++) {
                    for (unsigned int k = 0; k < C; k++) {
                        res.data[i][j] += data[i][k] * other.data[k][j];
                    }
                }
            }
            return res;
        }

        Matrix<R, C> operator+(const Matrix<R, C>& other) {
            Matrix<R, C> res;
            for (unsigned int i = 0; i < R; i++) {
                for (unsigned int j = 0; j < C; j++) {
                    res.data[i][j] = data[i][j] + other.data[i][j];
                }
            }
            return res;
        }

        Matrix<R, C> operator-(const Matrix<R, C>& other) {
            Matrix<R, C> res;
            for (unsigned int i = 0; i < R; i++) {
                for (unsigned int j = 0; j < C; j++) {
                    res.data[i][j] = data[i][j] - other.data[i][j];
                }
            }
            return res;
        }

        Matrix<R, C> operator*(float scalar) {
            Matrix<R, C> res;
            for (unsigned int i = 0; i < R; i++) {
                for (unsigned int j = 0; j < C; j++) {
                    res.data[i][j] = data[i][j] * scalar;
                }
            }
            return res;
        }

        Matrix<R, C> operator/(float scalar) {
            return (*this) * (1.0f / scalar);
        }

        Matrix<C, R> trans() {
            Matrix<C, R> res;
            for (unsigned int i = 0; i < R; i++) {
                for (unsigned int j = 0; j < C; j++) {
                    res.data[j][i] = data[i][j];
                }
            }
            return res;
        }

        static Matrix<R, R> eye() {
            Matrix<R, R> res;
            for (unsigned int i = 0; i < R; i++) {
                res.data[i][i] = 1.0f;
            }
            return res;
        }
    };

    template <unsigned int Dim>
    class RLS {
    public:
        RLS(float delta_ = 1e-5f, float lambda_ = 0.999f)
            : lambda(lambda_), delta(delta_) {
            reset();
        }

        void reset() {
            P = Matrix<Dim, Dim>::eye() * delta;
            theta = Matrix<Dim, 1>::zeros();
        }

        void update(Matrix<Dim, 1> phi, float y) {
            Matrix<1, 1> denominator = phi.trans() * P * phi;
            float denom = lambda + denominator.data[0][0];
            Matrix<Dim, 1> K = (P * phi) / denom;

            Matrix<1, 1> prediction = phi.trans() * theta;
            float error = y - prediction.data[0][0];
            theta = theta + K * error;

            P = (P - K * phi.trans() * P) / lambda;
        }

        float get_k1() { return theta.data[0][0]; }
        float get_k2() { return theta.data[1][0]; }

        void set_params(float k1, float k2) {
            theta.data[0][0] = k1;
            theta.data[1][0] = k2;
        }

    private:
        float lambda;
        float delta;
        Matrix<Dim, Dim> P;
        Matrix<Dim, 1> theta;
    };
}

class PowerLimiter {
public:
    struct ModelDebugData {
        float sum_torque_power;
        float k1_sum_abs_w;
        float k2_sum_current_sq;
        float k3;
        float predicted_power_raw;
        float predicted_power_with_safety;
        float real_power_feedback;
    };

    PowerLimiter();
    void Init();
    void Process(Motor* chassis_motors[4], float real_power_feedback, float limit_power,
        bool feedback_is_fresh);
    float GetPredictedPower() const;
    ModelDebugData GetModelDebugData() const;

private:
    Math::RLS<2> rls;

    // Model: P = tau*w + k1*|w| + k2*tau^2 + k3
    float k1;
    float k2;
    float k3;
    float last_scale;
    float power_correction_;
    float last_limit_power_;
    float filtered_power_feedback_;
    bool has_filtered_power_feedback_;
    float last_avg_setspeed_rpm_;
    uint16_t spin_start_frames_;
    float last_predicted_power_;
    ModelDebugData last_model_debug_;

    float clamp(float val, float min, float max);
};

extern PowerLimiter power_limiter;
