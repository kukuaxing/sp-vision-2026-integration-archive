/**
 * @brief 角度解算器
 * 将 [-180, 180] 范围的跳变角度转换为连续角度
 */
class AngleUnwrapper {
public:
    AngleUnwrapper() : last_angle_(0.0f), revolutions_(0), initialized_(false) {}

    /**
     * @brief 更新并获取连续角度
     * @param wrapped_angle 从陀螺仪读取的 [-180, 180] 角度
     * @return 连续角度 (例如 370°, 730° 等)
     */
    float unwrap(float wrapped_angle) {
        if (!initialized_) {
            last_angle_ = wrapped_angle;
            initialized_ = true;
            return wrapped_angle;
        }

        float delta = wrapped_angle - last_angle_;

        if (delta > 180.0f) {      // 从 +179° 跳到 -179° (大约)
            revolutions_--;
        }
        else if (delta < -180.0f) { // 从 -179° 跳到 +179° (大约)
            revolutions_++;
        }

        last_angle_ = wrapped_angle;
        return wrapped_angle + 360.0f * revolutions_;
    }

private:
    float last_angle_;
    int revolutions_;
    bool initialized_;
};
