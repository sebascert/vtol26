#pragma once

#include <cmath>
#include "module_data_types.hpp"
#include "module_interfaces.hpp"

namespace stallion {

class AttitudeComplementaryFilter : public IAttitudeEstimator {
public:
    AttitudeComplementaryFilter();
    ~AttitudeComplementaryFilter();
    
    bool update(uint64_t now_us) override;
    StateEstimate state() const override;
    bool is_healthy() const override;
    void reset() override;
    
    bool predict(float dt, float gx, float gy, float gz) override;
    bool update_accel(float ax, float ay, float az) override;
    bool update_mag(float mx, float my, float mz) override;
    
    void set_accel_weight(float w) { accel_weight_ = w; }
    void set_mag_weight(float w) { mag_weight_ = w; }
    
private:
    StateEstimate state_;
    uint32_t last_update_us_;
    bool is_healthy_;
    
    float accel_weight_;
    float mag_weight_;
    
    void quaternion_integrate(float gx, float gy, float gz, float dt);
    void quaternion_normalize();
    void accel_correct();
    void mag_correct();
    void quaternion_to_euler();
    
    float accel_x_, accel_y_, accel_z_;
    float mag_x_, mag_y_, mag_z_;
};

}