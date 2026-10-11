#pragma once
#include "module_data_types.hpp"

namespace stallion {

// ============================================================================
// ESTIMATOR INTERFACES
// ============================================================================

class IEstimator {
public:
    virtual ~IEstimator() = default;
    virtual bool update(uint64_t now_us) = 0;
    virtual StateEstimate state() const = 0;
    virtual bool is_healthy() const = 0;
    virtual void reset() = 0;
};

class IAttitudeEstimator : public IEstimator {
public:
    virtual ~IAttitudeEstimator() = default;
    virtual bool predict(float dt, float gx, float gy, float gz) = 0;
    virtual bool update_accel(float ax, float ay, float az) = 0;
    virtual bool update_mag(float mx, float my, float mz) = 0;
};

// ============================================================================
// CONTROLLER INTERFACES
// ============================================================================

class IAttitudeController {
public:
    virtual ~IAttitudeController() = default;

    virtual bool update(const StateEstimate& state,
                        const ControlSetpoint& setpoint,
                        float dt) = 0;

    virtual ControlOutput get_output() const = 0;
    virtual bool is_healthy() const = 0;
    virtual void reset() = 0;
};

// Legacy compatibility
class IController {
public:
    virtual ~IController() = default;

    virtual bool update(const StateEstimate& state,
                        const AttitudeSetpoint& sp,
                        float dt) = 0;

    virtual ControlOutput get_output() const = 0;
    virtual bool is_healthy() const = 0;
    virtual void reset() = 0;
};

} // namespace stallion