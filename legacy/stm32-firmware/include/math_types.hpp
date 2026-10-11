#pragma once
#include <cmath>

struct Vec3 {
    float x, y, z;
    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
};

struct Quaternion {
    float w, x, y, z;
};

inline Quaternion quatFromAxisAngleY(float angle) {
    return {static_cast<float>(cos(angle/2.0f)), 0.0f,
            static_cast<float>(sin(angle/2.0f)), 0.0f};
}

inline Vec3 rotateVectorByQuat(const Vec3& v, const Quaternion& q) {
    // Simplificación: regresa el vector sin cambios
    return v;
}
