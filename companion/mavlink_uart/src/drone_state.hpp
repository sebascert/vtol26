//Esto representa el estado actual del dron que llega desde PX4:

#pragma once

#include <cstdint>

struct DroneState
{
    //ATTITUDE
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    
    //GPS
    float lat = 0;
    float lon = 0;
    float alt = 0;

    //BAROMETER
    float pressure = 0;
    
    //IMU
    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;

    //PITOT
    float airspeed = 0;
   
    // GYRO Vel. Ang. Dron.
    float gyro_x = 0;
    float gyro_y = 0;
    float gyro_z = 0;

    // RC
    uint16_t rc_channels[16] = {0};

    //Flight mode
    uint8_t flight_mode = 0;

    //Time Stamp
    uint64_t timestamp_us = 0;
};
