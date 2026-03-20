#ifndef YANDYGIMBALDATA_HPP_
#define YANDYGIMBALDATA_HPP_

struct YandyGimbalData {
    float gimbal_z;      // Motor position in radians
    float gimbal_yaw;    // Yaw angle (from servo1) in radians
    float gimbal_pitch;  // Pitch angle (from servo2) in radians
};

#endif // YANDYGIMBALDATA_HPP_
