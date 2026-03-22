#ifndef YANDYGIMBALNODE_HPP
#define YANDYGIMBALNODE_HPP

#include <OF/lib/Node/Node.hpp>
#include <OF/lib/NotifyHub/NotifyGuard.hpp>
#include <one/can/CanDriver.hpp>
#include <one/motor/dji/DjiMotor.hpp>
#include <one/PID/PidParams.hpp>
#include <one/PID/PidChain.hpp>
#include <zephyr/drivers/pwm.h>

using namespace OF;
using one::pid::PidParams;
using one::pid::PidChain;
using one::motor::dji::M2006;
using one::can::CanDriver;

// M2006 PosAngMode PID parameters
static constexpr PidParams<> g_gimbal_pos_params{
    .Kp = 30,
    .Ki = 0.002,
    .Kd = 0.03,
    .MaxOutput = 15000,
    .Deadband = 0.005,
    .IntegralLimit = 800,
};

static constexpr PidParams<> g_gimbal_ang_params{
    .Kp = 8,
    .Ki = 0.0,
    .Kd = 0.0,
    .MaxOutput = 10000,
    .Deadband = 0.001,
    .IntegralLimit = 100,
};

// JX Servo PDI-6225MG: 300 degree range, 500-2500us pulse width
struct ServoConfig
{
    uint32_t min_pulse_us = 500; // 0 degrees
    uint32_t max_pulse_us = 2500; // 300 degrees
    float angle_range_deg = 300.0f;
};

class YandyGimbalNode : public Node<YandyGimbalNode>
{
public:
    struct Meta
    {
        static constexpr size_t stack_size = 4096;
        static constexpr int priority = 2;
        static constexpr const char* name = "yandy_gimbal";
    };

    struct Config
    {
        const device* can_dev;
        const pwm_dt_spec* servo1;
        const pwm_dt_spec* servo2;
    };

    inline static Config config = {};

    bool init();
    void run();

    void cleanup()
    {
    }

private:
    // Motor
    CanDriver m_driver;
    M2006 m_lift_motor;

    // Motor state
    float m_motor_target_pos = 0.0f;
    bool m_is_zeroed = false;

    // Servo state (0.0 - 1.0 normalized)
    float m_servo1_pos = 0.8f;
    float m_servo2_pos = 0.1f;

    // LED guard
    NotifyGuard<LEDStatus> m_led_guard{"gimbal"};

    // Constants
    static constexpr float MAX_MOTOR_POS = 30.0f; // rad
    static constexpr float ZERO_CALIBRATION_TARGET = -2.0f; // rad
    static constexpr float STALL_CURRENT_THRESHOLD = 3000.0f; // mA
    static constexpr float STALL_POSITION_TOLERANCE = 0.05f; // rad
    static constexpr int STALL_DETECTION_COUNT = 10; // consecutive frames

    // Helper functions
    static void setServoPosition(const pwm_dt_spec* servo, float normalized_pos, uint32_t min_pulse_us,
                                 uint32_t max_pulse_us);
    bool calibrateZero();
};

#endif // YANDYGIMBALNODE_HPP
