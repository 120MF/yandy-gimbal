#include <YandyGimbalNode.hpp>
#include <YandyGimbalData.hpp>

#include <OF/lib/VtHub/VtHub.hpp>
#include "RPL/Packets/VT03RemotePacket.hpp"

#include <zephyr/logging/log.h>
#include <cmath>

LOG_MODULE_REGISTER(YandyGimbalNode, CONFIG_YANDY_GIMBAL_LOG_LEVEL);

using namespace OF;
using one::motor::dji::PosAngMode;

// Register Topic for data communication
ONE_TOPIC_REGISTER(YandyGimbalData, topic_yandy_gimbal, "yandy_gimbal_data");

static constexpr led_color c_normal = COLOR_HEX("#00ff88");
static constexpr led_color c_warning = COLOR_HEX("#ff6600");
static constexpr led_color c_calibrating = COLOR_HEX("#ffff00");

void YandyGimbalNode::setServoPosition(const pwm_dt_spec* servo, float normalized_pos) {
    if (servo == nullptr || servo->dev == nullptr) {
        return;
    }
    
    // Clamp to 0-1 range
    normalized_pos = std::clamp(normalized_pos, 0.0f, 1.0f);
    
    // JX Servo PDI-6225MG: 500-2500us pulse for 300 degrees
    static constexpr ServoConfig cfg{};
    const auto pulse_us = static_cast<uint32_t>(
        cfg.min_pulse_us + normalized_pos * (cfg.max_pulse_us - cfg.min_pulse_us)
    );
    
    // PWM period is 20ms (50Hz), convert pulse to nanoseconds
    uint32_t pulse_ns = pulse_us * 1000;
    pwm_set_pulse_dt(servo, pulse_ns);
}

bool YandyGimbalNode::calibrateZero() {
    LOG_INF("Starting zero calibration...");
    m_led_guard.set({c_calibrating, LEDMode::Blink, 1, 100});
    
    // Set a small negative target to push motor to mechanical limit
    m_lift_motor.setPosRef(ZERO_CALIBRATION_TARGET);
    
    float last_position = 0.0f;
    int stall_count = 0;
    
    // Wait for motor to reach mechanical limit (stall detection)
    for (int i = 0; i < 500; ++i) {  // Max 5 seconds
        k_sleep(K_MSEC(10));
        
        auto status = m_lift_motor.getStatusPlain();
        float current_pos = status.reduced_angle_rad;
        float current_mA = std::abs(status.real_current_mA);
        
        // Check for stall: high current and position not changing
        if (current_mA > STALL_CURRENT_THRESHOLD && 
            std::abs(current_pos - last_position) < STALL_POSITION_TOLERANCE) {
            stall_count++;
            if (stall_count >= STALL_DETECTION_COUNT) {
                LOG_INF("Stall detected at position %.2f, current %.0f mA", 
                        static_cast<double>(current_pos), static_cast<double>(current_mA));
                
                // Set current position as zero by recording offset
                // The motor's internal offset will be updated on next enable
                m_motor_target_pos = 0.0f;
                m_lift_motor.setPosRef(0.0f);
                m_is_zeroed = true;
                
                LOG_INF("Zero calibration complete");
                return true;
            }
        } else {
            stall_count = 0;
        }
        
        last_position = current_pos;
    }
    
    LOG_WRN("Zero calibration timeout");
    return false;
}

bool YandyGimbalNode::init() {
    LOG_INF("Initializing gimbal node");
    
    if (config.can_dev == nullptr) {
        LOG_ERR("CAN device not configured");
        return false;
    }
    
    // Initialize CAN driver
    if (const auto result = m_driver.init(config.can_dev); !result) {
        LOG_ERR("Failed to init CAN driver");
        return false;
    }
    
    // Initialize M2006 motor with PosAngMode
    if (const auto result = m_lift_motor.init(m_driver,
            one::motor::dji::Param{5, PosAngMode{g_gimbal_pos_params, g_gimbal_ang_params}}); 
        !result) {
        LOG_ERR("Failed to init lift motor");
        return false;
    }
    
    m_lift_motor.setPosRef(0.0f);
    
    // Enable motor
    if (const auto result = m_lift_motor.enable(); !result) {
        LOG_ERR("Failed to enable lift motor");
        return false;
    }
    
    // Initialize servos to center position
    if (config.servo1 != nullptr && device_is_ready(config.servo1->dev)) {
        setServoPosition(config.servo1, 0.5f);
        LOG_INF("Servo 1 initialized");
    } else {
        LOG_WRN("Servo 1 not ready");
    }
    
    if (config.servo2 != nullptr && device_is_ready(config.servo2->dev)) {
        setServoPosition(config.servo2, 0.5f);
        LOG_INF("Servo 2 initialized");
    } else {
        LOG_WRN("Servo 2 not ready");
    }
    
    LOG_INF("Gimbal node initialized");
    return true;
}

void YandyGimbalNode::run() {
    LOG_INF("Running gimbal node");
    
    // Perform zero calibration at startup
    if (!calibrateZero()) {
        LOG_WRN("Calibration failed, using current position as zero");
        m_is_zeroed = true;
        m_motor_target_pos = 0.0f;
    }
    
    while (true) {
        k_sleep(K_MSEC(10));
        
        auto state = VtHub::get<VT03RemotePacket>();
        
        // Check connection and switch state (3 = enabled)
        if (!state || state.value().switch_state != 2) {  // switch_state: 0=down, 1=mid, 2=up
            m_led_guard.set({c_warning, LEDMode::Breathing, 1, 300});
            
            // Hold current position when disabled
            m_lift_motor.setPosRef(m_motor_target_pos);
            k_sleep(K_MSEC(100));
            continue;
        }
        
        m_led_guard.set({c_normal, LEDMode::Breathing, 1, 500});
        
        auto data = state.value();
        
        // Left stick X -> Servo 1 (map from 364-1684 to 0-1)
        float servo1_input = vt_stick_percent(data.left_stick_x);
        m_servo1_pos = (servo1_input + 1.0f) / 2.0f;  // Convert -1..1 to 0..1
        setServoPosition(config.servo1, m_servo1_pos);
        
        // Left stick Y -> Servo 2
        float servo2_input = vt_stick_percent(data.left_stick_y);
        m_servo2_pos = (servo2_input + 1.0f) / 2.0f;
        setServoPosition(config.servo2, m_servo2_pos);
        
        // Wheel -> Motor position (accumulate)
        // Use wheel as velocity control for height
        // value > center -> move up, value < center -> move down
        float wheel_velocity = vt_stick_percent(data.wheel);
        
        // Deadzone handled by vt_stick_percent (implied linear mapping) but adding small threshold good practice
        if (std::abs(wheel_velocity) < 0.1f) wheel_velocity = 0.0f;
        
        // Accumulate position: velocity * scale
        // Scale factor: max speed ~2.0 rad/s * 0.01s loop time = 0.02
        m_motor_target_pos += wheel_velocity * 0.05f;
        
        // Clamp to valid range
        m_motor_target_pos = std::clamp(m_motor_target_pos, 0.0f, MAX_MOTOR_POS);
        
        m_lift_motor.setPosRef(m_motor_target_pos);
        
        // Publish gimbal state
        auto status = m_lift_motor.getStatusPlain();
        topic_yandy_gimbal.write({
            static_cast<int>(status.reduced_angle_rad * 100)  // Position in centi-radians
        });
    }
}

// Register the Node with OneFramework
ONE_NODE_REGISTER(YandyGimbalNode);

