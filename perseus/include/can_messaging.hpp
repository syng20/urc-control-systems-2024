#pragma once
#include <libhal-arm-mcu/stm32_generic/quadrature_encoder.hpp>
#include <libhal/pointers.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/units.hpp>

#include <bldc_servo.hpp>

namespace sjsu::perseus {
    
class can_perseus 
{

public: 
    can_perseus(hal::u16 curr_servo_addr); 


    enum class action : uint32_t
    {
    // top priority
    freeze = 0x0C,  // hard stop the servo to be 0
    heartbeat = 0x0E, // are you alive
    // actuators
    homing = 0x111, 
    set_position = 0x12,
    // readers
    read_position = 0x20,
    read_velocity = 0x21,
    // setters
    set_pid_position = 0x31,
    set_pid_velocity = 0x32,
    // servo to servo 
    // send_current_actual_position = 0x40, 
    };
    enum servo_address : hal::u16
    {
    track_servo = 0x120,
    shoulder_servo = 0x121,
    elbow_servo = 0x122,
    wrist_pitch = 0x123,
    wrist_roll = 0x124,
    clamp = 0x125
    };

    void set_curr_servo_addr(hal::u16 servo_addr); 

    void print_can_message(hal::serial& p_console,
                        hal::can_message const& p_message); 
    float fixed_to_floating_point(hal::byte msb, hal::byte lsb, int exponent); 
    hal::u16 floating_to_fixed_point(float n, int exponent); 
    void process_can_message(hal::can_message const& p_message,
        hal::v5::strong_ptr<bldc_perseus> bldc,
        hal::v5::strong_ptr<hal::can_message> response);

private: 
    hal::u16 m_curr_servo_addr; 
}; 
} // namespace sjsu::perseus